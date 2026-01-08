#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esecuzione demo Behavior Tree via XML (BehaviorTree.CPP-style) in Python + ROS2.

- Usa la libreria `behaviortree.tree.Tree` per istanziare 3 alberi da:
    * SRM1.xml
    * SRM2.xml
    * SRM_Supervisor.xml

- Collega i nodi Action/Condition ai corrispondenti metodi Python che:
    * pubblicano comandi sulle stesse topic del pick_and_place_demo.py
    * simulano le parti sensoriali (FindObj, CheckAlignment, NearObj, ecc.)
      con semplici SUCCESS (mock) o piccole attese

- Esegue i tre alberi in "parallelo concettuale" tickandoli in un loop ROS2.

NB: si appoggia solo su:
    - behaviortree.tree.Tree
    - geometry_msgs/Twist
    - std_msgs/Float64MultiArray
    - servizi link-attacher + toggle gravità/collisioni (opzionali)
"""

import os
import sys
import time
import contextvars

import rclpy
from rclpy.node import Node

from behaviortree.tree import Tree
from xml.etree.ElementTree import parse

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from linkattacher_msgs.srv import AttachLink, DetachLink
from gazebo_link_gravity_toggle.srv import SetLinkGravity
from gazebo_collision_toggle.srv import SetCollisionEnabled

from ament_index_python.packages import get_package_share_directory

import os
import numpy as np
import math

#sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Task Prioritization è in /TaskPrioritization (sibling di /ros2_ws nel container)
# Individua la cartella anche da install space colcon e la aggiunge al sys.path
def _ensure_task_prioritization_on_path():
    here = os.path.abspath(os.path.dirname(__file__))
    candidates = [
        "/TaskPrioritization",  # posizione standard nel docker
        os.path.abspath(os.path.join(here, "..", "..", "..", "TaskPrioritization")),
        os.path.abspath(os.path.join(here, "..", "..", "..", "..", "..", "TaskPrioritization")),
    ]

    for cand in candidates:
        if os.path.isdir(cand):
            parent = os.path.dirname(cand) or cand
            if parent not in sys.path:
                sys.path.append(parent)
            break


_ensure_task_prioritization_on_path()
 
from TaskPrioritization.task_priority import TPManager
from TaskPrioritization.Trajectories.trajectory import Trajectory


# =============================================================================
# CONFIG: Durate e profili di movimento (ripresi dalla demo originale)
# =============================================================================

# Durate fasi (s) – puoi riallinearle ai valori della tua demo a stati
APPROACH_TIME = 31.8
DESCEND_AND_PICK_TIME = 20.5
COLLECT_TIME = 10.0
TRANSPORT_TIME = 25.0
DESCEND_AND_PLACE_TIME = 12.0
RELEASE_TIME = 1.0

# Velocità base in approach
APPROACH_LEFT_BASE_XY_VEL  = (0.0018, 0.25)
APPROACH_RIGHT_BASE_XY_VEL = (-0.022, 0.25)

# Velocità bracci durante l’approach
LEFT_ARM_VEL   =  [0.075, -0.01, 0.01, -0.01, -0.01, -0.01]
RIGHT_ARM_VEL  = [-0.09, -0.01, 0.01, -0.01, 0.01, 0.021]

# Pose di pick (discesa bracci)
LEFT_ARM_PICK   =  [-0.001, -0.08, 0.08, 0.0, -0.12, -0.055]
RIGHT_ARM_PICK  =  [0.001, -0.12, 0.08, 0.0, 0.12, 0.055]

# Collect / sollevamento
COLLECT_LEFT_BASE_XY_VEL  = (0.001, -0.15)
COLLECT_RIGHT_BASE_XY_VEL = (-0.0, -0.15)

LEFT_ARM_COLLECT   =  [0.0, 0.08, -0.08, 0.0, 0.0, -0.020]
RIGHT_ARM_COLLECT  =  [0.0, 0.08, -0.08, 0.0, 0.0, -0.025]

# Transport (transfer)
LEFT_TRANSPORT_VEL_XY  = (-0.24, 0.0)
RIGHT_TRANSPORT_VEL_XY = (-0.24 , -0.055)

# Discesa & place
# breve avvicinamento al pallet
PLACE_LEFT_BASE_XY_VEL  = (0.0, 0.09)
PLACE_RIGHT_BASE_XY_VEL = (0.0, 0.06)
# Pose per calata
LEFT_ARM_PLACE   =  [-0.001, -0.12, 0.08, 0.0, 0.0, 0.05]
RIGHT_ARM_PLACE  =  [0.001,  -0.12, 0.08, 0.0, -0.0, 0.0]

# Release
LEFT_ARM_RELEASE   =  [0.01, 0.0, 0.0, 0.0, 0.1, 0.0]
RIGHT_ARM_RELEASE  =  [-0.01, 0.0, 0.0, 0.0, -0.1, 0.0]

PACKAGE_LINK_NAME = "pacco_clone_1::link_1"




# =============================================================================
# Nodo ROS2: publisher + client servizi di base
# =============================================================================

class BTDemoNode(Node):
    """
    Nodo ROS2 minimale che espone:
    - publisher verso basi e bracci (come nella demo a stati)
    - client per attach/detach + collisioni/gravity (opzionali)
    - piccolo "blackboard" e gestione timer per le Action BT
    """

    def __init__(self):
        super().__init__("bt_pick_and_place_demo")

        # Topic default: allineati alla demo originale
        self.left_base_topic = "/left_summit_cmd_vel"
        self.right_base_topic = "/right_summit_cmd_vel"
        self.left_arm_topic = "/left/ur_left_joint_group_vel_controller/commands"
        self.right_arm_topic = "/right/ur_right_joint_group_vel_controller/commands"
          # /joint_states  
        # Publisher
        self.left_base_pub = self.create_publisher(Twist, self.left_base_topic, 10)
        self.right_base_pub = self.create_publisher(Twist, self.right_base_topic, 10)
        self.left_arm_pub = self.create_publisher(Float64MultiArray, self.left_arm_topic, 10)
        self.right_arm_pub = self.create_publisher(Float64MultiArray, self.right_arm_topic, 10)

        # Subscriber
        self._last_odom = {"left": None, "right": None}
        self._last_joint_states = {"left": None, "right": None}

        # subscribe (choose actual topic names in your system)
        self.create_subscription(Odometry, '/left/odom', lambda msg: self._odom_cb(msg, 'left'), 10)
        self.create_subscription(Odometry, '/right/odom', lambda msg: self._odom_cb(msg, 'right'), 10)
        self.create_subscription(JointState, '/left/joint_states', lambda msg: self._joint_states_cb(msg, 'left'), 10)
        self.create_subscription(JointState, '/right/joint_states', lambda msg: self._joint_states_cb(msg, 'right'), 10)

        # Services opzionali (se non ci sono semplicemente logga warning)
        self.attach_cli = self.create_client(AttachLink, "/ATTACHLINK")
        self.detach_cli = self.create_client(DetachLink, "/DETACHLINK")
        self.toggle_gravity_cli = self.create_client(SetLinkGravity, "/set_link_gravity")
        self.toggle_collision_cli = self.create_client(SetCollisionEnabled, "/set_collision_enabled")

        self.tp = TPManager(device='cuda', dtype='float32') # change device='cpu' to 'cuda' if GPU is available
        self.tp.add_robot(robot_name='robot', config_file='/ros2_ws/src/ps_try/config/dual_paletta.yaml')
        
        # initialize robot kinematics before using it
        self.robot = self.tp.get_robot_kinematics()
        initial_ee = self.robot.get_arm_ee_poses()
        self.tr_left  = Trajectory()
        self.tr_right = Trajectory()
        ## Impostato p_f come posizioni finali dei giunti per la presa del pacco
        self.tr_left.poly5(p_i=initial_ee[0], p_f=np.array([0.15708, -2.5831, -0.7326, -1.1107, 0.0524, -1.4469]), period=5.0)
        self.tr_right.poly5(p_i=initial_ee[1], p_f=np.array([-3.14, -2.7925, -0.4186, -1.1942, 3.1734, 1.6578]), period=5.0)

        self.ee_task = self.tp.add_EE_task('/ros2_ws/src/ps_try/config/ee_task.yaml')
        self.ee_task.use_base = False
        self.ee_task.set_trajectory([self.tr_left, None])
        

        # self.tp.initialize(initial_joint_pos=joint_pos)
        
        # posizione arm base-link rispetto al base-footprint
        # costruzione cinematica robot 
        # dentro lo yaml ^^
        # self.initialize(initial_joint_pos=initial_joint_positions, initial_base_pose=base_pose)

        # robot already initialized above
        # Piccolo "blackboard" per eventuale coordinazione (SRM1/SRM2/supervisor)
        self.bb = {}

        # Timer per azioni lunga durata: dict azione -> start_time
        self.action_timers = {}

        # Stato interno per azioni multi-fase (es. LiftObj)
        self.lift_phase = None

        self.get_logger().info("BTDemoNode inizializzato")

    def _joint_states_cb(self, msg: JointState, side: str):
        """Callback per gli stati dei giunti."""
        # keep a reference to the latest joint states msg
        self._last_joint_states[side] = msg

    def get_arm_joint_positions(self, side: str):
        """
        Return list of joint positions or None if no data yet.
        side: 'left' or 'right'
        """
        joint_states = self._last_joint_states.get(side)
        if joint_states is None:
            return None
        
        return list(joint_states.position)
    
    def _odom_cb(self, msg: Odometry, side: str):
        """Callback per l'odomentria delle basi."""
        # keep a reference to the latest odom msg
        self._last_odom[side] = msg

    @staticmethod
    def _quat_to_rpy(qx, qy, qz, qw):
        """Conversione da quaternioni a roll-pitch-yaw"""
        # standard quaternion -> roll,pitch,yaw
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def get_base_pose(self, side: str):
        """
        Return tuple (x, y, z, roll, pitch, yaw) or None if no data yet.
        side: 'left' or 'right'
        """
        odom = self._last_odom.get(side)
        if odom is None:
            return None

        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        pz = odom.pose.pose.position.z
        q = odom.pose.pose.orientation
        # Se TP vuole quaternioni scommenta questa riga e commenta le altre
        # return [px, py, pz, q]

        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        return [px, py, pz, roll, pitch, yaw]
    # -------------------------------------------------------------------------
    # Helpers per timer per azioni BT
    # -------------------------------------------------------------------------

    def get_action_timer(self, name: str):
        """Ritorna lo start_time dell'azione `name` (o None se non avviata)."""
        return self.action_timers.get(name)

    def start_action_timer(self, name: str):
        """Avvia (o resetta) il timer per l'azione `name`."""
        t = self.get_clock().now().nanoseconds/1e9
        self.action_timers[name] = t
        return t

    def clear_action_timer(self, name: str):
        """Cancella il timer associato a `name`."""
        if name in self.action_timers:
            del self.action_timers[name]

    # -------------------------------------------------------------------------
    # Helpers movimento
    # -------------------------------------------------------------------------

    def stop_all_movement(self):
        """Stop immediato di basi e bracci."""
        zero_twist = Twist()
        self.left_base_pub.publish(zero_twist)
        self.right_base_pub.publish(zero_twist)

        zero_arm = Float64MultiArray()
        zero_arm.data = [0.0] * 6
        self.left_arm_pub.publish(zero_arm)
        self.right_arm_pub.publish(zero_arm)


def set_package_gravity(node: BTDemoNode, gravity_on: bool) -> bool:
    client = node.toggle_gravity_cli
    if not client.service_is_ready():
        try:
            client.wait_for_service(timeout_sec=1.0)
        except Exception:
            pass
    if not client.service_is_ready():
        node.get_logger().warn(bt_fmt("[Gravity] toggle_gravity service unavailable"))
        return False

    req = SetLinkGravity.Request()
    req.model_name = ""
    req.link_name = PACKAGE_LINK_NAME
    req.gravity = gravity_on

    future = client.call_async(req)
    try:
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    except Exception:
        pass

    success = bool(future.done() and future.result() and getattr(future.result(), "success", True))
    if success:
        node.get_logger().info(bt_fmt(f"[Gravity] set gravity={gravity_on} on {PACKAGE_LINK_NAME}"))
    else:
        node.get_logger().warn(bt_fmt(f"[Gravity] failed to set gravity={gravity_on} on {PACKAGE_LINK_NAME}"))
    return success


# Nodo globale usato dalle funzioni delle foglie BT
bt_node: BTDemoNode | None = None

# Context per tracciare quale BT sta eseguendo una Action
CURRENT_BT_NAME = contextvars.ContextVar("CURRENT_BT_NAME", default="<unknown>" )


def get_current_bt_name() -> str:
    return CURRENT_BT_NAME.get("<unknown>")


def bt_fmt(message: str) -> str:
    return f"[{get_current_bt_name()}] {message}"


def assign_bt_name(tree: Tree, name: str):
    setattr(tree, "bt_name", name)
    for child in getattr(tree, "child_", []):
        assign_bt_name(child, name)


# =============================================================================
# PATCH DELLA CLASSE Tree: parsing XML "BTCPP" e run() semplificato
# =============================================================================

def gen_tree_from_btcpp(self, xmlfile, funcs):
    """
    Parser custom per XML stile BehaviorTree.CPP esportati da Groot.

    - Usa direttamente i TAG dei nodi come nome di funzione Python
      (FindObj, ApproachObject, Sync, MoveBase, ecc.)
    - I nodi di controllo (Sequence, Fallback, ParallelAll, RetryUntilSuccessful)
      vengono mappati sui type_ della Tree.
    """
    from behaviortree.tree import key_list  # usato solo per compatibilità

    # Dizionario: nome_funzione -> oggetto_funzione
    func_dict = {f.__name__: f for f in funcs}

    tree_xml = parse(xmlfile)
    root_elem = tree_xml.getroot()

    control_nodes = [
        "Sequence", "Fallback", "ParallelAll", "Parallel",
        "RetryUntilSuccessful", "BehaviorTree", "root",
        "TreeNodesModel", "SubTree",
    ]
    metadata_nodes = ["TreeNodesModel"]

    def _recurse(xml_elem, bt_root, parent_node, depth, child_index, is_first):
        current_node = parent_node

        # Salta descrizioni di modello/metadata
        if xml_elem.tag in metadata_nodes:
            return current_node

        # Nodo di controllo (Sequence, Fallback, ParallelAll, Retry...)
        if xml_elem.tag in control_nodes and xml_elem.tag not in ("root", "BehaviorTree"):
            if is_first:
                # Primo nodo di controllo: child diretto della Tree root
                current_node = bt_root._Tree__add_child(xml_elem.tag, depth, child_index)
                is_first = False
            else:
                # Successivi: child del parent_node
                current_node = parent_node._Tree__add_child(xml_elem.tag, depth, child_index)

        # Nodo foglia: TAG = nome funzione Python
        elif xml_elem.tag not in control_nodes:
            func_name = xml_elem.tag
            func_obj = func_dict.get(func_name, None)
            if parent_node is not None and func_obj is not None:
                parent_node._Tree__add_child("Action", depth, child_index, func_name, func_obj)
            elif parent_node is not None:
                print(f"[BT] WARNING: funzione '{func_name}' non trovata nel dizionario funzioni.")

        # Ricorsione sui figli XML
        if len(xml_elem) > 0:
            new_depth = depth + 1
            for i, child in enumerate(xml_elem):
                _recurse(child, bt_root, current_node, new_depth, i, is_first)

        return current_node

    # Partenza dalla root XML
    _recurse(root_elem, self, None, 1, 0, True)


def bt_run(self):
    """
    Implementazione semplificata di run() per Tree che supporta:

    - Foglie (Action/Condition): chiamano la funzione Python associata
      e memorizzano il proprio status per evitare loop infiniti.
        * True  -> SUCCESS
        * False -> RUNNING (nessun FAIL "duro" in questa demo)
        * None  -> RUNNING

    - Sequence:
        * scorre i figli in ordine
        * se un figlio è RUNNING -> RUNNING
        * se un figlio è FAIL    -> FAIL (non usato qui)
        * tutti SUCCESS          -> SUCCESS

    - Fallback:
        * ritorna SUCCESS al primo figlio SUCCESS
        * se un figlio è RUNNING -> RUNNING
        * tutti FAIL             -> FAIL

    - ParallelAll:
        * tutti i figli devono andare in SUCCESS
        * se almeno uno RUNNING  -> RUNNING
        * se uno FAIL            -> FAIL

    - RetryUntilSuccessful:
        * ritenta il primo figlio finché non va in SUCCESS
        * quando il figlio è SUCCESS -> SUCCESS
        * altrimenti RUNNING
    """
    node_type = getattr(self, "type_", None)

    # ---------------------- FOGLIA ----------------------
    if getattr(self, "id_", None) is not None and getattr(self, "func_", None) is not None:
        # Se la foglia è già in SUCCESS non rieseguire la funzione
        if getattr(self, "_status", "IDLE") == "SUCCESS":
            return True

        tree_name = getattr(self, "bt_name", "<unnamed>")
        token = CURRENT_BT_NAME.set(tree_name)
        try:
            result = self.func_()  # True / False / None
        finally:
            CURRENT_BT_NAME.reset(token)

        if result is True:
            self._status = "SUCCESS"
            return True
        if result is None:
            self._status = "RUNNING"
            return None
        if result is False:
            self._status = "FAIL"
            return False

        # qualsiasi altra cosa -> RUNNING
        self._status = "RUNNING"
        return None

    # ---------------------- NODI DI CONTROLLO ----------------------
    # Inizializza status se non presente
    if not hasattr(self, "_status"):
        self._status = "IDLE"

    # SEQUENCE ------------------------------------------------------
    if node_type == "Sequence":
        for child in self.child_:
            status = child.run()
            if status is None:   # RUNNING
                self._status = "RUNNING"
                return None
            if status is False:  # FAIL
                self._status = "FAIL"
                return False
        # Tutti SUCCESS
        self._status = "SUCCESS"
        return True

    # FALLBACK ------------------------------------------------------
    if node_type == "Fallback":
        for child in self.child_:
            status = child.run()
            if status is True:   # primo SUCCESS
                self._status = "SUCCESS"
                return True
            if status is None:
                self._status = "RUNNING"
                return None
        # tutti FAIL
        self._status = "FAIL"
        return False

    # PARALLELALL ---------------------------------------------------
    if node_type in ("ParallelAll", "Parallel"):
        running_seen = False
        for child in self.child_:
            status = child.run()
            if status is False:   # un fallimento blocca tutto
                self._status = "FAIL"
                return False
            if status is None:
                running_seen = True
        if running_seen:
            self._status = "RUNNING"
            return None
        self._status = "SUCCESS"
        return True

    # RETRYUNTILSUCCESSFUL ------------------------------------------
    if node_type == "RetryUntilSuccessful":
        if not self.child_:
            self._status = "SUCCESS"
            return True
        child = self.child_[0]
        status = child.run()
        if status is True:
            self._status = "SUCCESS"
            return True
        # fallimento o running -> continuo a ritentare
        self._status = "RUNNING"
        return None

    # Default: ticka i figli in ordine e ritorna il primo status non None
    for child in self.child_:
        status = child.run()
        if status is not None:
            return status
    return None


# Applica le patch alla classe Tree
Tree.gen_tree_from_btcpp = gen_tree_from_btcpp
Tree.run = bt_run


# =============================================================================
# FUNZIONI FOGLIA DEI BT (Action / Condition)
#   - usano il nodo globale bt_node
#   - ritornano:
#       True  -> SUCCESS
#       False -> RUNNING
#       None  -> RUNNING (per semplicità)
# =============================================================================

def _require_node() -> BTDemoNode:
    if bt_node is None:
        raise RuntimeError("bt_node non inizializzato")
    return bt_node


# -------------------------------------------------------------------------
# MOCK / CONDITION / AZIONI SEMPLICI
# -------------------------------------------------------------------------

def Sync():
    """
    Nodo di sincronizzazione (SRM1/SRM2/supervisor).
    Qui simulato come attesa di 1 s, poi SUCCESS.
    """
    node = _require_node()
    t0 = node.get_action_timer("Sync")
    if t0 is None:
        node.get_logger().info(bt_fmt("[Sync] start"))
        t0 = node.start_action_timer("Sync")

    if node.get_clock().now().nanoseconds/1e9 - t0 < 1.0:
        return None  # RUNNING
    node.clear_action_timer("Sync")
    node.get_logger().info(bt_fmt("[Sync] done"))
    return True


def FindObj():
    """
    Identificazione pacco target.
    Mock: attende 1 s e restituisce SUCCESS.
    """
    node = _require_node()
    t0 = node.get_action_timer("FindObj")
    if t0 is None:
        node.get_logger().info(bt_fmt("[FindObj] start (mock)"))
        t0 = node.start_action_timer("FindObj")

    if node.get_clock().now().nanoseconds/1e9 - t0 < 1.0:
        return None
    node.clear_action_timer("FindObj")
    node.bb["target_found"] = True
    node.get_logger().info(bt_fmt("[FindObj] target_found = True"))
    return True


def CalculateGoal():
    """
    Analisi posizione pacco / calcolo goal.
    Mock: SUCCESS immediato con side-effect sulla blackboard.
    """
    node = _require_node()
    node.bb["goal_computed"] = True
    node.get_logger().info(bt_fmt("[CalculateGoal] goal_computed = True (mock)"))
    return True


def CheckAlignment():
    """
    Verifica allineamento (basi / EE).
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    # Potresti leggere qualcosa dal mondo qui.
    node.get_logger().info(bt_fmt("[CheckAlignment] (mock)"))
    return True


def Set():
    """
    Settaggio stato (es. cambio modalita controller).
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[Set] (mock)"))
    return True


def CloseGripper():
    """
    Chiusura gripper / prese.
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[CloseGripper] (mock)"))
    return True


def SaySomething():
    """
    Nodo "log vocale".
    Qui solo log su console.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[SaySomething] (mock)"))
    return True


def NearObj():
    """
    Condizione "vicino all'oggetto".
    In questa demo viene legata al completamento di ApproachObject.
    """
    node = _require_node()
    tree_name = get_current_bt_name()
    key = f"{tree_name}_near_object"
    value = bool(node.bb.get(key, False))
    node.get_logger().info(bt_fmt(f"[NearObj] ({key}) = {value}"))
    return value


def DataReceived():
    """
    Condizione "dati ricevuti" (es. SRM2 che riceve info da SRM1).
    Mock: attesa 1 s dopo un eventuale messaggio, poi SUCCESS.
    """
    node = _require_node()
    t0 = node.get_action_timer("DataReceived")
    if t0 is None:
        node.get_logger().info(bt_fmt("[DataReceived] waiting (mock)"))
        t0 = node.start_action_timer("DataReceived")

    if node.get_clock().now().nanoseconds/1e9 - t0 < 1.0:
        return None
    node.clear_action_timer("DataReceived")
    node.get_logger().info(bt_fmt("[DataReceived] done (mock)"))
    return True


def Controller():
    """
    Nodo generico di controllo (forza, posizione, angoli, ecc.).
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[Controller] (mock)"))
    return True


def CorrectBasePos():
    """
    Correzione posizione basi per allineamento.
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[CorrectBasePos] (mock)"))
    return True


def AdjustPositioning():
    """
    Aggiustamento posizionamento per drop.
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[AdjustPositioning] (mock)"))
    return True


def ReturnToPallet():
    """
    Ritorno al pallet dopo il rilascio.
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[ReturnToPallet] (mock)"))
    return True


# -------------------------------------------------------------------------
# AZIONI DI MOVIMENTO "VERO" (pick & place)
# -------------------------------------------------------------------------

def ApproachObject():
    """
    Avvicinamento al pacco:
    - muove le basi in approach
    - muove i bracci con piccola velocità
    - dopo APPROACH_TIME secondi ritorna SUCCESS e stoppa i movimenti

    Mappa la fase 'approach' della demo a stati.
    """
    node = _require_node()
    tree_name = get_current_bt_name()
    timer_key = f"{tree_name}_ApproachObject"
    near_key = f"{tree_name}_near_object"

    # t0 = node.get_action_timer(timer_key)
    # if t0 is None:
    #     node.get_logger().info(bt_fmt(f"[ApproachObject] start (dur: {APPROACH_TIME}s)"))
    #     t0 = node.start_action_timer(timer_key)


    # elapsed = node.get_clock().now().nanoseconds/1e9 - t0

    # if elapsed < APPROACH_TIME:
    #     tl = Twist()
    #     tr = Twist()
    #     tl.linear.x, tl.linear.y = APPROACH_LEFT_BASE_XY_VEL
    #     tr.linear.x, tr.linear.y = APPROACH_RIGHT_BASE_XY_VEL
    #     node.left_base_pub.publish(tl)
    #     node.right_base_pub.publish(tr)

    #     la = Float64MultiArray()
    #     ra = Float64MultiArray()
    #     la.data = LEFT_ARM_VEL
    #     ra.data = RIGHT_ARM_VEL

    #     node.left_arm_pub.publish(la)
    #     node.right_arm_pub.publish(ra)

    #     # piccolo spin per far "vivere" ROS2
    #     rclpy.spin_once(node, timeout_sec=0.01)
    #     return None  # RUNNING
    # else:
    #     node.stop_all_movement()
    #     node.clear_action_timer(timer_key)
    #     node.bb[near_key] = True
    #     node.get_logger().info(bt_fmt(f"[ApproachObject] completed, {near_key}=True"))
    #     return True

    # use the TPManager instance attached to the node
    node.tp.cycle_starts()

    left_arm_jp = node.get_arm_joint_positions("left") # pose dai giunti da /left/joint_states
    right_arm_jp = node.get_arm_joint_positions("right") # pose dai giunti da /right/joint_states

    left_base = node.get_base_pose("left")   #pose absolute world fixed frame da left_summit_odom
    right_base = node.get_base_pose("right") #pose absolute world fixed frame da right_summit_odom
    
    # Check if all sensor data is available before proceeding
    if None in [left_arm_jp, right_arm_jp, left_base, right_base]:
        node.get_logger().warn(bt_fmt("[ApproachObject] Waiting for sensor data (joint_states/odom)"))
        node.get_logger().warn(bt_fmt(f"  left_arm_jp: {left_arm_jp}"))
        node.get_logger().warn(bt_fmt(f"  right_arm_jp: {right_arm_jp}"))
        node.get_logger().warn(bt_fmt(f"  left_base: {left_base}"))
        node.get_logger().warn(bt_fmt(f"  right_base: {right_base}"))
         # piccolo spin per far "vivere" ROS2
        rclpy.spin_once(node, timeout_sec=0.01)
        return None  # RUNNING - keep trying until data arrives
    
    #[np.array(xyzrpybase_left), np.array(joint_left_arm), np.array(xyzrpybase_right), np.array(joint_right_arm)] 
    joint_pos = [left_base, left_arm_jp, right_base, right_arm_jp]

    cmd = node.tp.execute(joint_pos=joint_pos)

    # Build ROS messages from numeric command vector
    if cmd is not None:
        # Cast to native floats because ROS2 message fields reject numpy/torch scalar types.
        left_base_cmd_vals = [float(v) for v in cmd[:3]]  # vx, vy, omega
        left_arm_cmd_vals = [float(v) for v in cmd[3:9]]
        right_base_cmd_vals = [float(v) for v in cmd[9:12]]
        right_arm_cmd_vals = [float(v) for v in cmd[12:18]]
    else:
        left_base_cmd_vals = [0.0, 0.0, 0.0]
        left_arm_cmd_vals = [0.0]*6
        right_base_cmd_vals = [0.0, 0.0, 0.0]
        right_arm_cmd_vals = [0.0]*6

    # Twist messages for base commands
    tl = Twist()
    tr = Twist()
    try:
        tl.linear.x, tl.linear.y, tl.angular.z = left_base_cmd_vals
    except Exception:
        # fallback: assign what we can
        if len(left_base_cmd_vals) >= 1:
            tl.linear.x = left_base_cmd_vals[0]
        if len(left_base_cmd_vals) >= 2:
            tl.linear.y = left_base_cmd_vals[1]
        if len(left_base_cmd_vals) >= 3:
            tl.angular.z = left_base_cmd_vals[2]

    try:
        tr.linear.x, tr.linear.y, tr.angular.z = right_base_cmd_vals
    except Exception:
        if len(right_base_cmd_vals) >= 1:
            tr.linear.x = right_base_cmd_vals[0]
        if len(right_base_cmd_vals) >= 2:
            tr.linear.y = right_base_cmd_vals[1]
        if len(right_base_cmd_vals) >= 3:
            tr.angular.z = right_base_cmd_vals[2]

    # Float64MultiArray for arm joint velocity commands
    la = Float64MultiArray()
    ra = Float64MultiArray()
    la.data = list(left_arm_cmd_vals)
    ra.data = list(right_arm_cmd_vals)

    node.left_base_pub.publish(tl)
    node.right_base_pub.publish(tr)
    node.left_arm_pub.publish(la)
    node.right_arm_pub.publish(ra)

    node.tp.cycle_ends()


def LiftObj():
    """
    Fase composita: discesa + pick + collect (sollevamento).
    - 1) DESCEND_AND_PICK_TIME  : bracci verso pose di pick
    - 2) 0.5 s                   : attach del pacco
    - 3) COLLECT_TIME           : basi + bracci in collect (sollevamento)

    Mappa le fasi:
      * discesa
      * pick (attach)
      * collect
    """
    node = _require_node()
    tree_name = get_current_bt_name()

    if tree_name == "Supervisor":
        srm1_ready = node.bb.get("SRM1_near_object", False)
        srm2_ready = node.bb.get("SRM2_near_object", False)
        if not (srm1_ready and srm2_ready):
            wait_key = "supervisor_lift_wait_logged"
            if not node.bb.get(wait_key):
                node.bb[wait_key] = True
                node.get_logger().info(bt_fmt("[LiftObj] waiting for SRM1/SRM2 approach completion"))
            return None
        else:
            node.bb.pop("supervisor_lift_wait_logged", None)

    t0 = node.get_action_timer("LiftObj")
    if t0 is None or node.lift_phase is None:
        node.get_logger().info(bt_fmt(f"[LiftObj] start (dur {DESCEND_AND_PICK_TIME + 0.5 + COLLECT_TIME}s)"))
        t0 = node.start_action_timer("LiftObj")
        node.lift_phase = "descend_pick"

    elapsed = node.get_clock().now().nanoseconds/1e9 - t0

    # 1) Discesa e pick
    if node.lift_phase == "descend_pick":
        if elapsed < DESCEND_AND_PICK_TIME:
            la = Float64MultiArray()
            ra = Float64MultiArray()
            la.data = LEFT_ARM_PICK
            ra.data = RIGHT_ARM_PICK
            node.left_arm_pub.publish(la)
            node.right_arm_pub.publish(ra)
            node.get_logger().info(bt_fmt("[LiftObj] descending..."))
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        else:
            # passa alla fase attach
            node.stop_all_movement()
            node.lift_phase = "attach"
            node.get_logger().info(bt_fmt("[LiftObj] reached pick pose, attaching..."))
            node.start_action_timer("LiftObj")  # restart timer per attach
            return None

    # 2) Attach (tempo breve)
    if node.lift_phase == "attach":
        if elapsed < 0.5:
            if node.attach_cli.service_is_ready():
                req = AttachLink.Request()
                setattr(req, "model1_name", "left_robot")
                setattr(req, "link1_name", "ur_left_wrist_3_link")
                setattr(req, "model2_name", "pacco_clone_1")
                setattr(req, "link2_name", "pacco_clone_1::link_1")
                node.attach_cli.call_async(req)
                node.get_logger().info(bt_fmt("[LiftObj] attach request sent"))
                if not node.bb.get("package_gravity_disabled", False):
                    if set_package_gravity(node, False):
                        node.bb["package_gravity_disabled"] = True
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        else:
            node.stop_all_movement()
            node.lift_phase = "collect"
            node.get_logger().info(bt_fmt("[LiftObj] attached"))
            node.start_action_timer("LiftObj")  # restart timer per collect
            return None

    # 3) Collect / sollevamento
    if node.lift_phase == "collect":
        if elapsed < COLLECT_TIME:
            tl = Twist()
            tr = Twist()
            tl.linear.x, tl.linear.y = COLLECT_LEFT_BASE_XY_VEL
            tr.linear.x, tr.linear.y = COLLECT_RIGHT_BASE_XY_VEL
            node.left_base_pub.publish(tl)
            node.right_base_pub.publish(tr)

            la = Float64MultiArray()
            ra = Float64MultiArray()
            la.data = LEFT_ARM_COLLECT
            ra.data = RIGHT_ARM_COLLECT
            node.left_arm_pub.publish(la)
            node.right_arm_pub.publish(ra)

            node.get_logger().info(bt_fmt("[LiftObj] collecting (lifting)..."))
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        else:
            node.stop_all_movement()
            node.lift_phase = None
            node.clear_action_timer("LiftObj")
            node.get_logger().info(bt_fmt("[LiftObj] completed"))
            return True

    # fallback
    node.lift_phase = None
    node.clear_action_timer("LiftObj")
    return True


def MoveBase():
    """
    Movimento delle basi (transfer):
    - usato dal supervisore per DST e ReturnToPallet
    - qui implementato come semplice "transport" per TRANSPORT_TIME.
    """
    node = _require_node()
    tree_name = get_current_bt_name()

    t0 = node.get_action_timer("MoveBase")
    if t0 is None:
        node.get_logger().info(bt_fmt(f"[MoveBase] start ({TRANSPORT_TIME}s)"))
        t0 = node.start_action_timer("MoveBase")

    if node.get_clock().now().nanoseconds/1e9 - t0 < TRANSPORT_TIME:
        tl = Twist()
        tr = Twist()
        tl.linear.x, tl.linear.y = LEFT_TRANSPORT_VEL_XY
        tr.linear.x, tr.linear.y = RIGHT_TRANSPORT_VEL_XY
        node.left_base_pub.publish(tl)
        node.right_base_pub.publish(tr)
        rclpy.spin_once(node, timeout_sec=0.01)
        return None
    else:
        node.stop_all_movement()
        node.clear_action_timer("MoveBase")
        node.get_logger().info(bt_fmt("[MoveBase] completed"))
        return True


def Drop():
    """
    Calata e posizionamento del pacco in zona place:
    - basi + bracci verso pose di place per DESCEND_AND_PLACE_TIME.

    Mappa la fase 'discesa + place'.
    """
    node = _require_node()
    t0 = node.get_action_timer("Drop")
    if t0 is None:
        node.get_logger().info(bt_fmt(f"[Drop] start ({DESCEND_AND_PLACE_TIME}s)"))
        t0 = node.start_action_timer("Drop")

    if node.get_clock().now().nanoseconds/1e9 - t0 < DESCEND_AND_PLACE_TIME:
        tl = Twist()
        tr = Twist()
        tl.linear.x, tl.linear.y = PLACE_LEFT_BASE_XY_VEL
        tr.linear.x, tr.linear.y = PLACE_RIGHT_BASE_XY_VEL
        node.left_base_pub.publish(tl)
        node.right_base_pub.publish(tr)

        la = Float64MultiArray()
        ra = Float64MultiArray()
        la.data = LEFT_ARM_PLACE
        ra.data = RIGHT_ARM_PLACE
        node.left_arm_pub.publish(la)
        node.right_arm_pub.publish(ra)

        rclpy.spin_once(node, timeout_sec=0.01)
        return None
    else:
        node.stop_all_movement()
        node.clear_action_timer("Drop")
        node.get_logger().info(bt_fmt("[Drop] completed"))
        return True


def Release():
    """
    Release dell'oggetto:
    - movimento bracci verso pose di release per RELEASE_TIME
    - detach del pacco (link-attacher)

    Mappa la fase 'release' della demo.
    """
    node = _require_node()
    t0 = node.get_action_timer("Release")
    if t0 is None:
        node.get_logger().info(bt_fmt(f"[Release] start ({RELEASE_TIME}s)"))
        t0 = node.start_action_timer("Release")

    if node.get_clock().now().nanoseconds/1e9 - t0 < RELEASE_TIME:
        la = Float64MultiArray()
        ra = Float64MultiArray()
        la.data = LEFT_ARM_RELEASE
        ra.data = RIGHT_ARM_RELEASE
        node.left_arm_pub.publish(la)
        node.right_arm_pub.publish(ra)
        rclpy.spin_once(node, timeout_sec=0.01)
        return None
    else:
        node.stop_all_movement()

        # Detach del pacco (best-effort)
        if node.detach_cli.service_is_ready():
            req = DetachLink.Request()
            setattr(req, "model1_name", "left_robot")
            setattr(req, "link1_name", "ur_left_wrist_3_link")
            setattr(req, "model2_name", "pacco_clone_1")
            setattr(req, "link2_name", "pacco_clone_1::link_1")
            node.detach_cli.call_async(req)
            node.get_logger().info(bt_fmt("[Release] detach request sent"))

        if node.bb.pop("package_gravity_disabled", False):
            set_package_gravity(node, True)

        # Inizio fase di allontanamento (retreat)
        node.clear_action_timer("Release")
        retreat_t0 = node.start_action_timer("ReleaseRetreat")
        node.get_logger().info(bt_fmt("[Release] retreat phase started"))

        while node.get_clock().now().nanoseconds/1e9 - retreat_t0 < COLLECT_TIME:
            tl = Twist()
            tr = Twist()
            tl.linear.x, tl.linear.y = COLLECT_LEFT_BASE_XY_VEL
            tr.linear.x, tr.linear.y = COLLECT_RIGHT_BASE_XY_VEL
            node.left_base_pub.publish(tl)
            node.right_base_pub.publish(tr)

            la = Float64MultiArray()
            ra = Float64MultiArray()
            la.data = LEFT_ARM_COLLECT
            ra.data = RIGHT_ARM_COLLECT
            node.left_arm_pub.publish(la)
            node.right_arm_pub.publish(ra)

            node.get_logger().info(bt_fmt("[Release] retreating (moving away)..."))
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.1)

        node.stop_all_movement()
        node.clear_action_timer("ReleaseRetreat")
        node.get_logger().info(bt_fmt("[Release] completed"))
        return True


# =============================================================================
# MAIN: setup, parsing XML, creazione BT e loop di esecuzione
# =============================================================================

def main():
    global bt_node

    rclpy.init()

    # Nodo ROS
    bt_node = BTDemoNode()
    bt_node.bb.setdefault("SRM1_done", False)
    bt_node.bb.setdefault("SRM2_done", False)

    # -------------------------------------------------------------------------
    # Individua i file XML
    # -------------------------------------------------------------------------
    try:
        pkg_share = get_package_share_directory("ps_try")
        xml_base_path = os.path.join(pkg_share, "scripts", "behaviortree_XML_demo")
    except Exception:
        # Path di sviluppo: stessa cartella del file + subdir
        script_dir = os.path.dirname(os.path.abspath(__file__))
        xml_base_path = os.path.join(script_dir, "behaviortree_XML_demo")

    xmlfile_supervisor = os.path.join(xml_base_path, "SRM_Supervisor.xml")
    xmlfile_srm1 = os.path.join(xml_base_path, "SRM1.xml")
    xmlfile_srm2 = os.path.join(xml_base_path, "SRM2.xml")

    for f in (xmlfile_supervisor, xmlfile_srm1, xmlfile_srm2):
        if not os.path.exists(f):
            bt_node.get_logger().error(f"XML non trovato: {f}")
            rclpy.shutdown()
            return

    bt_node.get_logger().info("XML BT trovati, costruzione alberi...")

    # -------------------------------------------------------------------------
    # Crea le Tree e genera dagli XML
    # -------------------------------------------------------------------------
    bt_tree_supervisor = Tree()
    bt_tree_srm1 = Tree()
    bt_tree_srm2 = Tree()

    # Lista completa delle funzioni disponibili (foglie)
    all_funcs = [
        # mock / condition
        Sync, FindObj, CalculateGoal, CheckAlignment, Set,
        CloseGripper, SaySomething, NearObj, DataReceived, Controller,
        CorrectBasePos, AdjustPositioning, ReturnToPallet,
        # azioni di movimento
        ApproachObject, LiftObj, MoveBase, Drop, Release
    ]

    bt_tree_supervisor.gen_tree_from_btcpp(xmlfile_supervisor, all_funcs)
    bt_tree_srm1.gen_tree_from_btcpp(xmlfile_srm1, all_funcs)
    bt_tree_srm2.gen_tree_from_btcpp(xmlfile_srm2, all_funcs)

    assign_bt_name(bt_tree_supervisor, "Supervisor")
    assign_bt_name(bt_tree_srm1, "SRM1")
    assign_bt_name(bt_tree_srm2, "SRM2")

    bt_node.get_logger().info("Behavior Tree caricati dagli XML.")

    # -------------------------------------------------------------------------
    # Loop di esecuzione: tick dei tre alberi in parallelo concettuale
    # -------------------------------------------------------------------------
    rate_hz = 10.0
    tick_dt = 1.0 / rate_hz

    bt_node.get_logger().info("Inizio esecuzione BT (Ctrl+C per uscire).")

    try:
        while rclpy.ok():
            result_srm1 = bt_tree_srm1.run()
            result_srm2 = bt_tree_srm2.run()
            result_supervisor = bt_tree_supervisor.run()
            # result_srm2 = True
            # result_supervisor = True

            if result_srm1 is True and not bt_node.bb.get("SRM1_done", False):
                bt_node.bb["SRM1_done"] = True
                bt_node.get_logger().info("[SRM1] ha completato l'albero (flag bb)")

            if result_srm2 is True and not bt_node.bb.get("SRM2_done", False):
                bt_node.bb["SRM2_done"] = True
                bt_node.get_logger().info("[SRM2] ha completato l'albero (flag bb)")

            # Avanza esecuzione ROS (callback servizi ecc.)
            rclpy.spin_once(bt_node, timeout_sec=0.01)

            # Stop opzionale quando tutti completati
            # (se vuoi, puoi usare una variabile d'ambiente o parametro)
            if result_supervisor is True and result_srm1 is True and result_srm2 is True:
                bt_node.get_logger().info("Tutti i Behavior Tree hanno riportato SUCCESS.")
                break

            time.sleep(tick_dt)

    except KeyboardInterrupt:
        bt_node.get_logger().info("Shutdown richiesto da tastiera.")
    finally:
        if bt_node is not None:
            try:
                bt_node.stop_all_movement()
            except Exception:
                pass
            bt_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
