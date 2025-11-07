#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, time
from typing import Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import py_trees
from py_trees.common import Status

# Messaggi / Servizi come nella demo pick and place
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import SetModelState, GetEntityState, SetEntityState
from linkattacher_msgs.srv import AttachLink, DetachLink
from gazebo_link_gravity_toggle.srv import SetLinkGravity
from gazebo_collision_toggle.srv import SetCollisionEnabled

import tf2_ros

# ========= CONFIG: durate e profili (ripresi dalla demo) =========

# Durate (s)
APPROACH_TIME = 10.7
DESCEND_AND_PICK_TIME = 12.0
COLLECT_TIME = 10.0
TRANSPORT_TIME = 16.0
DESCEND_AND_PLACE_TIME = 11.0
RELEASE_TIME = 1.0

# =========== APPROACH ==========
# Velocità base in approach

# APPROACH_LEFT_BASE_XY_VEL  = (0.0018, 0.25)
# APPROACH_RIGHT_BASE_XY_VEL = (-0.022, 0.25)

APPROACH_LEFT_BASE_XY_VEL  = (0.0018, 0.22)
APPROACH_RIGHT_BASE_XY_VEL = (-0.022, 0.22)

# APPROACH_LEFT_BASE_XY_VEL  = (0.002, 0.25)
# APPROACH_RIGHT_BASE_XY_VEL = (-0.022, 0.25)

# Velocità bracci (esempio; usa le tue)
LEFT_ARM_VEL   =  [0.075, -0.01, 0.01, -0.01, -0.01, -0.01]
RIGHT_ARM_VEL  = [-0.09, -0.01, 0.01, -0.01, 0.01, 0.021]
# LEFT_ARM_VEL   =  [0.072, -0.01, 0.01, -0.01, -0.01, -0.01]
# RIGHT_ARM_VEL  = [-0.09, -0.01, 0.01, -0.01, 0.01, 0.021]

# =========== PICK ==========
# Pose per “calata” e presa (comandi velocità giunti o target semplici)
LEFT_ARM_PICK   =  [-0.001, -0.08, 0.08, 0.0, -0.12, -0.055]
RIGHT_ARM_PICK  =  [0.001, -0.12, 0.08, 0.0, 0.12, 0.055]

# =========== COLLECT ==========
# Velocità base nel collect
COLLECT_LEFT_BASE_XY_VEL  = (0.001, -0.15)
COLLECT_RIGHT_BASE_XY_VEL = (-0.0, -0.15)

LEFT_ARM_COLLECT   =  [0.0, 0.08, -0.08, 0.0, 0.0, -0.020]
RIGHT_ARM_COLLECT  =  [0.0, 0.08, -0.08, 0.0, 0.0, -0.025]

# =========== TRANSPORT ==========
# Velocità base nel trasporto
LEFT_TRANSPORT_VEL_XY  = (-0.24, 0.0)
RIGHT_TRANSPORT_VEL_XY = (-0.24 , -0.055)

# ========== DESCEND & PLACE ==========
# breve avvicinamento al pallet
PLACE_LEFT_BASE_XY_VEL  = (0.0, 0.09)
PLACE_RIGHT_BASE_XY_VEL = (0.0, 0.06)
# Pose per calata
LEFT_ARM_PLACE   =  [-0.001, -0.12, 0.08, 0.0, 0.0, 0.05]
RIGHT_ARM_PLACE  =  [0.001,  -0.12, 0.08, 0.0, -0.0, 0.0]

# ========== RELEASE ==========
# Pose per rilascio
LEFT_ARM_RELEASE   =  [0.01, 0.0, 0.0, 0.0, 0.1, 0.0]
RIGHT_ARM_RELEASE  =  [-0.01, 0.0, 0.0, 0.0, -0.1, 0.0]


# logging level
LOG_LEVEL = rclpy.logging.LoggingSeverity.INFO

# ========= Nodo ROS 2 che fornisce publisher/clients a tutte le foglie =========

class DemoNode(Node):
    def __init__(self, args):
        super().__init__("bt_pick_place_demo")

        # Argomenti (stessi della demo)
        self.object_name = args.object
        self.object_link = args.object_link
        self.attach_robot_model = args.attach_robot_model
        self.attach_robot_link  = args.attach_robot_link

        # Topic movimento
        self.left_base_topic  = args.left_base_topic
        self.right_base_topic = args.right_base_topic
        self.left_arm_topic   = args.left_arm_topic
        self.right_arm_topic  = args.right_arm_topic

        # Publisher
        self.left_base_pub  = self.create_publisher(Twist, self.left_base_topic, 10)
        self.right_base_pub = self.create_publisher(Twist, self.right_base_topic, 10)
        self.left_arm_pub   = self.create_publisher(Float64MultiArray, self.left_arm_topic, 10)
        self.right_arm_pub  = self.create_publisher(Float64MultiArray, self.right_arm_topic, 10)

        # TF buffer/listener (per futuri controlli; qui non usati)
        self.tf_buffer   = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Services
        self.cli_set_model_state  = self.create_client(SetModelState,   "/gazebo/set_model_state")
        self.cli_get_entity_state = self.create_client(GetEntityState,  "/state/get_entity_state")
        self.cli_set_entity_state = self.create_client(SetEntityState,  "/state/set_entity_state")
        self.toggle_gravity_cli   = self.create_client(SetLinkGravity,  "/set_link_gravity")
        self.toggle_collision_cli = self.create_client(SetCollisionEnabled, "/set_collision_enabled")
        self.attach_cli           = self.create_client(AttachLink, "/ATTACHLINK")
        self.detach_cli           = self.create_client(DetachLink, "/DETACHLINK")

        # Attesa "soft" dei servizi principali (come nella demo)
        deadline = time.time() + 10.0
        while time.time() < deadline:
            ok_model = (self.cli_set_model_state.service_is_ready() or
                        self.cli_set_entity_state.service_is_ready())
            ok_attach = self.attach_cli.service_is_ready() and self.detach_cli.service_is_ready()
            if ok_model and ok_attach:
                break
            for cli in [self.cli_set_model_state, self.cli_set_entity_state,
                        self.attach_cli, self.detach_cli]:
                try:
                    if not cli.service_is_ready():
                        cli.wait_for_service(timeout_sec=0.3)
                except Exception:
                    pass

    # ——— Le stesse utility della demo, riusate nelle foglie ———
    def stop_all_movement(self):
        zero = Twist()
        self.left_base_pub.publish(zero)
        self.right_base_pub.publish(zero)
        z = Float64MultiArray(); z.data = [0.0]*6
        self.left_arm_pub.publish(z)
        self.right_arm_pub.publish(z)
        self.get_logger().info("[stop_all_movement] Published zero to all base and arm topics.")
        time.sleep(0.1)

    def set_paletta_collision(self, enable: bool, side="left") -> bool:
        if not self.toggle_collision_cli.service_is_ready():
            try: self.toggle_collision_cli.wait_for_service(timeout_sec=1.0)
            except Exception: pass
        if not self.toggle_collision_cli.service_is_ready():
            self.get_logger().warn("set_collision_enabled not ready"); return False
        req = SetCollisionEnabled.Request()
        if side == "left":
            req.model_name = "left_robot";  req.link_name  = "ur_left_wrist_3_link"
        else:
            req.model_name = "right_robot"; req.link_name  = "ur_right_wrist_3_link"
        req.collision_name = ""
        req.enable = enable
        fut = self.toggle_collision_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        return bool(fut.done() and fut.result() and getattr(fut.result(), "success", True))

    def set_pacco_gravity(self, gravity_on: bool) -> bool:
        if not self.toggle_gravity_cli.service_is_ready():
            try: self.toggle_gravity_cli.wait_for_service(timeout_sec=1.0)
            except Exception: pass
        if not self.toggle_gravity_cli.service_is_ready():
            self.get_logger().warn("toggle_gravity not ready"); return False
        req = SetLinkGravity.Request()
        req.model_name = ""                 # uso forma scoped
        req.link_name  = self.object_link
        req.gravity    = gravity_on
        fut = self.toggle_gravity_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        return bool(fut.done() and fut.result() and getattr(fut.result(), "success", True))

    def attach_fixed_joint(self) -> bool:
        if not self.attach_cli.service_is_ready():
            try: self.attach_cli.wait_for_service(timeout_sec=1.0)
            except Exception: pass
        if not self.attach_cli.service_is_ready():
            self.get_logger().warn("ATTACHLINK non disponibile"); return False
        req = AttachLink.Request()
        setattr(req, "model1_name", self.attach_robot_model)
        setattr(req, "link1_name",  self.attach_robot_link)
        setattr(req, "model2_name", self.object_name)
        setattr(req, "link2_name",  self.object_link)
        fut = self.attach_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None:
            return False
        res = fut.result()
        ok = getattr(res, "success", None)
        if ok is None: ok = getattr(res, "ok", False)
        return bool(ok)

    def detach_fixed_joint(self) -> bool:
        if not self.detach_cli.service_is_ready():
            try: self.detach_cli.wait_for_service(timeout_sec=1.0)
            except Exception: pass
        if not self.detach_cli.service_is_ready():
            self.get_logger().warn("DETACHLINK non disponibile"); return False
        req = DetachLink.Request()
        setattr(req, "model1_name", self.attach_robot_model)
        setattr(req, "link1_name",  self.attach_robot_link)
        setattr(req, "model2_name", self.object_name)
        setattr(req, "link2_name",  self.object_link)
        fut = self.detach_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None:
            return False
        res = fut.result()
        ok = getattr(res, "success", None)
        if ok is None: ok = getattr(res, "ok", False)
        return bool(ok)


# ========= Behaviour generiche (foglie) =========

class TimedBaseCmd(py_trees.behaviour.Behaviour):
    """Pubblica cmd_vel per (duration)s su (left/right), poi SUCCESS."""
    def __init__(self, name: str, node: DemoNode, side: str, vel_xy: Tuple[float,float], duration: float):
        super().__init__(name=name)
        self.node = node
        self.side = side
        self.vx, self.vy = vel_xy
        self.duration = duration
        self.t0 = 0.0

    def initialise(self):
        self.t0 = time.time()
        self.node.get_logger().info(f"[{self.name}] INITIALISE: vx={self.vx}, vy={self.vy}, duration={self.duration}")

    def update(self) -> Status:
        elapsed = time.time() - self.t0
        msg = Twist()
        msg.linear.x, msg.linear.y = float(self.vx), float(self.vy)
        if self.side == "left":
            self.node.left_base_pub.publish(msg)
            self.node.get_logger().debug(f"[{self.name}] Publishing to LEFT base: x={self.vx}, y={self.vy}, elapsed={elapsed:.2f}s")
        else:
            self.node.right_base_pub.publish(msg)
            self.node.get_logger().debug(f"[{self.name}] Publishing to RIGHT base: x={self.vx}, y={self.vy}, elapsed={elapsed:.2f}s")
        if elapsed >= self.duration:
            self.node.get_logger().info(f"[{self.name}] Duration reached ({elapsed:.2f}s >= {self.duration}s), returning SUCCESS")
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status: Status):
        if new_status in (Status.SUCCESS, Status.FAILURE):
            self.node.get_logger().info(f"[{self.name}] TERMINATE: new_status={new_status}, calling stop_all_movement()")
            self.node.stop_all_movement()


class TimedArmCmd(py_trees.behaviour.Behaviour):
    """Pubblica comandi giunti (Float64MultiArray) per (duration)s su (left/right), poi SUCCESS."""
    def __init__(self, name: str, node: DemoNode, side: str, joints: List[float], duration: float):
        super().__init__(name=name)
        self.node = node
        self.side = side
        self.joints = joints
        self.duration = duration
        self.t0 = 0.0

    def initialise(self):
        self.t0 = time.time()
        self.node.get_logger().info(f"[{self.name}] INITIALISE: joints={self.joints}, duration={self.duration}")

    def update(self) -> Status:
        elapsed = time.time() - self.t0
        msg = Float64MultiArray()
        msg.data = list(map(float, self.joints))
        if self.side == "left":
            self.node.left_arm_pub.publish(msg)
            self.node.get_logger().debug(f"[{self.name}] Publishing to LEFT arm: joints={self.joints}, elapsed={elapsed:.2f}s")
        else:
            self.node.right_arm_pub.publish(msg)
            self.node.get_logger().debug(f"[{self.name}] Publishing to RIGHT arm: joints={self.joints}, elapsed={elapsed:.2f}s")
        if elapsed >= self.duration:
            self.node.get_logger().info(f"[{self.name}] Duration reached ({elapsed:.2f}s >= {self.duration}s), returning SUCCESS")
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status: Status):
        if new_status in (Status.SUCCESS, Status.FAILURE):
            self.node.get_logger().info(f"[{self.name}] TERMINATE: new_status={new_status}, calling stop_all_movement()")
            self.node.stop_all_movement()


class InstantSuccess(py_trees.behaviour.Behaviour):
    """SUCCESS immediato (placeholder per FindObj, CheckAlignment, Controller, ecc.)."""
    def update(self) -> Status:
        return Status.SUCCESS

class InstantFailure(py_trees.behaviour.Behaviour):
    def update(self) -> Status:
        return Status.FAILURE

class BlackboardFlag(py_trees.behaviour.Behaviour):
    """Setta/attende un flag su blackboard (sincronizzazioni, dati scambiati)."""
    def __init__(self, name: str, key: str, mode: str, verbose: bool=False):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        # py_trees versions differ: some don't have READ_WRITE. Use a safe fallback.
        try:
            access = py_trees.common.Access.READ_WRITE
        except Exception:
            # fallback: map mode to READ or WRITE
            if mode == "set":
                access = py_trees.common.Access.WRITE
            else:
                access = py_trees.common.Access.READ
        self.bb.register_key(key=key, access=access)
        self.key = key
        self.mode = mode
        self.verbose = verbose
        self._last_seen = None  # per log solo al cambio

    def _log(self, msg: str):
        if not self.verbose:
            return
        node = getattr(self, 'node', None)
        (node.get_logger().info if node else print)(msg)

    def update(self) -> Status:
        # Log dettagliato per debug
        node = getattr(self, 'node', None)
        if self.mode == "set":
            setattr(self.bb, self.key, True)
            self._log(f"[BlackboardFlag] SET {self.key} = True")
            return Status.SUCCESS
        # modalità check_true -> SUCCESS se True, altrimenti FAILURE
        if self.mode == "check_true":
            try:
                val = getattr(self.bb, self.key)
            except KeyError:
                val = False
            if val != self._last_seen:
                self._last_seen = val
                self._log(f"[BlackboardFlag] {self.mode.upper()} {self.key} = {val}")
            return Status.SUCCESS if val else Status.FAILURE
        try:
            val = getattr(self.bb, self.key)
        except KeyError:
            val = False
        if val != self._last_seen:
            self._last_seen = val
            self._log(f"[BlackboardFlag] {self.mode.upper()} {self.key} = {val}")
        return Status.SUCCESS if val else Status.RUNNING


# ========= Azioni high-level (compongono base+braccia+servizi come nella SM) =========

def behaviour_approach(node: DemoNode) -> py_trees.behaviour.Behaviour:
    """Avvicinamento: basi + velocità bracci per APPROACH_TIME (come la SM)."""
    par = py_trees.composites.Parallel(name="approach", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    par.add_children([
        TimedBaseCmd("L_base_approach", node, "left",  APPROACH_LEFT_BASE_XY_VEL,  APPROACH_TIME),
        TimedBaseCmd("R_base_approach", node, "right", APPROACH_RIGHT_BASE_XY_VEL, APPROACH_TIME),
        TimedArmCmd ("L_arm_vel", node, "left",  LEFT_ARM_VEL,  APPROACH_TIME),
        TimedArmCmd ("R_arm_vel", node, "right", RIGHT_ARM_VEL, APPROACH_TIME),
    ])
    # Al termine, disabilita collisioni palette (come nella SM) e continua
    post = py_trees.composites.Sequence(name="post_approach", memory=False)
    post.add_children([
        par,
        py_trees.decorators.EternalGuard(
            name="disable_collisions_once",
            condition=lambda: True,
            blackboard_keys=[],
            child=py_trees.composites.Sequence(name="disable_collisions_seq", memory=False, children=[
                py_trees.behaviours.Success(name="disable_left")  # placeholder: faccio in terminate()
            ])
        )
    ])
    # Faccio il toggle collisioni nel terminate() di un piccolo wrapper
    class DisableCollisions(py_trees.behaviour.Behaviour):
        def __init__(self, name, node: DemoNode):
            super().__init__(name=name); self.node=node
        def update(self): return Status.SUCCESS
        def terminate(self, new_status):
            if new_status == Status.SUCCESS:
                self.node.set_paletta_collision(False, "left")
                self.node.set_paletta_collision(False, "right")
    return py_trees.composites.Sequence(name="ApproachSeq", memory=False, children=[post, DisableCollisions("DisableCollisions", node)])

def behaviour_descend_and_pick(node: DemoNode) -> py_trees.behaviour.Behaviour:
    """Calata e attach fisico + gravity OFF (fine step)."""
    par = py_trees.composites.Parallel(name="descend_and_pick", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    par.add_children([
        TimedArmCmd("L_arm_pick", node, "left",  LEFT_ARM_PICK,  DESCEND_AND_PICK_TIME),
        TimedArmCmd("R_arm_pick", node, "right", RIGHT_ARM_PICK, DESCEND_AND_PICK_TIME),
    ])
    class DoAttach(py_trees.behaviour.Behaviour):
        def __init__(self, node): super().__init__("AttachAndNoGravity"); self.node=node
        def update(self): return Status.SUCCESS
        def terminate(self, new_status):
            if new_status == Status.SUCCESS:
                if self.node.attach_fixed_joint():
                    self.node.get_logger().info("Attach OK")
                else:
                    self.node.get_logger().warn("Attach fallito (proseguo)")
                self.node.set_pacco_gravity(False)
    return py_trees.composites.Sequence(name="DescendAndPickSeq", memory=True, children=[par, DoAttach(node)])

def behaviour_collect(node: DemoNode) -> py_trees.behaviour.Behaviour:
    par = py_trees.composites.Parallel(name="collect", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    par.add_children([
        TimedBaseCmd("L_base_collect", node, "left",  COLLECT_LEFT_BASE_XY_VEL,  COLLECT_TIME),
        TimedBaseCmd("R_base_collect", node, "right", COLLECT_RIGHT_BASE_XY_VEL, COLLECT_TIME),
        TimedArmCmd ("L_arm_collect",  node, "left",  LEFT_ARM_COLLECT,  COLLECT_TIME),
        TimedArmCmd ("R_arm_collect",  node, "right", RIGHT_ARM_COLLECT, COLLECT_TIME),
    ])
    return par

def behaviour_transport(node: DemoNode) -> py_trees.behaviour.Behaviour:
    par = py_trees.composites.Parallel(name="transport", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    par.add_children([
        TimedBaseCmd("L_base_transport", node, "left",  LEFT_TRANSPORT_VEL_XY,  TRANSPORT_TIME),
        TimedBaseCmd("R_base_transport", node, "right", RIGHT_TRANSPORT_VEL_XY, TRANSPORT_TIME),
        # Bracci: in SM durante transport mantieni, qui pubblichiamo 0
    ])
    return par

def behaviour_descend_and_place(node: DemoNode) -> py_trees.behaviour.Behaviour:
    par = py_trees.composites.Parallel(name="descend_and_place", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    par.add_children([
        TimedBaseCmd("L_base_place", node, "left",  PLACE_LEFT_BASE_XY_VEL,  DESCEND_AND_PLACE_TIME),
        TimedBaseCmd("R_base_place", node, "right", PLACE_RIGHT_BASE_XY_VEL, DESCEND_AND_PLACE_TIME),
        TimedArmCmd ("L_arm_place",  node, "left",  LEFT_ARM_PLACE,  DESCEND_AND_PLACE_TIME),
        TimedArmCmd ("R_arm_place",  node, "right", RIGHT_ARM_PLACE, DESCEND_AND_PLACE_TIME),
    ])
    return par

def behaviour_release(node: DemoNode) -> py_trees.behaviour.Behaviour:
    arms = py_trees.composites.Parallel(name="release", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    arms.add_children([
        TimedArmCmd("L_arm_release", node, "left",  LEFT_ARM_RELEASE,  RELEASE_TIME),
        TimedArmCmd("R_arm_release", node, "right", RIGHT_ARM_RELEASE, RELEASE_TIME),
    ])
    class DoDetach(py_trees.behaviour.Behaviour):
        def __init__(self, node): super().__init__("DetachAndGravity"); self.node=node
        def update(self): return Status.SUCCESS
        def terminate(self, new_status):
            if new_status == Status.SUCCESS:
                self.node.set_paletta_collision(True, "left")
                ok = self.node.detach_fixed_joint()
                if ok: self.node.get_logger().info("Detach OK")
                else:  self.node.get_logger().warn("Detach fallito")
                self.node.set_pacco_gravity(True)
    return py_trees.composites.Sequence(name="ReleaseSeq", memory=False, children=[arms, DoDetach(node)])

def behaviour_getaway(node: DemoNode) -> py_trees.behaviour.Behaviour:
    par = py_trees.composites.Parallel(name="getaway", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    par.add_children([
        TimedBaseCmd("L_base_getaway", node, "left",  COLLECT_LEFT_BASE_XY_VEL,  COLLECT_TIME),
        TimedBaseCmd("R_base_getaway", node, "right", COLLECT_RIGHT_BASE_XY_VEL, COLLECT_TIME),
        TimedArmCmd ("L_arm_hold",     node, "left",  LEFT_ARM_COLLECT,  COLLECT_TIME),
        TimedArmCmd ("R_arm_hold",     node, "right", RIGHT_ARM_COLLECT, COLLECT_TIME),
    ])
    return par


# ========= Costruzione degli alberi (mappati dai file XML) =========

def selector_fallback(name: str, children):
    sel = py_trees.composites.Selector(name=name, memory=False)
    sel.add_children(children)
    return sel

def build_srm1_tree(node: DemoNode) -> py_trees.behaviour.Behaviour:
    # SRM1 (:contentReference[oaicite:3]{index=3})
    find_obj   = InstantSuccess(name="SRM1.FindObj")
    calc_goal  = InstantSuccess(name="SRM1.CalculateGoal")
    # Nuova logica: fallback controlla un flag su blackboard
    approach_done_flag = BlackboardFlag("SRM1.ApproachDone", key="srm1_approach_done", mode="check_true")
    approach = behaviour_approach(node)
    class SetApproachDone(py_trees.behaviour.Behaviour):
        def __init__(self, node):
            super().__init__("SRM1.SetApproachDone"); self.node = node
            self.bb = self.attach_blackboard_client(name=self.name)
            self.bb.register_key(key="srm1_approach_done", access=py_trees.common.Access.WRITE)
        def update(self):
            self.node.get_logger().info("[SRM1.SetApproachDone] Setting srm1_approach_done = True")
            self.bb.srm1_approach_done = True
            # Immediately return SUCCESS so fallback can exit
            return py_trees.common.Status.SUCCESS
        def terminate(self, new_status):
            if new_status == Status.FAILURE:
                self.node.get_logger().warn("[SRM1.SetApproachDone] FAILURE: Forzo srm1_approach_done = True")
                self.bb.srm1_approach_done = True
    set_approach_done = SetApproachDone(node)
    approach_seq = py_trees.composites.Sequence(name="SRM1_ApproachSeq", memory=False)
    approach_seq.add_children([approach, set_approach_done])
    guarded_approach = py_trees.decorators.EternalGuard(
        name="SRM1_RunApproachUntilDone",
        condition=lambda: not getattr(py_trees.blackboard.Blackboard(), "srm1_approach_done", False),
        blackboard_keys=["srm1_approach_done"],
        child=approach_seq
    )
    approach_fb = selector_fallback("SRM1.Fallback(AtPallet?)", [approach_done_flag, guarded_approach])
    say_data   = BlackboardFlag("SRM1.Say(DataToSRM2)", key="srm1_data_sent", mode="set")
    wait_srm2  = BlackboardFlag("SRM1.Wait(SRM2@Pallet)", key="srm2_near_pallet", mode="wait_true")
    sync       = BlackboardFlag("SRM1.Sync", key="positioning_sync_srm1", mode="set")
    seq_load   = py_trees.composites.Sequence(name="SRM1_to_LoadSite", memory=False)
    seq_load.add_children([approach_fb, say_data, wait_srm2, sync])
    root = py_trees.composites.Sequence(name="scenario1-SRM1", memory=True)
    root.add_children([find_obj, calc_goal, seq_load])
    return root

def build_srm2_tree(node: DemoNode) -> py_trees.behaviour.Behaviour:
    # SRM2 (:contentReference[oaicite:4]{index=4})
    wait_data  = BlackboardFlag("SRM2.Wait(DataFromSRM1)", key="srm1_data_sent", mode="wait_true")
    # Nuova logica: fallback controlla un flag su blackboard
    approach_done_flag = BlackboardFlag("SRM2.ApproachDone", key="srm2_approach_done", mode="check_true")
    approach = behaviour_approach(node)
    class SetApproachDone(py_trees.behaviour.Behaviour):
        def __init__(self, node):
            super().__init__("SRM2.SetApproachDone"); self.node = node
            self.bb = self.attach_blackboard_client(name=self.name)
            self.bb.register_key(key="srm2_approach_done", access=py_trees.common.Access.WRITE)
        def update(self):
            self.node.get_logger().info("[SRM2.SetApproachDone] Setting srm2_approach_done = True")
            self.bb.srm2_approach_done = True
            # Immediately return SUCCESS so fallback can exit
            return py_trees.common.Status.SUCCESS
        def terminate(self, new_status):
            if new_status == Status.FAILURE:
                self.node.get_logger().warn("[SRM2.SetApproachDone] FAILURE: Forzo srm2_approach_done = True")
                self.bb.srm2_approach_done = True
    set_approach_done = SetApproachDone(node)
    approach_seq = py_trees.composites.Sequence(name="SRM2_ApproachSeq", memory=False)
    approach_seq.add_children([approach, set_approach_done])
    guarded_approach = py_trees.decorators.EternalGuard(
        name="SRM2_RunApproachUntilDone",
        condition=lambda: not getattr(py_trees.blackboard.Blackboard(), "srm2_approach_done", False),
        blackboard_keys=["srm2_approach_done"],
        child=approach_seq
    )
    approach_fb = selector_fallback("SRM2.Fallback(AtPallet?)", [approach_done_flag, guarded_approach])
    mark_near  = BlackboardFlag("SRM2.MarkNear", key="srm2_near_pallet", mode="set")
    sync       = BlackboardFlag("SRM2.Sync", key="positioning_sync_srm2", mode="set")
    root = py_trees.composites.Sequence(name="scenario1-SRM2", memory=False)
    root.add_children([wait_data, approach_fb, mark_near, sync])
    return root


def make_step_once(name: str, step_behaviour: py_trees.behaviour.Behaviour, done_key: str):
    check = BlackboardFlag(name=f"{name}?done", key=done_key, mode="check_true")
    set_done = BlackboardFlag(name=f"{name}.set_done", key=done_key, mode="set")
    seq = py_trees.composites.Sequence(name=f"{name}.seq", memory=True)
    seq.add_children([step_behaviour, set_done])
    # Selector reattivo: se già fatto -> SUCCESS, altrimenti esegui seq
    sel = selector_fallback(name=f"{name}.once", children=[check, seq])
    return sel

def build_supervisor_tree(node: DemoNode) -> py_trees.behaviour.Behaviour:
    # Supervisor (:contentReference[oaicite:5]{index=5})
    # Attesa sincronizzazione SRM1-SRM2
    class WaitBoth(py_trees.behaviour.Behaviour):
        def __init__(self):
            super().__init__("Supervisor.Wait(SyncSRM1-SRM2)")
            self.bb = self.attach_blackboard_client(name=self.name)
            # Access enum may differ across py_trees versions; fallback to READ
            try:
                access_ro = py_trees.common.Access.READ_ONLY
            except Exception:
                access_ro = py_trees.common.Access.READ
            self.bb.register_key("positioning_sync_srm1", access=access_ro)
            self.bb.register_key("positioning_sync_srm2", access=access_ro)
        def update(self) -> Status:
            try:
                v1 = getattr(self.bb, "positioning_sync_srm1")
            except KeyError:
                v1 = False
            try:
                v2 = getattr(self.bb, "positioning_sync_srm2")
            except KeyError:
                v2 = False
            if v1 and v2:
                return Status.SUCCESS
            return Status.RUNNING

    identify_fb = selector_fallback("IdentifyPkg", [InstantSuccess("FindObj"), InstantSuccess("Say(CallOperator)")])
    calc_goal   = InstantSuccess("CalculateGoal(PkgPos)")
    align_fb    = selector_fallback("AlignOrMove", [InstantSuccess("CheckAlignment(ARM1-ARM2)"), InstantSuccess("MoveBase(CorrectBasePos)")])


    # TODO: ricostruire TransportSequence e DropSequence come in XML

    # transport_seq = py_trees.composites.Sequence(name="TransportSequence", memory=True)
    # transport_seq.add_children([
    #     selector_fallback("LiftOrCall", [InstantSuccess("LiftObj"), InstantSuccess("Say(PackageLost)")]),
    #     # ParallelAll semplificato: inizializzazioni -> move base (simulato)
    #     InstantSuccess("InitControllers"),
    #     InstantSuccess("MoveBase(DST)"),
    #     # DropSequence
    #     py_trees.composites.Sequence(name="DropSequence", memory=True, children=[
    #         selector_fallback("DropAlign", [InstantSuccess("CheckAlignment(DropPos)"), InstantSuccess("MoveBase(Adjust)")]),
    #         InstantSuccess("Drop"),  # simulato
    #         behaviour_release(node),
    #     ])
    # ])

    action_pipeline = py_trees.composites.Sequence(name="ActionPipeline", memory=True)
    action_pipeline.add_children([
        make_step_once("descend_and_pick",   behaviour_descend_and_pick(node),   "done_descend_and_pick"),
        make_step_once("collect",            behaviour_collect(node),            "done_collect"),
        make_step_once("transport",          behaviour_transport(node),          "done_transport"),
        make_step_once("descend_and_place",  behaviour_descend_and_place(node),  "done_descend_and_place"),
        make_step_once("release",            behaviour_release(node),            "done_release"),
        make_step_once("getaway",            behaviour_getaway(node),            "done_getaway"),
    ])
    # al termine della pipeline, marca la missione come completata
    set_mission_done = BlackboardFlag("mission.set_done", key="mission_done", mode="set")


    pick_prep = py_trees.composites.Sequence(name="Pick-upPrepSequence", memory=True)
    pick_prep.add_children([
        selector_fallback("EEReady", [InstantSuccess("CheckAlignment(EE)"), InstantSuccess("Set(EE)")]),
        InstantSuccess("CloseGripper"),
        # ParallelAll semplificato -> esegui la pipeline principale
        action_pipeline,
        set_mission_done,
        InstantSuccess("MoveBase(ReturnToPallet)")
    ])

    root = py_trees.composites.Sequence(name="scenario1-SRMSupervisor", memory=False)
    root.add_children([WaitBoth(), identify_fb, calc_goal, align_fb, pick_prep])
    return root


# ========= Runner: tick BT con un timer ROS2 =========

class BTRunner(Node):
    def __init__(self, demo_node: DemoNode, tick_hz: float = 10.0):
        super().__init__("bt_runner")
        self.demo = demo_node
        self.srm1_tree = py_trees.trees.BehaviourTree(root=build_srm1_tree(demo_node))
        self.srm2_tree = py_trees.trees.BehaviourTree(root=build_srm2_tree(demo_node))
        self.sup_tree  = py_trees.trees.BehaviourTree(root=build_supervisor_tree(demo_node))

        # handler stampa (opzionale) - passiamo il root node a unicode_tree
        for t in (self.srm1_tree, self.srm2_tree, self.sup_tree):
            # py_trees.display.unicode_tree expects a Behaviour (root),
            # while add_post_tick_handler will be called with the BehaviourTree.
            # Wrap it so the display function receives the tree root.
            # t.add_post_tick_handler(lambda tree, handler=py_trees.display.unicode_tree: handler(tree.root))
            
            ### PRINT DEBUG TREE ###
            # t.add_post_tick_handler(
            #     lambda tree: print(py_trees.display.unicode_tree(tree.root, show_status=True))
            # )
            pass


        self.timer = self.create_timer(1.0/max(1e-3, tick_hz), self.on_tick)
        self.get_logger().info(f"BT runner avviato @ {tick_hz:.1f} Hz")

    def on_tick(self):
        # Nota: tutti i BT condividono la blackboard globale -> sync via flags
        self.srm1_tree.tick()
        self.srm2_tree.tick()
        self.sup_tree.tick()


def parse_args():
    parser = argparse.ArgumentParser(description="BT-controlled Pick&Place demo (ROS 2 Humble)")
    # (stessi argomenti e default della demo pick and place) :contentReference[oaicite:6]{index=6}
    parser.add_argument('--object', default='pacco_clone_1')
    parser.add_argument('--object_link', default='pacco_clone_1::link_1')
    parser.add_argument('--attach_robot_model', default='left_robot')
    parser.add_argument('--attach_robot_link', default='ur_left_wrist_3_link')
    parser.add_argument('--left_base_topic',  default='/left_summit_cmd_vel')
    parser.add_argument('--right_base_topic', default='/right_summit_cmd_vel')
    parser.add_argument('--left_arm_topic',   default='/left/ur_left_joint_group_vel_controller/commands')
    parser.add_argument('--right_arm_topic',  default='/right/ur_right_joint_group_vel_controller/commands')
    parser.add_argument('--left_ee', default='ur_left_paletta')
    parser.add_argument('--right_ee', default='ur_right_paletta')
    parser.add_argument('--tick_hz', type=float, default=10.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    demo_node = DemoNode(args)
    bt_runner = BTRunner(demo_node, tick_hz=args.tick_hz)

    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(demo_node)
        executor.add_node(bt_runner)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bt_runner.destroy_node()
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
