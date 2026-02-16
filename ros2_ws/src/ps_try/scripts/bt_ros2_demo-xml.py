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
from typing import Dict, List, Optional

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

import numpy as np
import math

#sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Task Prioritization è in /TaskPrioritization (sibling di /ros2_ws nel container)
# Individua la cartella anche da install space colcon e la aggiunge al sys.path
def _ensure_task_prioritization_on_path():
    here = os.path.abspath(os.path.dirname(__file__))
    candidates = [
        "/TaskPrioritization",  # posizione standard nel docker
        "/ros2_ws/TaskPrioritization",
        "/ros2_ws/src/TaskPrioritization",
        os.path.abspath(os.path.join(here, "..", "..", "..", "TaskPrioritization")),
        os.path.abspath(os.path.join(here, "..", "..", "TaskPrioritization")),
        os.path.abspath(os.path.join(here, "..", "TaskPrioritization")),
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
APPROACH_TEST_DURATION = 6.0
APPROACH_TEST_TRAJ_TIME = 5.0
APPROACH_ARM_CMD_ABS_MAX = 0.25
APPROACH_BASE_CMD_XY_ABS_MAX = 0.05
APPROACH_BASE_CMD_WZ_ABS_MAX = 0.20
APPROACH_BASE_CMD_RAMP_TIME = 1.0




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
    @staticmethod
    def _pick_existing_file(candidates: List[str]) -> Optional[str]:
        for path in candidates:
            if path and os.path.isfile(path):
                return path
        return None

    @staticmethod
    def _normalize_joint_name(name: str) -> str:
        return str(name).split("/")[-1]

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
        self._last_joint_states = {"left": None, "right": None, "global": None}
        self._warn_timestamps: Dict[str, float] = {}
        self.approach_test_duration = APPROACH_TEST_DURATION

        # subscribe (choose actual topic names in your system)
        self.create_subscription(Odometry, '/left_summit_odom', lambda msg: self._odom_cb(msg, 'left'), 10)
        self.create_subscription(Odometry, '/right_summit_odom', lambda msg: self._odom_cb(msg, 'right'), 10)
        self.create_subscription(JointState, '/left/joint_states', lambda msg: self._joint_states_cb(msg, 'left'), 10)
        self.create_subscription(JointState, '/right/joint_states', lambda msg: self._joint_states_cb(msg, 'right'), 10)
        # Fallback: in alcune configurazioni ROS2 c'è un solo /joint_states aggregato.
        self.create_subscription(JointState, '/joint_states', self._joint_states_global_cb, 10)

        # Services opzionali per la demo (se non ci sono logga warning)
        self.attach_cli = self.create_client(AttachLink, "/ATTACHLINK")
        self.detach_cli = self.create_client(DetachLink, "/DETACHLINK")
        self.toggle_gravity_cli = self.create_client(SetLinkGravity, "/set_link_gravity")
        self.toggle_collision_cli = self.create_client(SetCollisionEnabled, "/set_collision_enabled")

        # Config path robusti: install-space, source-space e fallback assoluti.
        try:
            pkg_share = get_package_share_directory("ps_try")
        except Exception:
            pkg_share = None

        robot_cfg_env = os.getenv("SIMOD_TP_ROBOT_CFG", "").strip()
        if robot_cfg_env and not os.path.isfile(robot_cfg_env):
            self.get_logger().warn(f"SIMOD_TP_ROBOT_CFG not found: {robot_cfg_env}")
        robot_cfg = self._pick_existing_file([
            robot_cfg_env,
            "/ros2_ws/src/ps_try/config/dual_paletta.yaml",
            os.path.join(pkg_share, "config", "dual_paletta.yaml") if pkg_share else "",
            "/ros2_ws/TaskPrioritization/default_robots/__paletta__/dual_paletta.yaml",
            "/ros2_ws/src/TaskPrioritization/default_robots/__paletta__/dual_paletta.yaml",
            "/TaskPrioritization/default_robots/__paletta__/dual_paletta.yaml",
        ])
        if robot_cfg is None:
            raise FileNotFoundError("dual_paletta.yaml non trovato (ps_try/config o default_robots)")

        ee_cfg_env = os.getenv("SIMOD_TP_EE_CFG", "").strip()
        if ee_cfg_env and not os.path.isfile(ee_cfg_env):
            self.get_logger().warn(f"SIMOD_TP_EE_CFG not found: {ee_cfg_env}")
        ee_cfg = self._pick_existing_file([
            ee_cfg_env,
            "/ros2_ws/src/ps_try/config/ee_task.yaml",
            os.path.join(pkg_share, "config", "ee_task.yaml") if pkg_share else "",
        ])
        if ee_cfg is None:
            raise FileNotFoundError("ee_task.yaml non trovato (ps_try/config)")

        self.tp = TPManager(device='cuda', dtype='float32') # change device='cpu' to 'cuda' if GPU is available
        self.tp.add_robot(robot_name='__paletta__', config_file=robot_cfg)
        # Viewer TP (socket PyVistaQt). In Docker avvialo nello stesso container per avere path mesh validi.
        enable_tp_viewer = os.getenv("SIMOD_TP_VIEWER", "1").strip().lower() in ("1", "true", "yes", "on")
        if enable_tp_viewer:
            try:
                viewer_host = os.getenv("SIMOD_TP_VIEWER_HOST", "127.0.0.1")
                viewer_port = int(os.getenv("SIMOD_TP_VIEWER_PORT", "55001"))
                viewer_hz = float(os.getenv("SIMOD_TP_VIEWER_HZ", "60"))
                launch_viewer = os.getenv("SIMOD_TP_VIEWER_AUTOSTART", "1").strip().lower() in ("1", "true", "yes", "on")
                self.get_logger().info(
                    f"Connecting TP viewer host={viewer_host} port={viewer_port} hz={viewer_hz} autostart={launch_viewer}"
                )
                self.tp.connect_visualizer(
                    host=viewer_host,
                    port=viewer_port,
                    hz=viewer_hz,
                    launch_viewer=launch_viewer,
                    viewer_wait_timeout_s=8.0,
                )
                chain = getattr(self.tp, "chain", None)
                bridge = getattr(chain, "viewer_bridge", None) if chain is not None else None
                is_running = bool(bridge is not None and getattr(bridge, "is_running", lambda: False)())
                self.get_logger().info(f"TP viewer bridge running: {is_running}")
            except Exception as exc:
                self.get_logger().warn(f"TP viewer disabled after connection error: {exc}")
        else:
            self.get_logger().info("TP viewer disabled (set SIMOD_TP_VIEWER=1 to enable)")
        self.ee_task = self.tp.add_EE_task(ee_cfg)
        # Disattiva eventuali piani di traiettoria pre-caricati da YAML, così i valori di test non vengono sovrascritti al primo cycle.
        if hasattr(self.ee_task, "set_trajectory_plan"):
            self.ee_task.set_trajectory_plan(None)
        # initialize robot kinematics before using it
        self.robot = self.tp.get_robot_kinematics()
        self.tr_left: Optional[Trajectory] = None
        self.tr_right: Optional[Trajectory] = None

        # Approach-only test setup (trajectory verrà inizializzata al primo tick con stato reale da Gazebo).
        self._approach_left_delta = np.array([0.05, 0.05, 0.04, 0.3, 0.3, 0.0], dtype=np.float32)
        self._approach_right_delta = np.array([0.05, 0.05, 0.04, 0.3, 0.4, 0.0], dtype=np.float32)
        self._tp_approach_traj_initialized = False
        self._tp_last_exec_monotonic: Optional[float] = None
        self._tp_cmd_cache = None
        self._tp_cmd_cache_time: Optional[float] = None
        self.tp_cmd_min_period = float(os.getenv("SIMOD_TP_CMD_MIN_PERIOD", "0.03"))
        self.tp_arm_cmd_abs_max = float(os.getenv("SIMOD_TP_ARM_CMD_MAX", str(APPROACH_ARM_CMD_ABS_MAX)))
        self.approach_use_base = os.getenv("SIMOD_APPROACH_USE_BASE", "1").strip().lower() in ("1", "true", "yes", "on")
        self.tp_base_cmd_xy_abs_max = float(os.getenv("SIMOD_TP_BASE_CMD_XY_MAX", str(APPROACH_BASE_CMD_XY_ABS_MAX)))
        self.tp_base_cmd_wz_abs_max = float(os.getenv("SIMOD_TP_BASE_CMD_WZ_MAX", str(APPROACH_BASE_CMD_WZ_ABS_MAX)))
        self.tp_base_cmd_ramp_time = float(os.getenv("SIMOD_TP_BASE_CMD_RAMP_TIME", str(APPROACH_BASE_CMD_RAMP_TIME)))
        self.approach_only_mode = os.getenv("SIMOD_APPROACH_ONLY", "1").strip().lower() in ("1", "true", "yes", "on")
        if self.approach_only_mode:
            self.get_logger().info("Approach-only mode active: Lift/Transport/Drop are paused")
        self.get_logger().info(
            "Approach TP config: "
            f"use_base={self.approach_use_base}, "
            f"arm_clip={self.tp_arm_cmd_abs_max:.3f}, "
            f"base_clip_xy={self.tp_base_cmd_xy_abs_max:.3f}, "
            f"base_clip_wz={self.tp_base_cmd_wz_abs_max:.3f}, "
            f"base_ramp={self.tp_base_cmd_ramp_time:.2f}s"
        )
        if hasattr(self.ee_task, "set_use_base"):
            self.ee_task.set_use_base(self.approach_use_base)
        else:
            self.ee_task.use_base = self.approach_use_base
        self.ee_task.set_trajectory([None, None])

        self._expected_arm_joint_names = {"left": [], "right": []}
        self._build_expected_joint_map()
        self._tp_layout = {}
        self._build_tp_layout_map()
        self._arm_ee_index = {"left": 0, "right": 1}
        self._build_arm_ee_index_map()
        
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

        self.get_logger().info(f"BTDemoNode inizializzato - robot_cfg={robot_cfg}, ee_cfg={ee_cfg}")

    @staticmethod
    def _infer_side_from_text(text: str) -> Optional[str]:
        txt = str(text).lower()
        has_left = "left" in txt
        has_right = "right" in txt
        if has_left and not has_right:
            return "left"
        if has_right and not has_left:
            return "right"
        return None

    def _build_tp_layout_map(self):
        """
        Costruisce una mappa robusta tra side (left/right) e:
        - indice sottosistema (base/arm) usato da TP.update_infos
        - indice gruppo base nel vettore base_odom
        - slice nel vettore comando TP (dqd)
        """
        chain = getattr(self.tp, "chain", None)
        if chain is None:
            self._tp_layout = {}
            return

        list_of_lists_joint = list(getattr(chain, "list_of_lists_joint", []) or [])
        ordered_names = list(getattr(chain.robots_cl, "ordered_list_robot_names", []) or [])

        layout = {
            "num_subsystems": int(getattr(chain, "num_of_subsystems", 0)),
            "num_groups": len(list_of_lists_joint),
            "subsystem_sizes": [],
            "base_sub_idx": {"left": None, "right": None},
            "arm_sub_idx": {"left": None, "right": None},
            "base_group_idx": {"left": None, "right": None},
            "base_cmd_slice": {"left": None, "right": None},
            "arm_cmd_slice": {"left": None, "right": None},
        }

        sub_idx = 0
        cmd_off = 0
        for gidx, assembly in enumerate(list_of_lists_joint):
            names = ordered_names[gidx] if gidx < len(ordered_names) else []
            name_ptr = 0
            for slot, dof in enumerate(assembly):
                if dof is None:
                    continue
                dof_i = int(dof)
                comp_name = names[name_ptr] if name_ptr < len(names) else f"group{gidx}_slot{slot}"
                name_ptr += 1

                side = self._infer_side_from_text(comp_name)
                if side is not None:
                    if slot == 0:  # base
                        layout["base_sub_idx"][side] = sub_idx
                        layout["base_group_idx"][side] = gidx
                        layout["base_cmd_slice"][side] = (cmd_off, cmd_off + dof_i)
                    elif slot == 1:  # arm
                        layout["arm_sub_idx"][side] = sub_idx
                        layout["arm_cmd_slice"][side] = (cmd_off, cmd_off + dof_i)

                layout["subsystem_sizes"].append(dof_i)
                sub_idx += 1
                cmd_off += dof_i

        self._tp_layout = layout
        self.get_logger().info(
            "TP layout map - "
            f"base_sub_idx={layout['base_sub_idx']}, arm_sub_idx={layout['arm_sub_idx']}, "
            f"base_group_idx={layout['base_group_idx']}, arm_cmd_slice={layout['arm_cmd_slice']}"
        )

    def _build_arm_ee_index_map(self):
        """
        Mappa side->indice nel vettore EE restituito da robot.get_arm_ee_poses().
        Riduce i rischi di inversione left/right quando l'ordine interno cambia.
        """
        arm_names = list(getattr(self.robot, "arm_names", []) or [])
        idx_map: Dict[str, Optional[int]] = {"left": None, "right": None}

        for i, arm_name in enumerate(arm_names):
            side = self._infer_side_from_text(arm_name)
            if side is not None and idx_map[side] is None:
                idx_map[side] = i

        if idx_map["left"] is None and len(arm_names) >= 1:
            idx_map["left"] = 0
        if idx_map["right"] is None and len(arm_names) >= 2:
            idx_map["right"] = 1 if idx_map["left"] != 1 else 0

        self._arm_ee_index = {
            "left": int(idx_map["left"]) if idx_map["left"] is not None else 0,
            "right": int(idx_map["right"]) if idx_map["right"] is not None else 1,
        }
        self.get_logger().info(
            "TP arm EE index map - "
            f"arm_names={arm_names}, side_to_ee_idx={self._arm_ee_index}"
        )

    def _build_tp_inputs_from_side_data(
        self,
        left_arm_jp: List[float],
        right_arm_jp: List[float],
        left_base: List[float],
        right_base: List[float],
    ):
        layout = self._tp_layout or {}
        nsubs = int(layout.get("num_subsystems", 0))
        sizes = list(layout.get("subsystem_sizes", []))
        ngroups = int(layout.get("num_groups", 0))

        if nsubs <= 0 or len(sizes) != nsubs or ngroups <= 0:
            # Fallback storico (left base, left arm, right base, right arm)
            joint_pos = [
                np.zeros(3, dtype=np.float32),
                np.asarray(left_arm_jp, dtype=np.float32),
                np.zeros(3, dtype=np.float32),
                np.asarray(right_arm_jp, dtype=np.float32),
            ]
            base_odom = [
                np.asarray(left_base, dtype=np.float32),
                np.asarray(right_base, dtype=np.float32),
            ]
            return joint_pos, base_odom

        joint_pos = [np.zeros(int(sz), dtype=np.float32) for sz in sizes]
        base_odom = [np.zeros(6, dtype=np.float32) for _ in range(ngroups)]

        arm_sub_idx = layout.get("arm_sub_idx", {})
        base_group_idx = layout.get("base_group_idx", {})

        idx_l_arm = arm_sub_idx.get("left", None)
        idx_r_arm = arm_sub_idx.get("right", None)
        if idx_l_arm is not None and 0 <= idx_l_arm < len(joint_pos):
            joint_pos[idx_l_arm] = np.asarray(left_arm_jp, dtype=np.float32)
        if idx_r_arm is not None and 0 <= idx_r_arm < len(joint_pos):
            joint_pos[idx_r_arm] = np.asarray(right_arm_jp, dtype=np.float32)

        gidx_l = base_group_idx.get("left", None)
        gidx_r = base_group_idx.get("right", None)
        if gidx_l is not None and 0 <= gidx_l < len(base_odom):
            base_odom[gidx_l] = np.asarray(left_base, dtype=np.float32)
        if gidx_r is not None and 0 <= gidx_r < len(base_odom):
            base_odom[gidx_r] = np.asarray(right_base, dtype=np.float32)

        return joint_pos, base_odom

    def _get_arm_cmd_values(self, cmd, side: str) -> List[float]:
        layout = self._tp_layout or {}
        arm_slices = layout.get("arm_cmd_slice", {})
        sl = arm_slices.get(side, None)
        if sl is not None and isinstance(sl, (list, tuple)) and len(sl) == 2:
            return _cmd_slice(cmd, int(sl[0]), int(sl[1]), 6)

        # Fallback storico (left=3:9, right=12:18)
        if side == "left":
            return _cmd_slice(cmd, 3, 9, 6)
        return _cmd_slice(cmd, 12, 18, 6)

    def _get_base_cmd_values(self, cmd, side: str) -> List[float]:
        layout = self._tp_layout or {}
        base_slices = layout.get("base_cmd_slice", {})
        sl = base_slices.get(side, None)
        if sl is not None and isinstance(sl, (list, tuple)) and len(sl) == 2:
            return _cmd_slice(cmd, int(sl[0]), int(sl[1]), 3)

        # Fallback storico (left=0:3, right=9:12)
        if side == "left":
            return _cmd_slice(cmd, 0, 3, 3)
        return _cmd_slice(cmd, 9, 12, 3)

    def _init_tp_approach_trajectory_from_live_state(
        self,
        left_arm_jp: List[float],
        right_arm_jp: List[float],
        left_base: List[float],
        right_base: List[float],
    ) -> bool:
        """
        Inizializza la traiettoria di test sulla posa EE reale corrente (non su stato YAML).
        """
        try:
            joint_pos, base_odom = self._build_tp_inputs_from_side_data(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            )

            # Porta il modello cinematico TP nello stato corrente letto dai topic ROS.
            self.robot.compute(joint_pos, base_odom)
            live_ee = self.robot.get_arm_ee_poses()
            if live_ee is None or len(live_ee) == 0:
                raise RuntimeError("robot.get_arm_ee_poses() returned no arm poses")

            l_idx = int((self._arm_ee_index or {}).get("left", 0))
            r_idx = int((self._arm_ee_index or {}).get("right", 1))
            if l_idx < 0 or l_idx >= len(live_ee):
                l_idx = 0
            if r_idx < 0 or r_idx >= len(live_ee):
                r_idx = min(1, len(live_ee) - 1)
            left_ee = np.asarray(live_ee[l_idx], dtype=np.float32)
            right_ee = np.asarray(live_ee[r_idx], dtype=np.float32)

            self.tr_left = Trajectory()
            self.tr_right = Trajectory()
            self.tr_left.poly5(
                p_i=left_ee,
                p_f=left_ee + self._approach_left_delta,
                period=APPROACH_TEST_TRAJ_TIME,
            )
            self.tr_right.poly5(
                p_i=right_ee,
                p_f=right_ee + self._approach_right_delta,
                period=APPROACH_TEST_TRAJ_TIME,
            )

            if hasattr(self.ee_task, "set_use_base"):
                self.ee_task.set_use_base(self.approach_use_base)
            else:
                self.ee_task.use_base = self.approach_use_base
            self.ee_task.set_trajectory([self.tr_left, self.tr_right])
            self._tp_approach_traj_initialized = True
            self._tp_last_exec_monotonic = None
            self._tp_cmd_cache = None
            self._tp_cmd_cache_time = None

            self.get_logger().info(
                f"Approach TP trajectory initialized from live EE pose (T={APPROACH_TEST_TRAJ_TIME:.1f}s)"
            )
            return True
        except Exception as exc:
            self._warn_throttled(
                "approach_tp_traj_init_failed",
                f"Unable to initialize TP trajectory from live state: {exc}",
                period_s=2.0,
            )
            return False

    def _log_throttled(self, key: str, msg: str, period_s: float = 2.0, level: str = "warn"):
        now = self.get_clock().now().nanoseconds / 1e9
        last = self._warn_timestamps.get(key, -1e9)
        if (now - last) >= period_s:
            self._warn_timestamps[key] = now
            if str(level).lower() == "info":
                self.get_logger().info(msg)
            else:
                self.get_logger().warn(msg)

    def _warn_throttled(self, key: str, msg: str, period_s: float = 2.0):
        self._log_throttled(key=key, msg=msg, period_s=period_s, level="warn")

    def _info_throttled(self, key: str, msg: str, period_s: float = 2.0):
        self._log_throttled(key=key, msg=msg, period_s=period_s, level="info")

    def _build_expected_joint_map(self):
        """
        Ricava i nomi giunto attesi da TP, mantenendo fallback robusto left/right.
        """
        try:
            arm_joint_groups = self.robot.get_arm_joint_names(skip_fixed=True)
        except Exception as exc:
            self._warn_throttled("joint_map_missing", f"Unable to read arm joint names from TP model: {exc}")
            return

        arm_names = list(getattr(self.robot, "arm_names", []))
        side_map: Dict[str, List[str]] = {"left": [], "right": []}

        for idx, joints in enumerate(arm_joint_groups):
            arm_name = arm_names[idx] if idx < len(arm_names) else f"arm_{idx}"
            lname = arm_name.lower()
            if "left" in lname:
                side_map["left"] = list(joints)
            elif "right" in lname:
                side_map["right"] = list(joints)
            elif not side_map["left"]:
                side_map["left"] = list(joints)
            elif not side_map["right"]:
                side_map["right"] = list(joints)

        self._expected_arm_joint_names = side_map
        self.get_logger().info(
            "TP expected joint mapping - "
            f"left:{len(side_map['left'])} joints, right:{len(side_map['right'])} joints"
        )

    def _joint_states_cb(self, msg: JointState, side: str):
        """Callback per gli stati dei giunti."""
        target_side = side
        try:
            if msg.name:
                inferred = self._infer_side_from_text(" ".join(msg.name))
                if inferred is not None:
                    target_side = inferred
        except Exception:
            pass
        if target_side != side:
            self._warn_throttled(
                f"joint_states_side_remap_{side}_{target_side}",
                f"joint_states remapped from '{side}' callback to '{target_side}' using joint names",
                period_s=2.0,
            )
        self._last_joint_states[target_side] = msg

    def _joint_states_global_cb(self, msg: JointState):
        """Fallback callback quando i joint state arrivano su topic aggregata."""
        self._last_joint_states["global"] = msg

    def get_arm_joint_positions(self, side: str):
        """
        Return list of joint positions or None if no data yet.
        side: 'left' or 'right'
        """
        joint_states = self._last_joint_states.get(side)
        if joint_states is None:
            joint_states = self._last_joint_states.get("global")
            if joint_states is not None:
                self._warn_throttled(
                    f"{side}_joint_global_fallback",
                    f"[{side}] using /joint_states fallback (side topic missing)"
                )
        if joint_states is None:
            return None

        expected_names = self._expected_arm_joint_names.get(side, [])

        if not expected_names:
            if len(joint_states.position) >= 6:
                self._warn_throttled(
                    f"{side}_joint_fallback",
                    f"[{side}] TP joint map unavailable, fallback to first 6 joint_states entries"
                )
                return [float(v) for v in joint_states.position[:6]]
            self._warn_throttled(
                f"{side}_joint_short",
                f"[{side}] no TP joint map and joint_states has only {len(joint_states.position)} values"
            )
            return None

        if len(joint_states.name) != len(joint_states.position):
            self._warn_throttled(
                f"{side}_joint_name_len_mismatch",
                f"[{side}] joint_states name/position length mismatch: {len(joint_states.name)} != {len(joint_states.position)}"
            )
            return None

        name_to_pos = {name: pos for name, pos in zip(joint_states.name, joint_states.position)}
        norm_to_pos = {self._normalize_joint_name(name): pos for name, pos in zip(joint_states.name, joint_states.position)}

        ordered = []
        missing = []
        for exp_name in expected_names:
            if exp_name in name_to_pos:
                ordered.append(float(name_to_pos[exp_name]))
                continue
            norm_name = self._normalize_joint_name(exp_name)
            if norm_name in norm_to_pos:
                ordered.append(float(norm_to_pos[norm_name]))
                continue
            missing.append(exp_name)

        if missing:
            self._warn_throttled(
                f"{side}_joint_missing",
                f"[{side}] missing {len(missing)} expected joints in /{side}/joint_states "
                f"(first missing: {missing[0]})"
            )
            return None

        return ordered
    
    def _odom_cb(self, msg: Odometry, side: str):
        """Callback per l'odomentria delle basi."""
        target_side = side
        try:
            txt = f"{msg.child_frame_id} {msg.header.frame_id}"
            inferred = self._infer_side_from_text(txt)
            if inferred is not None:
                target_side = inferred
        except Exception:
            pass
        if target_side != side:
            self._warn_throttled(
                f"odom_side_remap_{side}_{target_side}",
                f"odom remapped from '{side}' callback to '{target_side}' using frame ids",
                period_s=2.0,
            )
        self._last_odom[target_side] = msg

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

def _cmd_slice(cmd, start: int, end: int, expected_len: int) -> List[float]:
    if cmd is None:
        return [0.0] * expected_len
    try:
        out = [float(v) for v in cmd[start:end]]
    except Exception:
        out = []
    if len(out) < expected_len:
        out.extend([0.0] * (expected_len - len(out)))
    return out[:expected_len]


def _sanitize_arm_cmd(values: List[float], abs_max: float) -> List[float]:
    out = []
    lim = max(float(abs_max), 1e-3)
    for v in values:
        vv = float(v) if np.isfinite(v) else 0.0
        if vv > lim:
            vv = lim
        elif vv < -lim:
            vv = -lim
        out.append(vv)
    return out


def _sanitize_base_cmd(values: List[float], xy_abs_max: float, wz_abs_max: float, ramp: float = 1.0) -> List[float]:
    if len(values) < 3:
        values = list(values) + [0.0] * (3 - len(values))
    r = min(max(float(ramp), 0.0), 1.0)
    lim_xy = max(float(xy_abs_max), 1e-3) * r
    lim_wz = max(float(wz_abs_max), 1e-3) * r

    out = []
    for i, v in enumerate(values[:3]):
        vv = float(v) if np.isfinite(v) else 0.0
        lim = lim_xy if i < 2 else lim_wz
        if vv > lim:
            vv = lim
        elif vv < -lim:
            vv = -lim
        out.append(vv)
    return out


def ApproachObject():
    """
    Avvicinamento in modalità Task Prioritization (test iniziale arm+base):
    - usa TP con traiettoria breve sui due end-effector
    - integra anche la base mobile (vx, vy, wz) con limiti conservativi
    - al termine del test imposta near_object=True e ritorna SUCCESS
    """
    node = _require_node()
    tree_name = get_current_bt_name()
    timer_key = f"{tree_name}_ApproachObject"
    near_key = f"{tree_name}_near_object"

    left_arm_jp = node.get_arm_joint_positions("left") # pose dai giunti da /left/joint_states
    right_arm_jp = node.get_arm_joint_positions("right") # pose dai giunti da /right/joint_states

    left_base = node.get_base_pose("left")   #pose absolute world fixed frame da left_summit_odom
    right_base = node.get_base_pose("right") #pose absolute world fixed frame da right_summit_odom
    
    # Check if all sensor data is available before proceeding
    if None in [left_arm_jp, right_arm_jp, left_base, right_base]:
        node._warn_throttled(f"{tree_name}_approach_wait", bt_fmt("[ApproachObject] Waiting for sensor data (joint_states/odom)"))
        return None  # RUNNING - keep trying until data arrives

    t0 = node.get_action_timer(timer_key)
    if t0 is None:
        t0 = node.start_action_timer(timer_key)
        node.bb[near_key] = False
        node.get_logger().info(
            bt_fmt(
                f"[ApproachObject] start TP test "
                f"(use_base={node.approach_use_base}) "
                f"(dur={node.approach_test_duration:.1f}s)"
            )
        )

    if not node._tp_approach_traj_initialized:
        ok = node._init_tp_approach_trajectory_from_live_state(
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
        )
        if not ok:
            return None  # RUNNING

    # NOTE: execute() richiede i sottosistemi in ordine base+arm per ogni robot.
    # In questo test passiamo sempre odometria reale delle basi; i cmd base vengono poi
    # abilitati/disabilitati da `approach_use_base` prima della pubblicazione.
    joint_pos, base_odom = node._build_tp_inputs_from_side_data(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )

    now_mono = time.monotonic()
    cmd = node._tp_cmd_cache
    can_reuse = (
        cmd is not None
        and node._tp_cmd_cache_time is not None
        and (now_mono - node._tp_cmd_cache_time) < max(node.tp_cmd_min_period, 1e-3)
    )
    if not can_reuse:
        if node._tp_last_exec_monotonic is None:
            tp_dt = 1.0 / 30.0
        else:
            tp_dt = now_mono - node._tp_last_exec_monotonic
        node._tp_last_exec_monotonic = now_mono
        # dt robusto: evita valori troppo piccoli/grandi che destabilizzano il controllo.
        node.tp._delta_t = max(1e-3, min(float(tp_dt), 0.2))
        cmd = node.tp.execute(joint_pos=joint_pos, base_odom=base_odom)
        node._tp_cmd_cache = cmd
        node._tp_cmd_cache_time = now_mono
    if cmd is not None and len(cmd) < 18:
        node._warn_throttled(
            "approach_cmd_short",
            bt_fmt(f"[ApproachObject] TP command vector too short: len={len(cmd)}")
        )

    elapsed = node.get_clock().now().nanoseconds / 1e9 - t0
    ramp = 1.0
    if node.approach_use_base:
        ramp = min(max(elapsed / max(node.tp_base_cmd_ramp_time, 1e-3), 0.0), 1.0)

    if node.approach_use_base:
        left_base_cmd_vals = _sanitize_base_cmd(
            node._get_base_cmd_values(cmd, "left"),
            xy_abs_max=node.tp_base_cmd_xy_abs_max,
            wz_abs_max=node.tp_base_cmd_wz_abs_max,
            ramp=ramp,
        )
        right_base_cmd_vals = _sanitize_base_cmd(
            node._get_base_cmd_values(cmd, "right"),
            xy_abs_max=node.tp_base_cmd_xy_abs_max,
            wz_abs_max=node.tp_base_cmd_wz_abs_max,
            ramp=ramp,
        )
    else:
        left_base_cmd_vals = [0.0, 0.0, 0.0]
        right_base_cmd_vals = [0.0, 0.0, 0.0]

    left_arm_cmd_vals = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "left"), node.tp_arm_cmd_abs_max)
    right_arm_cmd_vals = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "right"), node.tp_arm_cmd_abs_max)
    node._info_throttled(
        f"{tree_name}_approach_cmd",
        bt_fmt(
            "[ApproachObject] cmd max abs "
            f"BL={max(abs(v) for v in left_base_cmd_vals):.4f}, "
            f"BR={max(abs(v) for v in right_base_cmd_vals):.4f}, "
            f"L={max(abs(v) for v in left_arm_cmd_vals):.4f}, "
            f"R={max(abs(v) for v in right_arm_cmd_vals):.4f}, "
            f"dt={node.tp._delta_t:.4f}s, arm_clip={node.tp_arm_cmd_abs_max:.3f}, "
            f"base_clip_xy={node.tp_base_cmd_xy_abs_max:.3f}, base_clip_wz={node.tp_base_cmd_wz_abs_max:.3f}, "
            f"ramp={ramp:.2f}"
        ),
        period_s=1.0,
    )

    tl = Twist()
    tr = Twist()
    tl.linear.x, tl.linear.y, tl.angular.z = left_base_cmd_vals
    tr.linear.x, tr.linear.y, tr.angular.z = right_base_cmd_vals

    la = Float64MultiArray()
    ra = Float64MultiArray()
    la.data = left_arm_cmd_vals
    ra.data = right_arm_cmd_vals

    node.left_base_pub.publish(tl)
    node.right_base_pub.publish(tr)
    node.left_arm_pub.publish(la)
    node.right_arm_pub.publish(ra)

    if elapsed >= node.approach_test_duration:
        node.stop_all_movement()
        node.clear_action_timer(timer_key)
        node.bb[near_key] = True
        node.get_logger().info(bt_fmt(f"[ApproachObject] completed, {near_key}=True"))
        return True

    return None


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
    if node.approach_only_mode:
        node._info_throttled(
            "approach_only_liftobj",
            bt_fmt("[LiftObj] paused (SIMOD_APPROACH_ONLY=1)"),
            period_s=5.0,
        )
        return None
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
