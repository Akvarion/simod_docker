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
import yaml
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates
from linkattacher_msgs.srv import AttachLink, DetachLink
from gazebo_link_gravity_toggle.srv import SetLinkGravity
from gazebo_collision_toggle.srv import SetCollisionEnabled

from ament_index_python.packages import get_package_share_directory

import numpy as np
from bt_xml_demo.bootstrap import ensure_task_prioritization_on_path
from bt_xml_demo.params import DemoConfig, load_demo_config

# Task Prioritization è in /TaskPrioritization (sibling di /ros2_ws nel container)
# Individua la cartella anche da install space colcon e la aggiunge al sys.path
ensure_task_prioritization_on_path(os.path.dirname(os.path.abspath(__file__)))
 
from TaskPrioritization.task_priority import TPManager
from TaskPrioritization.Trajectories.trajectory import Trajectory


def bt_fmt(message: str) -> str:
    """Core-side log formatting fallback (actions module adds BT context)."""
    return str(message)


# =============================================================================
# CONFIG: Durate e profili di movimento (caricati da YAML esterno)
# =============================================================================

DEMO_CFG, BT_DEMO_PARAMS_PATH = load_demo_config()


# =============================================================================
# Nodo ROS2: publisher + client servizi di base
# =============================================================================

from bt_xml_demo.mixins.gazebo_bridge import GazeboBridgeMixin
from bt_xml_demo.mixins.tp_control import TPControlMixin
from bt_xml_demo.mixins.approach_planning import ApproachPlanningMixin

class BTDemoNode(ApproachPlanningMixin, TPControlMixin, GazeboBridgeMixin, Node):
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

    @staticmethod
    def _parse_joint_goal_text(value: str) -> Optional[np.ndarray]:
        txt = str(value).strip()
        if not txt:
            return None
        clean = txt.replace(";", ",").replace(" ", ",")
        parts = [p for p in clean.split(",") if p]
        if len(parts) != 6:
            return None
        try:
            vec = np.asarray([float(p) for p in parts], dtype=np.float32)
        except Exception:
            return None
        if not bool(np.all(np.isfinite(vec))):
            return None
        return vec

    @staticmethod
    def _parse_rpy_text(value: str) -> Optional[np.ndarray]:
        txt = str(value).strip()
        if not txt:
            return None
        clean = txt.replace(";", ",").replace(" ", ",")
        parts = [p for p in clean.split(",") if p]
        if len(parts) != 3:
            return None
        try:
            vec = np.asarray([float(p) for p in parts], dtype=np.float32)
        except Exception:
            return None
        if not bool(np.all(np.isfinite(vec))):
            return None
        return vec


    @staticmethod
    def _load_initial_joints_from_robot_cfg(robot_cfg_path: str) -> Dict[str, Optional[np.ndarray]]:
        out: Dict[str, Optional[np.ndarray]] = {"left": None, "right": None}
        if not robot_cfg_path or not os.path.isfile(robot_cfg_path):
            return out
        try:
            with open(robot_cfg_path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh) or {}
            if not isinstance(data, dict):
                return out
            for key, cfg in data.items():
                if not isinstance(cfg, dict):
                    continue
                initials = cfg.get("initial_joint_positions", None)
                if not isinstance(initials, (list, tuple)) or len(initials) != 6:
                    continue
                try:
                    vec = np.asarray([float(v) for v in initials], dtype=np.float32)
                except Exception:
                    continue
                lkey = str(key).lower()
                if "left" in lkey:
                    out["left"] = vec
                elif "right" in lkey:
                    out["right"] = vec
            return out
        except Exception:
            return out

    def _init_approach_runtime_config(self):
        ap = self.cfg.approach

        # Runtime trajectory/cache state.
        self.approach_duration = float(ap.test_duration)
        self.approach_traj_time = float(ap.test_traj_time)
        self.approach_test_duration = self.approach_duration
        self._approach_left_delta = np.array([0.00, 0.00, 0.04, 0.0, 0.0, 0.0], dtype=np.float32)
        self._approach_right_delta = np.array([0.00, 0.00, 0.04, 0.0, 0.0, 0.0], dtype=np.float32)
        self._tp_approach_traj_initialized = False
        self._tp_last_exec_monotonic: Optional[float] = None
        self._tp_cmd_cache = None
        self._tp_cmd_cache_time: Optional[float] = None
        self._approach_pallet_source_used: str = "unset"
        self._approach_pallet_xy_used: Optional[List[float]] = None

        # Single assignment step for typed YAML defaults.
        runtime_defaults = {
            "tp_cmd_min_period": float(self.cfg.tp.cmd_min_period),
            "approach_estimated_timeout": float(ap.estimated_timeout),
            "tp_arm_cmd_abs_max": float(ap.arm_cmd_abs_max),
            "approach_jtc_base_kp_xy": float(ap.jtc_base_kp_xy),
            "approach_jtc_base_kp_yaw": float(ap.jtc_base_kp_yaw),
            "approach_jtc_arm_kp": float(ap.jtc_arm_kp),
            "approach_use_base": bool(ap.use_base),
            "tp_base_cmd_xy_abs_max": float(ap.base_cmd_xy_abs_max),
            "tp_base_cmd_wz_abs_max": float(ap.base_cmd_wz_abs_max),
            "tp_base_cmd_ramp_time": float(ap.base_cmd_ramp_time),
            "approach_base_goal_tol": float(ap.base_goal_tol),
            "approach_base_kp_x": float(ap.base_kp_x),
            "approach_base_kp_y": float(ap.base_kp_y),
            "approach_base_keep_lane": bool(ap.base_keep_lane),
            "approach_force_full_2d": bool(ap.force_full_2d),
            "approach_base_auto_swap_slots": bool(ap.base_auto_swap_slots),
            "approach_base_swap_hyst": float(ap.base_swap_hyst),
            "approach_base_align_yaw": bool(ap.base_align_yaw),
            "approach_base_left_offset_x": float(ap.base_left_offset_x),
            "approach_base_left_offset_y": float(ap.base_left_offset_y),
            "approach_base_right_offset_x": float(ap.base_right_offset_x),
            "approach_base_right_offset_y": float(ap.base_right_offset_y),
            "approach_arm_profile_scale": float(ap.arm_profile_scale),
            "approach_arm_max_delta": float(ap.arm_max_delta),
            "approach_ee_left_offset_x": float(ap.ee_left_offset_x),
            "approach_ee_left_offset_y": float(ap.ee_left_offset_y),
            "approach_ee_right_offset_x": float(ap.ee_right_offset_x),
            "approach_ee_right_offset_y": float(ap.ee_right_offset_y),
            "approach_ee_offset_z": float(ap.ee_offset_z),
            "mock_pallet_forward": float(ap.mock_pallet_forward),
            "mock_pallet_standoff": float(ap.mock_pallet_standoff),
            "mock_lateral_half": max(0.0, float(ap.mock_lateral_half)),
            "approach_arm_joint_tol": float(ap.arm_joint_tol),
            "approach_arm_ee_pos_tol": float(ap.arm_ee_pos_tol),
            "approach_min_exec_time": float(ap.min_exec_time),
            "approach_arm_delay_enable": bool(ap.arm_delay_enable),
            "approach_arm_enable_dist": max(0.0, float(ap.arm_enable_dist)),
            "approach_arm_home_traj_time": max(0.2, float(ap.arm_home_traj_time)),
            "approach_arm_enable_traj_time": max(0.2, float(ap.arm_enable_traj_time)),
            "approach_goal_cache_enabled": bool(ap.goal_cache_enabled),
            "approach_goal_cache_file": str(ap.goal_cache_file).strip() or ap.goal_cache_file,
            "mock_pallet_x_env": str(ap.mock_pallet_x).strip(),
            "mock_pallet_y_env": str(ap.mock_pallet_y).strip(),
            "mock_pallet_z_env": str(ap.mock_pallet_z).strip(),
        }
        for attr, value in runtime_defaults.items():
            setattr(self, attr, value)

        self.approach_base_ctrl = str(ap.base_ctrl).strip().lower()
        if self.approach_base_ctrl != "tp":
            self.get_logger().warn(
                f"approach.base_ctrl={self.approach_base_ctrl} ignored: Approach is forced to full-TP base control"
            )
            self.approach_base_ctrl = "tp"

        self.approach_base_target_mode = str(ap.base_target_mode).strip().lower()
        if self.approach_base_target_mode not in ("pallet_offset", "legacy"):
            self.get_logger().warn(
                f"Invalid approach.base_target_mode={self.approach_base_target_mode}, fallback to 'pallet_offset'"
            )
            self.approach_base_target_mode = "pallet_offset"

        self.approach_arm_target_mode = str(ap.arm_target_mode).strip().lower()
        if self.approach_arm_target_mode not in ("pallet_offset", "joint_profile"):
            self.get_logger().warn(
                f"Invalid approach.arm_target_mode={self.approach_arm_target_mode}, fallback to 'pallet_offset'"
            )
            self.approach_arm_target_mode = "pallet_offset"

        if self.approach_force_full_2d and self.approach_base_keep_lane:
            self.get_logger().warn(
                "approach.base_keep_lane=true ignored because approach.force_full_2d=true "
                "(approach runs in full 2D toward pallet targets)"
            )
        self.approach_base_keep_lane = bool(self.approach_base_keep_lane and (not self.approach_force_full_2d))

        self.approach_ee_orient_mode = str(ap.ee_orient_mode).strip().lower()
        if self.approach_ee_orient_mode not in ("keep", "fixed"):
            self.get_logger().warn(
                f"Invalid approach.ee_orient_mode={self.approach_ee_orient_mode}, fallback to 'keep'"
            )
            self.approach_ee_orient_mode = "keep"

        left_rpy_raw = str(ap.ee_left_rpy).strip()
        right_rpy_raw = str(ap.ee_right_rpy).strip()
        self.approach_ee_left_rpy_goal = self._parse_rpy_text(left_rpy_raw)
        self.approach_ee_right_rpy_goal = self._parse_rpy_text(right_rpy_raw)
        if left_rpy_raw and self.approach_ee_left_rpy_goal is None:
            self.get_logger().warn(
                "Invalid approach.ee_left_rpy (expected 3 comma-separated floats: roll,pitch,yaw)."
            )
        if right_rpy_raw and self.approach_ee_right_rpy_goal is None:
            self.get_logger().warn(
                "Invalid approach.ee_right_rpy (expected 3 comma-separated floats: roll,pitch,yaw)."
            )

        self._approach_cfg_start_joints = self._load_initial_joints_from_robot_cfg(self.robot_cfg_path)
        left_home_cfg_raw = str(ap.left_arm_home).strip()
        right_home_cfg_raw = str(ap.right_arm_home).strip()
        left_home_cfg = self._parse_joint_goal_text(left_home_cfg_raw)
        right_home_cfg = self._parse_joint_goal_text(right_home_cfg_raw)
        if left_home_cfg_raw and left_home_cfg is None:
            self.get_logger().warn(
                "Invalid approach.left_arm_home (expected 6 comma-separated floats). "
                "Fallback to config initial joints/live state."
            )
        if right_home_cfg_raw and right_home_cfg is None:
            self.get_logger().warn(
                "Invalid approach.right_arm_home (expected 6 comma-separated floats). "
                "Fallback to config initial joints/live state."
            )
        self._approach_home_env_goal = {
            "left": np.asarray(left_home_cfg, dtype=np.float32) if left_home_cfg is not None else None,
            "right": np.asarray(right_home_cfg, dtype=np.float32) if right_home_cfg is not None else None,
        }
        self._approach_home_goal_source = {"left": "unset", "right": "unset"}

        if self.approach_arm_target_mode == "pallet_offset":
            self._approach_arm_goal_joints = {"left": None, "right": None}
            self._approach_arm_goal_source = {"left": "pallet_offset", "right": "pallet_offset"}
        else:
            left_goal_cfg_raw = str(ap.left_arm_goal).strip()
            right_goal_cfg_raw = str(ap.right_arm_goal).strip()
            left_goal_cfg = self._parse_joint_goal_text(left_goal_cfg_raw)
            right_goal_cfg = self._parse_joint_goal_text(right_goal_cfg_raw)
            if left_goal_cfg_raw and left_goal_cfg is None:
                self.get_logger().warn(
                    "Invalid approach.left_arm_goal (expected 6 comma-separated floats). "
                    "Fallback to profile-derived goal."
                )
            if right_goal_cfg_raw and right_goal_cfg is None:
                self.get_logger().warn(
                    "Invalid approach.right_arm_goal (expected 6 comma-separated floats). "
                    "Fallback to profile-derived goal."
                )
            self._approach_arm_goal_joints = {
                "left": np.asarray(left_goal_cfg, dtype=np.float32) if left_goal_cfg is not None else None,
                "right": np.asarray(right_goal_cfg, dtype=np.float32) if right_goal_cfg is not None else None,
            }
            cfg_l = self._approach_cfg_start_joints.get("left", None)
            cfg_r = self._approach_cfg_start_joints.get("right", None)
            if self._approach_arm_goal_joints["left"] is None and cfg_l is not None:
                self._approach_arm_goal_joints["left"] = self._build_profile_goal(
                    np.asarray(cfg_l, dtype=np.float32),
                    self.left_arm_vel_profile,
                )
            if self._approach_arm_goal_joints["right"] is None and cfg_r is not None:
                self._approach_arm_goal_joints["right"] = self._build_profile_goal(
                    np.asarray(cfg_r, dtype=np.float32),
                    self.right_arm_vel_profile,
                )
            self._approach_arm_goal_source = {
                "left": "config" if left_goal_cfg is not None else ("config_profile" if cfg_l is not None else "profile"),
                "right": "config" if right_goal_cfg is not None else ("config_profile" if cfg_r is not None else "profile"),
            }

        self._approach_goal_ee: Dict[str, Optional[np.ndarray]] = {"left": None, "right": None}
        self._approach_base_goal_xy: Dict[str, Optional[List[float]]] = {"left": None, "right": None}
        self._approach_home_goal_joints: Dict[str, Optional[np.ndarray]] = {"left": None, "right": None}
        self._approach_arm_motion_enabled = True
        self._approach_arm_delay_active = False
        self._approach_arm_enable_ts: Optional[float] = None
        self._approach_gate_dist = float("inf")
        self._approach_slot_assignment: Optional[Dict[str, str]] = None
        if self.approach_goal_cache_enabled and self.approach_arm_target_mode != "pallet_offset":
            self._load_approach_goal_cache()

        self.approach_only_mode = bool(ap.approach_only)
        self.approach_only_raw = "1" if self.approach_only_mode else "0"
        if self.approach_only_mode:
            self.get_logger().info(
                f"Approach-only mode active: Lift/Transport/Drop are paused "
                f"(approach.approach_only={self.approach_only_mode})"
            )

        self.get_logger().info(
            "Approach TP config: "
            f"use_base={self.approach_use_base}, "
            f"base_ctrl={self.approach_base_ctrl}, "
            f"task_mode={self.approach_task_mode}, "
            f"full_tp_base_ctrl={self.approach_use_base}, "
            f"dur={self.approach_duration:.2f}s, traj={self.approach_traj_time:.2f}s, "
            f"estimated_timeout={self.approach_estimated_timeout:.2f}s, "
            f"jtc_base_kp_xy={self.approach_jtc_base_kp_xy:.2f}, "
            f"jtc_base_kp_yaw={self.approach_jtc_base_kp_yaw:.2f}, "
            f"jtc_arm_kp={self.approach_jtc_arm_kp:.2f}, "
            f"arm_clip={self.tp_arm_cmd_abs_max:.3f}, "
            f"base_clip_xy={self.tp_base_cmd_xy_abs_max:.3f}, "
            f"base_clip_wz={self.tp_base_cmd_wz_abs_max:.3f}, "
            f"base_ramp={self.tp_base_cmd_ramp_time:.2f}s, "
            f"base_tol={self.approach_base_goal_tol:.3f}m, "
            f"lateral_half={self.mock_lateral_half:.3f}m, "
            f"base_target_mode={self.approach_base_target_mode}, "
            f"arm_target_mode={self.approach_arm_target_mode}, "
            f"arm_delay_enable={self.approach_arm_delay_enable}, "
            f"arm_enable_dist={self.approach_arm_enable_dist:.2f}m, "
            f"arm_home_traj={self.approach_arm_home_traj_time:.2f}s, "
            f"arm_enable_traj={self.approach_arm_enable_traj_time:.2f}s, "
            f"force_full_2d={self.approach_force_full_2d}, "
            f"keep_lane={self.approach_base_keep_lane}, "
            f"auto_swap_slots={self.approach_base_auto_swap_slots}, "
            f"align_yaw={self.approach_base_align_yaw}, "
            f"base_pose_source={self.approach_base_pose_source}, "
            f"goal_cache={self.approach_goal_cache_enabled}, "
            f"arm_profile_scale={self.approach_arm_profile_scale:.2f}, "
            f"arm_max_delta={self.approach_arm_max_delta:.2f}, "
            f"base_offsets=L({self.approach_base_left_offset_x:.2f},{self.approach_base_left_offset_y:.2f})/"
            f"R({self.approach_base_right_offset_x:.2f},{self.approach_base_right_offset_y:.2f}), "
            f"ee_offsets=L({self.approach_ee_left_offset_x:.2f},{self.approach_ee_left_offset_y:.2f},{self.approach_ee_offset_z:.2f})/"
            f"R({self.approach_ee_right_offset_x:.2f},{self.approach_ee_right_offset_y:.2f},{self.approach_ee_offset_z:.2f}), "
            f"ee_orient_mode={self.approach_ee_orient_mode}, "
            f"ee_rpy_goal=L:{None if self.approach_ee_left_rpy_goal is None else np.round(self.approach_ee_left_rpy_goal, 3).tolist()}/"
            f"R:{None if self.approach_ee_right_rpy_goal is None else np.round(self.approach_ee_right_rpy_goal, 3).tolist()}, "
            f"arm_joint_tol={self.approach_arm_joint_tol:.3f}rad, "
            f"arm_ee_tol={self.approach_arm_ee_pos_tol:.3f}m, "
            f"arm_goal_source=L:{self._approach_arm_goal_source['left']}/R:{self._approach_arm_goal_source['right']}, "
            f"arm_home_source=L:{'config' if self._approach_home_env_goal['left'] is not None else 'cfg/live'}/"
            f"R:{'config' if self._approach_home_env_goal['right'] is not None else 'cfg/live'}, "
            f"approach_only={self.approach_only_mode} (raw='{self.approach_only_raw}'), "
            f"pallet_model_candidates={self.approach_mock_pallet_model_candidates}, "
            f"base_model_candidates={self._gazebo_base_model_candidates}"
        )

        if hasattr(self.ee_task, "set_use_base"):
            self.ee_task.set_use_base(self.approach_use_base)
        else:
            self.ee_task.use_base = self.approach_use_base
        self.ee_task.set_trajectory([None, None])

    def __init__(self):
        super().__init__("bt_pick_and_place_demo")
        self.cfg: DemoConfig = DEMO_CFG
        ap = self.cfg.approach
        mp = self.cfg.motion_profiles
        tp_cfg = self.cfg.tp

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
        self.approach_mock_world_file = str(ap.mock_world_file or "")
        self.left_arm_vel_profile = [float(v) for v in mp.left_arm_vel]
        self.right_arm_vel_profile = [float(v) for v in mp.right_arm_vel]

        self.approach_base_pose_source = str(ap.base_pose_source).strip().lower()
        if self.approach_base_pose_source not in ("odom", "gazebo", "aligned_odom", "auto"):
            self.get_logger().warn(
                f"Invalid approach.base_pose_source={self.approach_base_pose_source}, fallback to 'aligned_odom'"
            )
            self.approach_base_pose_source = "aligned_odom"

        model_candidates_raw = str(ap.mock_pallet_model or "")
        self.approach_mock_pallet_model_candidates = self._split_model_candidates(model_candidates_raw)
        if not self.approach_mock_pallet_model_candidates:
            self.approach_mock_pallet_model_candidates = [str(ap.mock_pallet_model)]

        left_base_models_raw = str(ap.left_base_model or "")
        right_base_models_raw = str(ap.right_base_model or "")
        self._gazebo_base_model_candidates = {
            "left": self._split_model_candidates(left_base_models_raw),
            "right": self._split_model_candidates(right_base_models_raw),
        }
        if not self._gazebo_base_model_candidates["left"]:
            self._gazebo_base_model_candidates["left"] = [str(ap.left_base_model)]
        if not self._gazebo_base_model_candidates["right"]:
            self._gazebo_base_model_candidates["right"] = [str(ap.right_base_model)]

        self._gazebo_base_pose_start: Dict[str, Optional[List[float]]] = {"left": None, "right": None}
        self._gazebo_base_pose_last: Dict[str, Optional[List[float]]] = {"left": None, "right": None}
        self._gazebo_base_pose_start_model: Dict[str, Optional[str]] = {"left": None, "right": None}
        self._gazebo_base_pose_last_model: Dict[str, Optional[str]] = {"left": None, "right": None}
        self._base_world_calib: Dict[str, Optional[Dict[str, float]]] = {"left": None, "right": None}

        self._gazebo_pallet_pose_start_xy: Optional[List[float]] = None
        self._gazebo_pallet_pose_last_xy: Optional[List[float]] = None
        self._gazebo_pallet_pose_start_xyz: Optional[List[float]] = None
        self._gazebo_pallet_pose_last_xyz: Optional[List[float]] = None
        self._gazebo_pallet_pose_start_model: Optional[str] = None
        self._gazebo_ee_pose_last: Dict[str, Optional[List[float]]] = {"left": None, "right": None}
        self._gazebo_ee_link_name: Dict[str, Optional[str]] = {"left": None, "right": None}
        self._world_pallet_pose_checked = False
        self._world_pallet_pose_cache: Optional[List[float]] = None
        self._world_pallet_pose_model: Optional[str] = None
        self._world_pallet_pose_file: Optional[str] = None

        # subscribe (choose actual topic names in your system)
        self.create_subscription(Odometry, '/left_summit_odom', lambda msg: self._odom_cb(msg, 'left'), 10)
        self.create_subscription(Odometry, '/right_summit_odom', lambda msg: self._odom_cb(msg, 'right'), 10)
        self.create_subscription(JointState, '/left/joint_states', lambda msg: self._joint_states_cb(msg, 'left'), 10)
        self.create_subscription(JointState, '/right/joint_states', lambda msg: self._joint_states_cb(msg, 'right'), 10)
        # Fallback: in alcune configurazioni ROS2 c'è un solo /joint_states aggregato.
        self.create_subscription(JointState, '/joint_states', self._joint_states_global_cb, 10)
        # Gazebo model states: snapshot iniziale della posa del pallet/pacco.
        self.create_subscription(ModelStates, '/model_states', self._model_states_cb, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self._model_states_cb, 10)
        self.create_subscription(ModelStates, '/state/model_states', self._model_states_cb, 10)
        # Gazebo link states: posa effettiva EE nel simulatore fisico (debug/allineamento viewer-vs-gazebo).
        self.create_subscription(LinkStates, '/link_states', self._link_states_cb, 10)
        self.create_subscription(LinkStates, '/gazebo/link_states', self._link_states_cb, 10)

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
        if BT_DEMO_PARAMS_PATH:
            self.get_logger().info(f"BT demo params loaded from: {BT_DEMO_PARAMS_PATH}")
        else:
            self.get_logger().warn(
                "BT demo params file not found, using built-in defaults "
                "(expected config/bt_demo_params.yaml)"
            )

        robot_cfg_hint = str(tp_cfg.robot_cfg_path or "").strip()
        if robot_cfg_hint and not os.path.isfile(robot_cfg_hint):
            self.get_logger().warn(f"tp.robot_cfg_path not found: {robot_cfg_hint}")
        robot_cfg = self._pick_existing_file([
            robot_cfg_hint,
            "/ros2_ws/src/ps_try/config/dual_paletta.yaml",
            os.path.join(pkg_share, "config", "dual_paletta.yaml") if pkg_share else "",
            "/ros2_ws/TaskPrioritization/default_robots/__paletta__/dual_paletta.yaml",
            "/ros2_ws/src/TaskPrioritization/default_robots/__paletta__/dual_paletta.yaml",
            "/TaskPrioritization/default_robots/__paletta__/dual_paletta.yaml",
        ])
        if robot_cfg is None:
            raise FileNotFoundError("dual_paletta.yaml non trovato (ps_try/config o default_robots)")
        self.robot_cfg_path = robot_cfg

        ee_cfg_hint = str(tp_cfg.ee_cfg_path or "").strip()
        if ee_cfg_hint and not os.path.isfile(ee_cfg_hint):
            self.get_logger().warn(f"tp.ee_cfg_path not found: {ee_cfg_hint}")
        ee_cfg = self._pick_existing_file([
            ee_cfg_hint,
            "/ros2_ws/src/ps_try/config/ee_task.yaml",
            os.path.join(pkg_share, "config", "ee_task.yaml") if pkg_share else "",
        ])
        if ee_cfg is None:
            raise FileNotFoundError("ee_task.yaml non trovato (ps_try/config)")

        self.tp = TPManager(device='cuda', dtype='float32') # change device='cpu' to 'cuda' if GPU is available
        self.tp.add_robot(robot_name='__paletta__', config_file=robot_cfg)
        # Viewer TP (socket PyVistaQt). In Docker avvialo nello stesso container per avere path mesh validi.
        enable_tp_viewer = bool(tp_cfg.viewer.enable)
        if enable_tp_viewer:
            try:
                viewer_host = str(tp_cfg.viewer.host)
                viewer_port = int(tp_cfg.viewer.port)
                viewer_hz = float(tp_cfg.viewer.hz)
                launch_viewer = bool(tp_cfg.viewer.autostart)
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
            self.get_logger().info("TP viewer disabled by config (tp.viewer.enable=false)")
        self.ee_task = self.tp.add_EE_task(ee_cfg)
        self.approach_task_mode = str(ap.task_mode).strip().lower()
        if self.approach_task_mode not in ("joint", "ee", "hybrid"):
            self.get_logger().warn(
                f"Invalid approach.task_mode={self.approach_task_mode}, fallback to 'hybrid'"
            )
            self.approach_task_mode = "hybrid"
        self.approach_jtc_task = self.tp.add_task("JointTrajectoryControl", priority=2)
        try:
            self.approach_jtc_task.deactivate()
        except Exception:
            pass
        # Disattiva eventuali piani di traiettoria pre-caricati da YAML, così i valori di test non vengono sovrascritti al primo cycle.
        if hasattr(self.ee_task, "set_trajectory_plan"):
            self.ee_task.set_trajectory_plan(None)
        # initialize robot kinematics before using it
        self.robot = self.tp.get_robot_kinematics()
        self.tr_left: Optional[Trajectory] = None
        self.tr_right: Optional[Trajectory] = None

        self._init_approach_runtime_config()

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
        self.bb.setdefault("SRM1_near_object", False)
        self.bb.setdefault("SRM2_near_object", False)
        self.bb.setdefault("pallet_pose_world", None)
        self.bb.setdefault("pallet_pose_world_xyz", None)
        self.bb.setdefault("pallet_pose_source", "unset")
        self.bb.setdefault("pallet_info_ready", False)
        self.bb.setdefault("srm1_data_to_srm2", False)
        self.bb.setdefault("approach_lateral_half", float(self.mock_lateral_half))
        self.bb.setdefault("SRM1_base_goal_reached", False)
        self.bb.setdefault("SRM2_base_goal_reached", False)

        # Timer per azioni lunga durata: dict azione -> start_time
        self.action_timers = {}

        # Stato interno per azioni multi-fase (es. LiftObj)
        self.lift_phase = None

        self.get_logger().info(f"BTDemoNode inizializzato - robot_cfg={robot_cfg}, ee_cfg={ee_cfg}")

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

    def stop_side_movement(self, side: str):
        """Stop immediato per un solo lato."""
        side = str(side).lower()
        zero_twist = Twist()
        zero_arm = Float64MultiArray()
        zero_arm.data = [0.0] * 6
        if side == "left":
            self.left_base_pub.publish(zero_twist)
            self.left_arm_pub.publish(zero_arm)
        elif side == "right":
            self.right_base_pub.publish(zero_twist)
            self.right_arm_pub.publish(zero_arm)
        else:
            self.stop_all_movement()

    def side_from_bt_name(self, bt_name: str) -> Optional[str]:
        name = str(bt_name).strip().upper()
        if name == "SRM1":
            return "left"
        if name == "SRM2":
            return "right"
        return None


def set_package_gravity(node: BTDemoNode, gravity_on: bool) -> bool:
    client = node.toggle_gravity_cli
    package_link_name = str(node.cfg.package.link_name)
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
    req.link_name = package_link_name
    req.gravity = gravity_on

    future = client.call_async(req)
    try:
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    except Exception:
        pass

    success = bool(future.done() and future.result() and getattr(future.result(), "success", True))
    if success:
        node.get_logger().info(bt_fmt(f"[Gravity] set gravity={gravity_on} on {package_link_name}"))
    else:
        node.get_logger().warn(bt_fmt(f"[Gravity] failed to set gravity={gravity_on} on {package_link_name}"))
    return success
