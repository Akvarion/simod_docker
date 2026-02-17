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

import json
import os
import sys
import time
import contextvars
import yaml
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from behaviortree.tree import Tree
from xml.etree.ElementTree import parse

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
# Approach "reale" (non più micro-test)
APPROACH_TEST_DURATION = APPROACH_TIME
APPROACH_TEST_TRAJ_TIME = APPROACH_TIME
APPROACH_ARM_CMD_ABS_MAX = 0.25
APPROACH_BASE_CMD_XY_ABS_MAX = 0.12
APPROACH_BASE_CMD_WZ_ABS_MAX = 0.20
APPROACH_BASE_CMD_RAMP_TIME = 1.0
APPROACH_BASE_GOAL_TOL = 0.08
APPROACH_BASE_KP_X = 0.9
APPROACH_BASE_KP_Y = 0.9
APPROACH_MOCK_PALLET_FORWARD = 1.20
APPROACH_MOCK_PALLET_STANDOFF = 0.85
APPROACH_MOCK_LATERAL_HALF = 0.35
APPROACH_BASE_LEFT_OFFSET_X = -0.50
APPROACH_BASE_LEFT_OFFSET_Y = -0.50
APPROACH_BASE_RIGHT_OFFSET_X = 0.50
APPROACH_BASE_RIGHT_OFFSET_Y = -0.50
APPROACH_EE_LEFT_OFFSET_X = -0.35
APPROACH_EE_LEFT_OFFSET_Y = -0.10
APPROACH_EE_RIGHT_OFFSET_X = 0.35
APPROACH_EE_RIGHT_OFFSET_Y = -0.10
APPROACH_EE_OFFSET_Z = 0.40
APPROACH_MOCK_PALLET_MODEL = "pacco_clone_1,pacco,pallet2"
APPROACH_MOCK_WORLD_FILE = ""
APPROACH_LEFT_BASE_MODEL = "left_srm,left_summit_xls"
APPROACH_RIGHT_BASE_MODEL = "right_srm,right_summit_xls"
APPROACH_BASE_POSE_SOURCE = "aligned_odom"
APPROACH_BASE_KEEP_LANE = False
APPROACH_FORCE_FULL_2D = True
APPROACH_BASE_AUTO_SWAP_SLOTS = True
APPROACH_BASE_SWAP_HYST = 0.10
APPROACH_BASE_ALIGN_YAW = False
APPROACH_ARM_PROFILE_SCALE = 0.18
APPROACH_ARM_MAX_DELTA = 0.60
APPROACH_BASE_TARGET_MODE = "pallet_offset"
APPROACH_ARM_TARGET_MODE = "pallet_offset"
APPROACH_ARM_JOINT_TOL = 0.10
APPROACH_ARM_EE_POS_TOL = 0.03
APPROACH_MIN_EXEC_TIME = 2.0
APPROACH_GOAL_CACHE_FILE = "/tmp/simod_approach_goal_joints.json"




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

    def _build_profile_goal(self, q_ref: np.ndarray, vel_ref: List[float]) -> np.ndarray:
        q0 = np.asarray(q_ref, dtype=np.float32)
        vel = np.asarray(vel_ref, dtype=np.float32)
        delta = vel * float(self.approach_duration) * float(self.approach_arm_profile_scale)
        max_d = float(self.approach_arm_max_delta)
        if max_d > 0.0:
            delta = np.clip(delta, -max_d, max_d)
        return q0 + delta

    def _load_approach_goal_cache(self) -> bool:
        path = str(self.approach_goal_cache_file or "").strip()
        if not path or not os.path.isfile(path):
            return False
        try:
            with open(path, "r", encoding="utf-8") as fh:
                payload = json.load(fh)
            loaded_any = False
            for side in ("left", "right"):
                if self._approach_arm_goal_joints.get(side) is not None:
                    continue
                raw = payload.get(side, None) if isinstance(payload, dict) else None
                if not isinstance(raw, (list, tuple)) or len(raw) != 6:
                    continue
                vec = np.asarray(raw, dtype=np.float32)
                if not bool(np.all(np.isfinite(vec))):
                    continue
                self._approach_arm_goal_joints[side] = vec
                self._approach_arm_goal_source[side] = "cache"
                loaded_any = True
            if loaded_any:
                self.get_logger().info(f"Loaded approach arm goals from cache: {path}")
            return loaded_any
        except Exception as exc:
            self._warn_throttled(
                "approach_goal_cache_load",
                f"Unable to load approach goal cache '{path}': {exc}",
                period_s=5.0,
            )
            return False

    def _save_approach_goal_cache(self):
        if not bool(self.approach_goal_cache_enabled):
            return
        path = str(self.approach_goal_cache_file or "").strip()
        if not path:
            return
        left = self._approach_arm_goal_joints.get("left", None)
        right = self._approach_arm_goal_joints.get("right", None)
        if left is None or right is None:
            return
        try:
            folder = os.path.dirname(path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            payload = {
                "left": [float(v) for v in np.asarray(left, dtype=np.float32).tolist()],
                "right": [float(v) for v in np.asarray(right, dtype=np.float32).tolist()],
            }
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, indent=2)
        except Exception as exc:
            self._warn_throttled(
                "approach_goal_cache_save",
                f"Unable to save approach goal cache '{path}': {exc}",
                period_s=5.0,
            )

    @staticmethod
    def _split_model_candidates(raw: str) -> List[str]:
        txt = str(raw or "").replace(";", ",")
        return [p.strip() for p in txt.split(",") if p and p.strip()]

    @staticmethod
    def _pick_model_index(model_names: List[str], raw_candidates: List[str]) -> Optional[int]:
        if not model_names:
            return None
        candidates = [str(c).lower() for c in (raw_candidates or []) if str(c).strip()]
        if not candidates:
            return None
        lowered = [str(n).lower() for n in model_names]

        # 1) match esatto
        for cand in candidates:
            for i, name in enumerate(lowered):
                if name == cand:
                    return i

        # 2) match prefisso/suffisso
        for cand in candidates:
            for i, name in enumerate(lowered):
                if name.startswith(cand) or name.endswith(cand):
                    return i

        # 3) contains
        for cand in candidates:
            for i, name in enumerate(lowered):
                if cand in name:
                    return i

        return None

    def _pick_pallet_model_index(self, model_names: List[str]) -> Optional[int]:
        return self._pick_model_index(model_names, self.approach_mock_pallet_model_candidates)

    def _pick_base_model_index(self, model_names: List[str], side: str) -> Optional[int]:
        side = str(side).lower()
        idx = self._pick_model_index(model_names, self._gazebo_base_model_candidates.get(side, []))
        if idx is not None:
            return idx
        lowered = [str(n).lower() for n in model_names]
        for i, name in enumerate(lowered):
            if side in name and ("srm" in name or "summit" in name):
                return i
        for i, name in enumerate(lowered):
            if side in name:
                return i
        return None

    @staticmethod
    def _pick_side_ee_link_index(link_names: List[str], side: str) -> Optional[int]:
        side = str(side).lower()
        token = "ur_left_paletta" if side == "left" else "ur_right_paletta"
        if not link_names:
            return None
        lowered = [str(n).lower() for n in link_names]

        # 1) exact suffix match (model::link)
        for i, name in enumerate(lowered):
            if name.endswith(f"::{token}") or name == token:
                return i
        # 2) contains fallback
        for i, name in enumerate(lowered):
            if token in name:
                return i
        return None

    @staticmethod
    def _pose6_from_gazebo_pose_msg(pose_msg) -> List[float]:
        p = pose_msg.position
        q = pose_msg.orientation
        roll, pitch, yaw = BTDemoNode._quat_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))
        return [float(p.x), float(p.y), float(p.z), float(roll), float(pitch), float(yaw)]

    def _model_states_cb(self, msg: ModelStates):
        try:
            names = list(getattr(msg, "name", []) or [])
            poses = list(getattr(msg, "pose", []) or [])
            if not names or len(names) != len(poses):
                return

            for side in ("left", "right"):
                bidx = self._pick_base_model_index(names, side)
                if bidx is None or bidx < 0 or bidx >= len(poses):
                    continue
                pose6 = self._pose6_from_gazebo_pose_msg(poses[bidx])
                self._gazebo_base_pose_last[side] = list(pose6)
                self._gazebo_base_pose_last_model[side] = str(names[bidx])
                if self._gazebo_base_pose_start[side] is None:
                    self._gazebo_base_pose_start[side] = list(pose6)
                    self._gazebo_base_pose_start_model[side] = str(names[bidx])
                    self.get_logger().info(
                        f"[BaseFrame] startup {side} model pose from Gazebo "
                        f"('{self._gazebo_base_pose_start_model[side]}'): "
                        f"x={pose6[0]:.3f}, y={pose6[1]:.3f}, yaw={pose6[5]:.3f}"
                    )
                self._try_init_base_world_calibration(side)

            idx = self._pick_pallet_model_index(names)
            if idx is None or idx < 0 or idx >= len(poses):
                return
            p = poses[idx].position
            xy = [float(p.x), float(p.y)]
            xyz = [float(p.x), float(p.y), float(p.z)]
            self._gazebo_pallet_pose_last_xy = xy
            self._gazebo_pallet_pose_last_xyz = xyz
            if self._gazebo_pallet_pose_start_xy is None:
                self._gazebo_pallet_pose_start_xy = list(xy)
                self._gazebo_pallet_pose_start_xyz = list(xyz)
                self._gazebo_pallet_pose_start_model = str(names[idx])
                self.get_logger().info(
                    f"[VisionMock] startup pallet pose from Gazebo model '{self._gazebo_pallet_pose_start_model}': "
                    f"x={xyz[0]:.3f}, y={xyz[1]:.3f}, z={xyz[2]:.3f}"
                )
        except Exception as exc:
            self._warn_throttled("model_states_cb", f"[VisionMock] model_states callback error: {exc}", period_s=5.0)

    def _link_states_cb(self, msg: LinkStates):
        try:
            names = list(getattr(msg, "name", []) or [])
            poses = list(getattr(msg, "pose", []) or [])
            if not names or len(names) != len(poses):
                return

            for side in ("left", "right"):
                idx = self._pick_side_ee_link_index(names, side)
                if idx is None or idx < 0 or idx >= len(poses):
                    continue
                pose6 = self._pose6_from_gazebo_pose_msg(poses[idx])
                self._gazebo_ee_pose_last[side] = list(pose6)
                link_name = str(names[idx])
                if self._gazebo_ee_link_name.get(side) != link_name:
                    self._gazebo_ee_link_name[side] = link_name
                    self.get_logger().info(
                        f"[EEFrame] tracking Gazebo EE link for {side}: '{link_name}' "
                        f"(x={pose6[0]:.3f}, y={pose6[1]:.3f}, z={pose6[2]:.3f})"
                    )
        except Exception as exc:
            self._warn_throttled("link_states_cb", f"[EEFrame] link_states callback error: {exc}", period_s=5.0)

    @staticmethod
    def _parse_pose_xy_text(pose_text: str) -> Optional[List[float]]:
        txt = str(pose_text or "").strip()
        if not txt:
            return None
        parts = txt.split()
        if len(parts) < 2:
            return None
        try:
            x = float(parts[0])
            y = float(parts[1])
            return [x, y]
        except Exception:
            return None

    @staticmethod
    def _parse_pose_xyz_text(pose_text: str) -> Optional[List[float]]:
        txt = str(pose_text or "").strip()
        if not txt:
            return None
        parts = txt.split()
        if len(parts) < 3:
            return None
        try:
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            return [x, y, z]
        except Exception:
            return None

    def _resolve_world_file_candidates(self) -> List[str]:
        here = os.path.abspath(os.path.dirname(__file__))
        env_path = os.getenv("SIMOD_APPROACH_WORLD_FILE", APPROACH_MOCK_WORLD_FILE).strip()
        cands = [
            env_path,
            "/ros2_ws/src/simod_docker/gazebo_world/usecase.world",
            "/ros2_ws/simod_docker/gazebo_world/usecase.world",
            "/simod_docker/gazebo_world/usecase.world",
            os.path.abspath(os.path.join(here, "..", "..", "..", "..", "..", "simod_docker", "gazebo_world", "usecase.world")),
            os.path.abspath(os.path.join(here, "..", "..", "..", "..", "gazebo_world", "usecase.world")),
        ]
        out = []
        seen = set()
        for c in cands:
            if not c:
                continue
            cc = os.path.abspath(c)
            if cc in seen:
                continue
            seen.add(cc)
            out.append(cc)
        return out

    def _load_pallet_pose_from_world_file(self) -> Optional[List[float]]:
        import xml.etree.ElementTree as ET

        if self._world_pallet_pose_cache is not None:
            return list(self._world_pallet_pose_cache)
        if self._world_pallet_pose_checked:
            return None
        self._world_pallet_pose_checked = True

        name_cands = [c.lower() for c in self.approach_mock_pallet_model_candidates if c]
        if not name_cands:
            name_cands = [APPROACH_MOCK_PALLET_MODEL]

        for world_path in self._resolve_world_file_candidates():
            if not os.path.isfile(world_path):
                continue
            try:
                root = ET.parse(world_path).getroot()
            except Exception as exc:
                self._warn_throttled(
                    "world_parse_err",
                    f"[VisionMock] unable to parse world file '{world_path}': {exc}",
                    period_s=5.0,
                )
                continue

            # Prefer pose in <state> as startup snapshot.
            for model in root.findall(".//state/model"):
                mname = str(model.attrib.get("name", "")).strip()
                lname = mname.lower()
                if not any((lname == c or lname.startswith(c) or lname.endswith(c) or c in lname) for c in name_cands):
                    continue
                pose_el = model.find("pose")
                xy = self._parse_pose_xy_text(pose_el.text if pose_el is not None else "")
                if xy is not None:
                    self._world_pallet_pose_cache = list(xy)
                    self._world_pallet_pose_model = mname
                    self._world_pallet_pose_file = world_path
                    self.get_logger().info(
                        f"[VisionMock] world-file pallet pose from state '{mname}' in '{world_path}': "
                        f"x={xy[0]:.3f}, y={xy[1]:.3f}"
                    )
                    return list(xy)

            # Fallback: generic model pose.
            for model in root.findall(".//model"):
                mname = str(model.attrib.get("name", "")).strip()
                lname = mname.lower()
                if not any((lname == c or lname.startswith(c) or lname.endswith(c) or c in lname) for c in name_cands):
                    continue
                pose_el = model.find("pose")
                xy = self._parse_pose_xy_text(pose_el.text if pose_el is not None else "")
                if xy is not None:
                    self._world_pallet_pose_cache = list(xy)
                    self._world_pallet_pose_model = mname
                    self._world_pallet_pose_file = world_path
                    self.get_logger().info(
                        f"[VisionMock] world-file pallet pose from model '{mname}' in '{world_path}': "
                        f"x={xy[0]:.3f}, y={xy[1]:.3f}"
                    )
                    return list(xy)

        return None

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

        self.approach_base_pose_source = os.getenv("SIMOD_APPROACH_BASE_POSE_SOURCE", APPROACH_BASE_POSE_SOURCE).strip().lower()
        if self.approach_base_pose_source not in ("odom", "gazebo", "aligned_odom", "auto"):
            self.get_logger().warn(
                f"Invalid SIMOD_APPROACH_BASE_POSE_SOURCE={self.approach_base_pose_source}, fallback to 'aligned_odom'"
            )
            self.approach_base_pose_source = "aligned_odom"

        model_candidates_raw = os.getenv("SIMOD_APPROACH_PALLET_MODEL", APPROACH_MOCK_PALLET_MODEL)
        self.approach_mock_pallet_model_candidates = self._split_model_candidates(model_candidates_raw)
        if not self.approach_mock_pallet_model_candidates:
            self.approach_mock_pallet_model_candidates = [APPROACH_MOCK_PALLET_MODEL]

        left_base_models_raw = os.getenv("SIMOD_APPROACH_LEFT_BASE_MODEL", APPROACH_LEFT_BASE_MODEL)
        right_base_models_raw = os.getenv("SIMOD_APPROACH_RIGHT_BASE_MODEL", APPROACH_RIGHT_BASE_MODEL)
        self._gazebo_base_model_candidates = {
            "left": self._split_model_candidates(left_base_models_raw),
            "right": self._split_model_candidates(right_base_models_raw),
        }
        if not self._gazebo_base_model_candidates["left"]:
            self._gazebo_base_model_candidates["left"] = [APPROACH_LEFT_BASE_MODEL]
        if not self._gazebo_base_model_candidates["right"]:
            self._gazebo_base_model_candidates["right"] = [APPROACH_RIGHT_BASE_MODEL]

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
        self.robot_cfg_path = robot_cfg

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
        self.approach_task_mode = os.getenv("SIMOD_APPROACH_TASK_MODE", "hybrid").strip().lower()
        if self.approach_task_mode not in ("joint", "ee", "hybrid"):
            self.get_logger().warn(
                f"Invalid SIMOD_APPROACH_TASK_MODE={self.approach_task_mode}, fallback to 'hybrid'"
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

        # Approach setup (traj inizializzata al primo tick con stato reale + target da profilo storico).
        self.approach_duration = float(os.getenv("SIMOD_APPROACH_DURATION", str(APPROACH_TEST_DURATION)))
        self.approach_traj_time = float(os.getenv("SIMOD_APPROACH_TRAJ_TIME", str(APPROACH_TEST_TRAJ_TIME)))
        self.approach_test_duration = self.approach_duration
        # Delta fallback usata solo se non è possibile ricavare un target sensato dal profilo giunti storico.
        self._approach_left_delta = np.array([0.00, 0.00, 0.04, 0.0, 0.0, 0.0], dtype=np.float32)
        self._approach_right_delta = np.array([0.00, 0.00, 0.04, 0.0, 0.0, 0.0], dtype=np.float32)
        self._tp_approach_traj_initialized = False
        self._tp_last_exec_monotonic: Optional[float] = None
        self._tp_cmd_cache = None
        self._tp_cmd_cache_time: Optional[float] = None
        self._approach_pallet_source_used: str = "unset"
        self._approach_pallet_xy_used: Optional[List[float]] = None
        self.tp_cmd_min_period = float(os.getenv("SIMOD_TP_CMD_MIN_PERIOD", "0.03"))
        self.approach_estimated_timeout = float(os.getenv("SIMOD_APPROACH_ESTIMATED_TIMEOUT", "2.0"))
        self.tp_arm_cmd_abs_max = float(os.getenv("SIMOD_TP_ARM_CMD_MAX", str(APPROACH_ARM_CMD_ABS_MAX)))
        self.approach_jtc_base_kp_xy = float(os.getenv("SIMOD_APPROACH_JTC_BASE_KP_XY", "1.2"))
        self.approach_jtc_base_kp_yaw = float(os.getenv("SIMOD_APPROACH_JTC_BASE_KP_YAW", "1.0"))
        self.approach_jtc_arm_kp = float(os.getenv("SIMOD_APPROACH_JTC_ARM_KP", "1.3"))
        self.approach_use_base = os.getenv("SIMOD_APPROACH_USE_BASE", "1").strip().lower() in ("1", "true", "yes", "on")
        self.tp_base_cmd_xy_abs_max = float(os.getenv("SIMOD_TP_BASE_CMD_XY_MAX", str(APPROACH_BASE_CMD_XY_ABS_MAX)))
        self.tp_base_cmd_wz_abs_max = float(os.getenv("SIMOD_TP_BASE_CMD_WZ_MAX", str(APPROACH_BASE_CMD_WZ_ABS_MAX)))
        self.tp_base_cmd_ramp_time = float(os.getenv("SIMOD_TP_BASE_CMD_RAMP_TIME", str(APPROACH_BASE_CMD_RAMP_TIME)))
        self.approach_base_ctrl = os.getenv("SIMOD_APPROACH_BASE_CTRL", "tp").strip().lower()
        if self.approach_base_ctrl != "tp":
            self.get_logger().warn(
                f"SIMOD_APPROACH_BASE_CTRL={self.approach_base_ctrl} ignored: Approach is forced to full-TP base control"
            )
            self.approach_base_ctrl = "tp"
        self.approach_base_goal_tol = float(os.getenv("SIMOD_APPROACH_BASE_GOAL_TOL", str(APPROACH_BASE_GOAL_TOL)))
        self.approach_base_kp_x = float(os.getenv("SIMOD_APPROACH_BASE_KP_X", str(APPROACH_BASE_KP_X)))
        self.approach_base_kp_y = float(os.getenv("SIMOD_APPROACH_BASE_KP_Y", str(APPROACH_BASE_KP_Y)))
        self.approach_base_target_mode = os.getenv("SIMOD_APPROACH_BASE_TARGET_MODE", APPROACH_BASE_TARGET_MODE).strip().lower()
        if self.approach_base_target_mode not in ("pallet_offset", "legacy"):
            self.get_logger().warn(
                f"Invalid SIMOD_APPROACH_BASE_TARGET_MODE={self.approach_base_target_mode}, fallback to 'pallet_offset'"
            )
            self.approach_base_target_mode = "pallet_offset"
        self.approach_arm_target_mode = os.getenv("SIMOD_APPROACH_ARM_TARGET_MODE", APPROACH_ARM_TARGET_MODE).strip().lower()
        if self.approach_arm_target_mode not in ("pallet_offset", "joint_profile"):
            self.get_logger().warn(
                f"Invalid SIMOD_APPROACH_ARM_TARGET_MODE={self.approach_arm_target_mode}, fallback to 'pallet_offset'"
            )
            self.approach_arm_target_mode = "pallet_offset"
        self.approach_base_keep_lane = os.getenv("SIMOD_APPROACH_BASE_KEEP_LANE", str(int(APPROACH_BASE_KEEP_LANE))).strip().lower() in ("1", "true", "yes", "on")
        self.approach_force_full_2d = os.getenv("SIMOD_APPROACH_FORCE_FULL_2D", str(int(APPROACH_FORCE_FULL_2D))).strip().lower() in ("1", "true", "yes", "on")
        if self.approach_force_full_2d and self.approach_base_keep_lane:
            self.get_logger().warn(
                "SIMOD_APPROACH_BASE_KEEP_LANE=1 ignored because SIMOD_APPROACH_FORCE_FULL_2D=1 "
                "(approach runs in full 2D toward pallet targets)"
            )
        self.approach_base_keep_lane = bool(self.approach_base_keep_lane and (not self.approach_force_full_2d))
        self.approach_base_auto_swap_slots = os.getenv("SIMOD_APPROACH_BASE_AUTO_SWAP_SLOTS", str(int(APPROACH_BASE_AUTO_SWAP_SLOTS))).strip().lower() in ("1", "true", "yes", "on")
        self.approach_base_swap_hyst = float(os.getenv("SIMOD_APPROACH_BASE_SWAP_HYST", str(APPROACH_BASE_SWAP_HYST)))
        self.approach_base_align_yaw = os.getenv("SIMOD_APPROACH_BASE_ALIGN_YAW", str(int(APPROACH_BASE_ALIGN_YAW))).strip().lower() in ("1", "true", "yes", "on")
        self.approach_base_left_offset_x = float(os.getenv("SIMOD_APPROACH_BASE_LEFT_OFFSET_X", str(APPROACH_BASE_LEFT_OFFSET_X)))
        self.approach_base_left_offset_y = float(os.getenv("SIMOD_APPROACH_BASE_LEFT_OFFSET_Y", str(APPROACH_BASE_LEFT_OFFSET_Y)))
        self.approach_base_right_offset_x = float(os.getenv("SIMOD_APPROACH_BASE_RIGHT_OFFSET_X", str(APPROACH_BASE_RIGHT_OFFSET_X)))
        self.approach_base_right_offset_y = float(os.getenv("SIMOD_APPROACH_BASE_RIGHT_OFFSET_Y", str(APPROACH_BASE_RIGHT_OFFSET_Y)))
        self.approach_arm_profile_scale = float(os.getenv("SIMOD_APPROACH_ARM_PROFILE_SCALE", str(APPROACH_ARM_PROFILE_SCALE)))
        self.approach_arm_max_delta = float(os.getenv("SIMOD_APPROACH_ARM_MAX_DELTA", str(APPROACH_ARM_MAX_DELTA)))
        self.approach_ee_left_offset_x = float(os.getenv("SIMOD_APPROACH_EE_LEFT_OFFSET_X", str(APPROACH_EE_LEFT_OFFSET_X)))
        self.approach_ee_left_offset_y = float(os.getenv("SIMOD_APPROACH_EE_LEFT_OFFSET_Y", str(APPROACH_EE_LEFT_OFFSET_Y)))
        self.approach_ee_right_offset_x = float(os.getenv("SIMOD_APPROACH_EE_RIGHT_OFFSET_X", str(APPROACH_EE_RIGHT_OFFSET_X)))
        self.approach_ee_right_offset_y = float(os.getenv("SIMOD_APPROACH_EE_RIGHT_OFFSET_Y", str(APPROACH_EE_RIGHT_OFFSET_Y)))
        self.approach_ee_offset_z = float(os.getenv("SIMOD_APPROACH_EE_OFFSET_Z", str(APPROACH_EE_OFFSET_Z)))
        self.mock_pallet_forward = float(os.getenv("SIMOD_APPROACH_MOCK_PALLET_FORWARD", str(APPROACH_MOCK_PALLET_FORWARD)))
        self.mock_pallet_standoff = float(os.getenv("SIMOD_APPROACH_MOCK_PALLET_STANDOFF", str(APPROACH_MOCK_PALLET_STANDOFF)))
        self.mock_lateral_half = max(0.0, float(os.getenv("SIMOD_APPROACH_MOCK_LATERAL_HALF", str(APPROACH_MOCK_LATERAL_HALF))))
        self.approach_arm_joint_tol = float(os.getenv("SIMOD_APPROACH_ARM_JOINT_TOL", str(APPROACH_ARM_JOINT_TOL)))
        self.approach_arm_ee_pos_tol = float(os.getenv("SIMOD_APPROACH_ARM_EE_POS_TOL", str(APPROACH_ARM_EE_POS_TOL)))
        self.approach_min_exec_time = float(os.getenv("SIMOD_APPROACH_MIN_EXEC_TIME", str(APPROACH_MIN_EXEC_TIME)))
        self.approach_goal_cache_enabled = os.getenv("SIMOD_APPROACH_GOAL_CACHE", "0").strip().lower() in ("1", "true", "yes", "on")
        self.approach_goal_cache_file = os.getenv("SIMOD_APPROACH_GOAL_CACHE_FILE", APPROACH_GOAL_CACHE_FILE).strip() or APPROACH_GOAL_CACHE_FILE
        self._approach_cfg_start_joints = self._load_initial_joints_from_robot_cfg(self.robot_cfg_path)

        if self.approach_arm_target_mode == "pallet_offset":
            self._approach_arm_goal_joints = {"left": None, "right": None}
            self._approach_arm_goal_source = {"left": "pallet_offset", "right": "pallet_offset"}
        else:
            left_goal_env_raw = os.getenv("SIMOD_APPROACH_LEFT_ARM_GOAL", "").strip()
            right_goal_env_raw = os.getenv("SIMOD_APPROACH_RIGHT_ARM_GOAL", "").strip()
            left_goal_env = self._parse_joint_goal_text(left_goal_env_raw)
            right_goal_env = self._parse_joint_goal_text(right_goal_env_raw)
            if left_goal_env_raw and left_goal_env is None:
                self.get_logger().warn(
                    "Invalid SIMOD_APPROACH_LEFT_ARM_GOAL (expected 6 comma-separated floats). "
                    "Fallback to profile-derived goal."
                )
            if right_goal_env_raw and right_goal_env is None:
                self.get_logger().warn(
                    "Invalid SIMOD_APPROACH_RIGHT_ARM_GOAL (expected 6 comma-separated floats). "
                    "Fallback to profile-derived goal."
                )
            self._approach_arm_goal_joints = {
                "left": np.asarray(left_goal_env, dtype=np.float32) if left_goal_env is not None else None,
                "right": np.asarray(right_goal_env, dtype=np.float32) if right_goal_env is not None else None,
            }
            cfg_l = self._approach_cfg_start_joints.get("left", None)
            cfg_r = self._approach_cfg_start_joints.get("right", None)
            if self._approach_arm_goal_joints["left"] is None and cfg_l is not None:
                self._approach_arm_goal_joints["left"] = self._build_profile_goal(np.asarray(cfg_l, dtype=np.float32), LEFT_ARM_VEL)
            if self._approach_arm_goal_joints["right"] is None and cfg_r is not None:
                self._approach_arm_goal_joints["right"] = self._build_profile_goal(np.asarray(cfg_r, dtype=np.float32), RIGHT_ARM_VEL)
            self._approach_arm_goal_source = {
                "left": "env" if left_goal_env is not None else ("config_profile" if cfg_l is not None else "profile"),
                "right": "env" if right_goal_env is not None else ("config_profile" if cfg_r is not None else "profile"),
            }
        self._approach_goal_ee: Dict[str, Optional[np.ndarray]] = {"left": None, "right": None}
        self._approach_base_goal_xy: Dict[str, Optional[List[float]]] = {"left": None, "right": None}
        self._approach_slot_assignment: Optional[Dict[str, str]] = None
        if self.approach_goal_cache_enabled and self.approach_arm_target_mode != "pallet_offset":
            self._load_approach_goal_cache()

        self.mock_pallet_x_env = os.getenv("SIMOD_APPROACH_PALLET_X", "").strip()
        self.mock_pallet_y_env = os.getenv("SIMOD_APPROACH_PALLET_Y", "").strip()
        self.mock_pallet_z_env = os.getenv("SIMOD_APPROACH_PALLET_Z", "").strip()
        self.approach_only_raw = os.getenv("SIMOD_APPROACH_ONLY", "1")
        self.approach_only_mode = self.approach_only_raw.strip().lower() in ("1", "true", "yes", "on")
        if self.approach_only_mode:
            self.get_logger().info(
                f"Approach-only mode active: Lift/Transport/Drop are paused "
                f"(SIMOD_APPROACH_ONLY='{self.approach_only_raw}')"
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
            f"arm_joint_tol={self.approach_arm_joint_tol:.3f}rad, "
            f"arm_ee_tol={self.approach_arm_ee_pos_tol:.3f}m, "
            f"arm_goal_source=L:{self._approach_arm_goal_source['left']}/R:{self._approach_arm_goal_source['right']}, "
            f"approach_only={self.approach_only_mode} (raw='{self.approach_only_raw}'), "
            f"pallet_model_candidates={self.approach_mock_pallet_model_candidates}, "
            f"base_model_candidates={self._gazebo_base_model_candidates}"
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
        pallet_xy: Optional[List[float]] = None,
        pallet_xyz: Optional[List[float]] = None,
    ) -> bool:
        """
        Inizializza l'Approach TP usando lo stato reale corrente.
        - base target: da offset rispetto al pallet (world frame)
        - arm target: da offset EE rispetto al pallet (world frame) oppure profilo giunti
        """
        try:
            self._approach_slot_assignment = None
            joint_pos_live, base_odom_live = self._build_tp_inputs_from_side_data(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            )

            # Porta il modello cinematico TP nello stato corrente letto dai topic ROS.
            self.robot.compute(joint_pos_live, base_odom_live)
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
            left_goal_ee = left_ee + self._approach_left_delta
            right_goal_ee = right_ee + self._approach_right_delta

            q_left_goal: Optional[np.ndarray] = None
            q_right_goal: Optional[np.ndarray] = None
            left_from_profile = False
            right_from_profile = False
            if self.approach_arm_target_mode == "pallet_offset" and pallet_xyz is not None:
                left_goal_ee = self._compute_side_ee_target_pose("left", left_ee, pallet_xyz)
                right_goal_ee = self._compute_side_ee_target_pose("right", right_ee, pallet_xyz)
                self._approach_arm_goal_source["left"] = "pallet_offset"
                self._approach_arm_goal_source["right"] = "pallet_offset"
            else:
                # Target giunto approach:
                # - usa goal assoluti (env/cache) se disponibili
                # - altrimenti rigenera dal profilo storico
                q_left_goal = self._approach_arm_goal_joints.get("left", None)
                q_right_goal = self._approach_arm_goal_joints.get("right", None)
                if q_left_goal is None or len(q_left_goal) != len(left_arm_jp):
                    q_left_goal = self._build_profile_goal(np.asarray(left_arm_jp, dtype=np.float32), LEFT_ARM_VEL)
                    left_from_profile = True
                    self._approach_arm_goal_source["left"] = "live_profile"
                else:
                    q_left_goal = np.asarray(q_left_goal, dtype=np.float32)
                if q_right_goal is None or len(q_right_goal) != len(right_arm_jp):
                    q_right_goal = self._build_profile_goal(np.asarray(right_arm_jp, dtype=np.float32), RIGHT_ARM_VEL)
                    right_from_profile = True
                    self._approach_arm_goal_source["right"] = "live_profile"
                else:
                    q_right_goal = np.asarray(q_right_goal, dtype=np.float32)

                # Clipping su limiti giunto se disponibili.
                try:
                    j_up_all, j_low_all = self.robot.get_arm_joint_limits_pos()
                    if isinstance(j_up_all, list) and isinstance(j_low_all, list):
                        if 0 <= l_idx < len(j_up_all) and 0 <= l_idx < len(j_low_all):
                            q_left_goal = np.clip(q_left_goal, np.asarray(j_low_all[l_idx], dtype=np.float32), np.asarray(j_up_all[l_idx], dtype=np.float32))
                        if 0 <= r_idx < len(j_up_all) and 0 <= r_idx < len(j_low_all):
                            q_right_goal = np.clip(q_right_goal, np.asarray(j_low_all[r_idx], dtype=np.float32), np.asarray(j_up_all[r_idx], dtype=np.float32))
                except Exception:
                    pass

            # Se disponibile la posa pallet, imponi un target base (x,y,yaw) per lato.
            base_left_goal = np.asarray(left_base, dtype=np.float32)
            base_right_goal = np.asarray(right_base, dtype=np.float32)
            if self.approach_use_base and pallet_xy is not None:
                left_target_xy = self._compute_side_base_target_xy("left", left_base, right_base, pallet_xy)
                right_target_xy = self._compute_side_base_target_xy("right", left_base, right_base, pallet_xy)
                base_left_goal[0] = float(left_target_xy[0])
                base_left_goal[1] = float(left_target_xy[1])
                base_right_goal[0] = float(right_target_xy[0])
                base_right_goal[1] = float(right_target_xy[1])
                # Rotazione opzionale: disattiva per evitare urti/cadute in dinamica fisica.
                if self.approach_base_align_yaw:
                    base_left_goal[5] = float(math.atan2(float(pallet_xy[1]) - float(left_target_xy[1]), float(pallet_xy[0]) - float(left_target_xy[0])))
                    base_right_goal[5] = float(math.atan2(float(pallet_xy[1]) - float(right_target_xy[1]), float(pallet_xy[0]) - float(right_target_xy[0])))
                else:
                    base_left_goal[5] = float(left_base[5])
                    base_right_goal[5] = float(right_base[5])
                self._approach_base_goal_xy["left"] = [float(left_target_xy[0]), float(left_target_xy[1])]
                self._approach_base_goal_xy["right"] = [float(right_target_xy[0]), float(right_target_xy[1])]
            else:
                self._approach_base_goal_xy["left"] = None
                self._approach_base_goal_xy["right"] = None

            # Se ho target giunto, ricavo target EE coerente via FK.
            if q_left_goal is not None and q_right_goal is not None:
                joint_pos_goal, base_odom_goal = self._build_tp_inputs_from_side_data(
                    left_arm_jp=[float(v) for v in q_left_goal],
                    right_arm_jp=[float(v) for v in q_right_goal],
                    left_base=[float(v) for v in base_left_goal.tolist()],
                    right_base=[float(v) for v in base_right_goal.tolist()],
                )
                self.robot.compute(joint_pos_goal, base_odom_goal)
                goal_ee_all = self.robot.get_arm_ee_poses()
                if goal_ee_all is not None and len(goal_ee_all) > max(l_idx, r_idx):
                    left_goal_ee = np.asarray(goal_ee_all[l_idx], dtype=np.float32)
                    right_goal_ee = np.asarray(goal_ee_all[r_idx], dtype=np.float32)

            # Cache target per verifica convergenza reale in ApproachObject.
            self._approach_arm_goal_joints["left"] = np.asarray(q_left_goal, dtype=np.float32) if q_left_goal is not None else None
            self._approach_arm_goal_joints["right"] = np.asarray(q_right_goal, dtype=np.float32) if q_right_goal is not None else None
            self._approach_goal_ee["left"] = np.asarray(left_goal_ee, dtype=np.float32)
            self._approach_goal_ee["right"] = np.asarray(right_goal_ee, dtype=np.float32)
            if q_left_goal is not None and q_right_goal is not None:
                self.get_logger().info(
                    "Approach TP arm goals (joint): "
                    f"left={np.round(np.asarray(q_left_goal, dtype=np.float32), 3).tolist()}, "
                    f"right={np.round(np.asarray(q_right_goal, dtype=np.float32), 3).tolist()}"
                )
            self.get_logger().info(
                "Approach TP arm goals (ee): "
                f"left={np.round(np.asarray(left_goal_ee, dtype=np.float32)[:3], 3).tolist()}, "
                f"right={np.round(np.asarray(right_goal_ee, dtype=np.float32)[:3], 3).tolist()}"
            )
            if (left_from_profile or right_from_profile) and q_left_goal is not None and q_right_goal is not None:
                self._save_approach_goal_cache()

            # Ripristina lo stato live per coerenza interna prima di ripartire coi cycle.
            self.robot.compute(joint_pos_live, base_odom_live)

            mode = str(self.approach_task_mode or "hybrid").lower()
            if mode == "joint" and (q_left_goal is None or q_right_goal is None):
                self.get_logger().warn(
                    "Approach task_mode=joint requested but arm joint goals are not available. "
                    "Falling back to task_mode=hybrid."
                )
                mode = "hybrid"

            base_targets = [
                np.asarray([float(base_left_goal[0]), float(base_left_goal[1]), float(base_left_goal[5])], dtype=np.float32),
                np.asarray([float(base_right_goal[0]), float(base_right_goal[1]), float(base_right_goal[5])], dtype=np.float32),
            ]
            base_kp_yaw = float(self.approach_jtc_base_kp_yaw) if self.approach_base_align_yaw else 0.0
            k_omni = np.asarray(
                [
                    [float(self.approach_jtc_base_kp_xy), float(self.approach_jtc_base_kp_xy), float(base_kp_yaw)],
                    [float(self.approach_jtc_base_kp_xy), float(self.approach_jtc_base_kp_xy), float(base_kp_yaw)],
                ],
                dtype=np.float32,
            )
            k_arm = np.full((len(left_arm_jp) + len(right_arm_jp),), float(self.approach_jtc_arm_kp), dtype=np.float32)
            current_joint_targets = [np.asarray(left_arm_jp, dtype=np.float32), np.asarray(right_arm_jp, dtype=np.float32)]

            if mode in ("joint", "hybrid") and self.approach_jtc_task is not None:
                try:
                    self.approach_jtc_task.activate()
                    self.approach_jtc_task.set_activation("base", bool(self.approach_use_base))
                    self.approach_jtc_task.set_activation("arm", bool(mode == "joint"))
                except Exception:
                    pass
                self.approach_jtc_task.set_omni_trajectories_pnts(
                    target_cart_positions=base_targets,
                    K_omni=k_omni,
                    period=float(self.approach_traj_time),
                )
                if mode == "joint":
                    self.approach_jtc_task.set_arm_trajectories_pnts(
                        target_joint_positions=[np.asarray(q_left_goal, dtype=np.float32), np.asarray(q_right_goal, dtype=np.float32)],
                        K_arm=k_arm,
                        period=float(self.approach_traj_time),
                    )
                else:
                    # In hybrid mode il braccio è controllato da ArmCartesianControl.
                    # Mantengo il target giunto uguale allo stato attuale per evitare transitori nel task JTC.
                    self.approach_jtc_task.set_arm_trajectories_pnts(
                        target_joint_positions=current_joint_targets,
                        K_arm=k_arm,
                        period=float(self.approach_traj_time),
                    )
                pallet_xy_log = [float(pallet_xy[0]), float(pallet_xy[1])] if pallet_xy is not None else None
                self.get_logger().info(
                    f"Approach TP base targets ({mode}-mode): "
                    f"left=[x={base_targets[0][0]:.3f}, y={base_targets[0][1]:.3f}, yaw={base_targets[0][2]:.3f}], "
                    f"right=[x={base_targets[1][0]:.3f}, y={base_targets[1][1]:.3f}, yaw={base_targets[1][2]:.3f}], "
                    f"pallet={pallet_xy_log}, slot_assign={self._approach_slot_assignment}, kp_yaw={base_kp_yaw:.2f}"
                )
            else:
                try:
                    if self.approach_jtc_task is not None:
                        self.approach_jtc_task.deactivate()
                except Exception:
                    pass

            if mode in ("ee", "hybrid"):
                self.tr_left = Trajectory()
                self.tr_right = Trajectory()
                self.tr_left.poly5(
                    p_i=left_ee,
                    p_f=left_goal_ee,
                    period=float(self.approach_traj_time),
                )
                self.tr_right.poly5(
                    p_i=right_ee,
                    p_f=right_goal_ee,
                    period=float(self.approach_traj_time),
                )
                try:
                    if self.ee_task is not None:
                        self.ee_task.activate()
                except Exception:
                    pass
                ee_use_base = bool(self.approach_use_base and mode == "ee")
                if hasattr(self.ee_task, "set_use_base"):
                    self.ee_task.set_use_base(ee_use_base)
                else:
                    self.ee_task.use_base = ee_use_base
                self.ee_task.set_trajectory([self.tr_left, self.tr_right])
            else:
                try:
                    if self.ee_task is not None:
                        self.ee_task.deactivate()
                except Exception:
                    pass
            self._tp_approach_traj_initialized = True
            self._tp_last_exec_monotonic = None
            self._tp_cmd_cache = None
            self._tp_cmd_cache_time = None
            self._approach_pallet_source_used = str(self.bb.get("pallet_pose_source", "unset")).lower()
            self._approach_pallet_xy_used = [float(pallet_xy[0]), float(pallet_xy[1])] if pallet_xy is not None else None

            self.get_logger().info(
                "Approach TP trajectory initialized "
                f"(mode={mode}, "
                f"goal_source=L:{self._approach_arm_goal_source['left']}/R:{self._approach_arm_goal_source['right']}, "
                f"base_goal={'pallet' if self._approach_base_goal_xy['left'] is not None else 'current'}, "
                f"pallet_source={self._approach_pallet_source_used}, "
                f"T={self.approach_traj_time:.1f}s, duration_ref={self.approach_duration:.1f}s)"
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
        self._try_init_base_world_calibration(target_side)

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

    @staticmethod
    def _odom_msg_to_pose6(msg: Odometry) -> List[float]:
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        pz = float(msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        roll, pitch, yaw = BTDemoNode._quat_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))
        return [px, py, pz, float(roll), float(pitch), float(yaw)]

    def _try_init_base_world_calibration(self, side: str):
        side = str(side).lower()
        if side not in ("left", "right"):
            return
        if self._base_world_calib.get(side) is not None:
            return
        gazebo_pose = self._gazebo_base_pose_start.get(side)
        odom_msg = self._last_odom.get(side)
        if gazebo_pose is None or odom_msg is None:
            return

        try:
            odom_pose = self._odom_msg_to_pose6(odom_msg)
            dyaw = self._angle_diff(float(gazebo_pose[5]), float(odom_pose[5]))
            c = math.cos(dyaw)
            s = math.sin(dyaw)
            x_o = float(odom_pose[0])
            y_o = float(odom_pose[1])
            dx = float(gazebo_pose[0]) - (c * x_o - s * y_o)
            dy = float(gazebo_pose[1]) - (s * x_o + c * y_o)
            self._base_world_calib[side] = {
                "dx": float(dx),
                "dy": float(dy),
                "dyaw": float(dyaw),
            }
            self.get_logger().info(
                f"[BaseFrame] calibrated odom->world for {side}: "
                f"dx={dx:.3f}, dy={dy:.3f}, dyaw={dyaw:.3f} rad "
                f"(model={self._gazebo_base_pose_start_model.get(side)})"
            )
        except Exception as exc:
            self._warn_throttled(
                f"base_calib_{side}",
                f"[BaseFrame] failed odom->world calibration for {side}: {exc}",
                period_s=5.0,
            )

    def _map_odom_pose_to_world(self, side: str, odom_pose: List[float]) -> Optional[List[float]]:
        calib = self._base_world_calib.get(str(side).lower())
        if calib is None:
            return None
        dyaw = float(calib.get("dyaw", 0.0))
        c = math.cos(dyaw)
        s = math.sin(dyaw)

        x_o = float(odom_pose[0])
        y_o = float(odom_pose[1])
        x_w = c * x_o - s * y_o + float(calib.get("dx", 0.0))
        y_w = s * x_o + c * y_o + float(calib.get("dy", 0.0))
        yaw_w = float(odom_pose[5]) + dyaw
        yaw_w = (yaw_w + math.pi) % (2.0 * math.pi) - math.pi
        return [x_w, y_w, float(odom_pose[2]), float(odom_pose[3]), float(odom_pose[4]), yaw_w]
    
    def get_base_pose(self, side: str):
        """
        Return tuple (x, y, z, roll, pitch, yaw) or None if no data yet.
        side: 'left' or 'right'
        """
        side = str(side).lower()
        source = str(self.approach_base_pose_source or "aligned_odom").strip().lower()
        odom = self._last_odom.get(side)
        odom_pose = self._odom_msg_to_pose6(odom) if odom is not None else None
        gazebo_pose = self._gazebo_base_pose_last.get(side)

        if source == "gazebo":
            if gazebo_pose is not None:
                return [float(v) for v in gazebo_pose]
            return odom_pose

        if source in ("aligned_odom", "auto"):
            if odom_pose is not None:
                mapped = self._map_odom_pose_to_world(side, odom_pose)
                if mapped is not None:
                    return mapped
                if source == "aligned_odom":
                    self._warn_throttled(
                        f"base_pose_calib_missing_{side}",
                        f"[BaseFrame] odom->world calibration missing for {side}, using raw odom pose",
                        period_s=5.0,
                    )
                return odom_pose
            if gazebo_pose is not None:
                return [float(v) for v in gazebo_pose]
            return None

        # source == "odom"
        if odom_pose is not None:
            return odom_pose
        if gazebo_pose is not None:
            return [float(v) for v in gazebo_pose]
        return None
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

    def _ensure_mock_pallet_pose(
        self,
        left_base: Optional[List[float]] = None,
        right_base: Optional[List[float]] = None,
    ) -> Optional[List[float]]:
        """
        Simula il riconoscimento visivo pallet:
        - se presenti env SIMOD_APPROACH_PALLET_X/Y usa quelli
        - altrimenti usa la posa iniziale del modello Gazebo (pacco/pallet)
        - fallback: stima da odometria (centro basi + offset in avanti).
        """
        source = str(self.bb.get("pallet_pose_source", "unset")).lower()
        pose = self.bb.get("pallet_pose_world", None)

        if self.mock_pallet_x_env and self.mock_pallet_y_env:
            try:
                px = float(self.mock_pallet_x_env)
                py = float(self.mock_pallet_y_env)
                needs_update = (
                    source != "env"
                    or not (isinstance(pose, (list, tuple)) and len(pose) >= 2)
                    or abs(float(pose[0]) - px) > 1e-6
                    or abs(float(pose[1]) - py) > 1e-6
                )
                if needs_update:
                    self.bb["pallet_pose_world"] = [px, py]
                    self.bb["pallet_pose_source"] = "env"
                    self.bb["pallet_info_ready"] = True
                    self.get_logger().info(
                        bt_fmt(f"[VisionMock] pallet pose from env: x={px:.3f}, y={py:.3f}")
                    )
                return [px, py]
            except Exception as exc:
                self._warn_throttled("vision_mock_env_invalid", f"[VisionMock] invalid env pallet pose: {exc}")

        # Default mock: usa la posa iniziale del modello pacco/pallet in Gazebo.
        if self._gazebo_pallet_pose_start_xy is not None and source != "env":
            gx, gy = float(self._gazebo_pallet_pose_start_xy[0]), float(self._gazebo_pallet_pose_start_xy[1])
            if source != "gazebo" or not (isinstance(pose, (list, tuple)) and len(pose) >= 2):
                self.bb["pallet_pose_world"] = [gx, gy]
                self.bb["pallet_pose_source"] = "gazebo"
                self.bb["pallet_info_ready"] = True
                self.get_logger().info(
                    bt_fmt(
                        "[VisionMock] pallet pose from Gazebo startup snapshot "
                        f"({self._gazebo_pallet_pose_start_model}): x={gx:.3f}, y={gy:.3f}"
                    )
                )
            return [gx, gy]

        # Fallback robusto: leggi dal file world usato da Gazebo (snapshot iniziale).
        if source != "env":
            world_xy = self._load_pallet_pose_from_world_file()
            if world_xy is not None:
                wx, wy = float(world_xy[0]), float(world_xy[1])
                if source != "world_file" or not (isinstance(pose, (list, tuple)) and len(pose) >= 2):
                    self.bb["pallet_pose_world"] = [wx, wy]
                    self.bb["pallet_pose_source"] = "world_file"
                    self.bb["pallet_info_ready"] = True
                    self.get_logger().info(
                        bt_fmt(
                            "[VisionMock] pallet pose from world file snapshot "
                            f"({self._world_pallet_pose_model}): x={wx:.3f}, y={wy:.3f}"
                        )
                    )
                return [wx, wy]

        if isinstance(pose, (list, tuple)) and len(pose) >= 2:
            return [float(pose[0]), float(pose[1])]

        if left_base is None:
            left_base = self.get_base_pose("left")
        if right_base is None:
            right_base = self.get_base_pose("right")
        if left_base is None or right_base is None:
            return None

        px = 0.5 * (float(left_base[0]) + float(right_base[0]))
        py = max(float(left_base[1]), float(right_base[1])) + float(self.mock_pallet_forward)
        self.bb["pallet_pose_world"] = [px, py]
        self.bb["pallet_pose_source"] = "estimated"
        self.bb["pallet_info_ready"] = True
        self.get_logger().info(
            bt_fmt(
                "[VisionMock] estimated pallet pose "
                f"x={px:.3f}, y={py:.3f}, source=estimated"
            )
        )
        return [px, py]

    def _ensure_mock_pallet_pose_xyz(
        self,
        left_base: Optional[List[float]] = None,
        right_base: Optional[List[float]] = None,
    ) -> Optional[List[float]]:
        xy = self._ensure_mock_pallet_pose(left_base=left_base, right_base=right_base)
        if xy is None:
            return None

        source = str(self.bb.get("pallet_pose_source", "unset")).lower()
        pose_xyz = self.bb.get("pallet_pose_world_xyz", None)
        if isinstance(pose_xyz, (list, tuple)) and len(pose_xyz) >= 3:
            if abs(float(pose_xyz[0]) - float(xy[0])) < 1e-6 and abs(float(pose_xyz[1]) - float(xy[1])) < 1e-6:
                return [float(pose_xyz[0]), float(pose_xyz[1]), float(pose_xyz[2])]

        z = 0.0
        if source == "env" and self.mock_pallet_z_env:
            try:
                z = float(self.mock_pallet_z_env)
            except Exception:
                z = 0.0
        elif self._gazebo_pallet_pose_start_xyz is not None:
            z = float(self._gazebo_pallet_pose_start_xyz[2])
        elif self._gazebo_pallet_pose_last_xyz is not None:
            z = float(self._gazebo_pallet_pose_last_xyz[2])
        elif self.mock_pallet_z_env:
            try:
                z = float(self.mock_pallet_z_env)
            except Exception:
                z = 0.0

        xyz = [float(xy[0]), float(xy[1]), float(z)]
        self.bb["pallet_pose_world_xyz"] = xyz
        return xyz

    def _compute_side_base_target_xy(
        self,
        side: str,
        left_base: List[float],
        right_base: List[float],
        pallet_xy: List[float],
    ) -> List[float]:
        side = str(side).lower()
        if self.approach_base_target_mode == "pallet_offset":
            if side == "left":
                tx = float(pallet_xy[0]) + float(self.approach_base_left_offset_x)
                ty = float(pallet_xy[1]) + float(self.approach_base_left_offset_y)
            else:
                tx = float(pallet_xy[0]) + float(self.approach_base_right_offset_x)
                ty = float(pallet_xy[1]) + float(self.approach_base_right_offset_y)
            self._approach_slot_assignment = {"left": "left", "right": "right"}
            self.bb["approach_side_order"] = "pallet_offset_fixed"
            self.bb["approach_slot_assignment"] = "left->left,right->right"
            return [tx, ty]

        if self.approach_base_keep_lane:
            # Mantiene la corsia iniziale: solo avanzamento lungo y verso il pallet.
            tx = float(left_base[0]) if side == "left" else float(right_base[0])
            ty = float(pallet_xy[1]) - float(self.mock_pallet_standoff)
            self.bb["approach_side_order"] = "lane_lock"
            self.bb["approach_lateral_half"] = 0.5 * abs(float(right_base[0]) - float(left_base[0]))
            return [tx, ty]

        lateral_half = float(self.mock_lateral_half)
        self.bb["approach_lateral_half"] = float(lateral_half)
        self.bb["approach_side_order"] = "full_2d"
        slot_assign = self._approach_slot_assignment
        if not isinstance(slot_assign, dict) or "left" not in slot_assign or "right" not in slot_assign:
            slot_assign = {"left": "left", "right": "right"}
            if self.approach_base_auto_swap_slots:
                lx, ly = float(left_base[0]), float(left_base[1])
                rx, ry = float(right_base[0]), float(right_base[1])
                sx_l = float(pallet_xy[0]) - float(lateral_half)
                sx_r = float(pallet_xy[0]) + float(lateral_half)
                sy = float(pallet_xy[1]) - float(self.mock_pallet_standoff)
                cost_fixed = math.hypot(sx_l - lx, sy - ly) + math.hypot(sx_r - rx, sy - ry)
                cost_swap = math.hypot(sx_r - lx, sy - ly) + math.hypot(sx_l - rx, sy - ry)
                if cost_swap + float(self.approach_base_swap_hyst) < cost_fixed:
                    slot_assign = {"left": "right", "right": "left"}
            self._approach_slot_assignment = slot_assign
        self.bb["approach_slot_assignment"] = f"left->{slot_assign['left']},right->{slot_assign['right']}"

        slot = slot_assign.get(side, side)
        side_sign = -1.0 if slot == "left" else 1.0
        tx = float(pallet_xy[0]) + side_sign * float(lateral_half)
        ty = float(pallet_xy[1]) - float(self.mock_pallet_standoff)
        return [tx, ty]

    def _compute_side_ee_target_pose(
        self,
        side: str,
        live_ee_pose: np.ndarray,
        pallet_xyz: List[float],
    ) -> np.ndarray:
        side = str(side).lower()
        goal = np.asarray(live_ee_pose, dtype=np.float32).copy()
        if goal.shape[0] < 6:
            goal = np.pad(goal, (0, max(0, 6 - goal.shape[0])), mode="constant")
        if side == "left":
            goal[0] = float(pallet_xyz[0]) + float(self.approach_ee_left_offset_x)
            goal[1] = float(pallet_xyz[1]) + float(self.approach_ee_left_offset_y)
        else:
            goal[0] = float(pallet_xyz[0]) + float(self.approach_ee_right_offset_x)
            goal[1] = float(pallet_xyz[1]) + float(self.approach_ee_right_offset_y)
        goal[2] = float(pallet_xyz[2]) + float(self.approach_ee_offset_z)
        return goal

    def _compute_base_cmd_from_pallet(
        self,
        side: str,
        base_pose: List[float],
        target_xy: List[float],
        ramp: float,
    ) -> tuple[List[float], bool]:
        # Errore in world frame
        dx_w = float(target_xy[0]) - float(base_pose[0])
        dy_w = float(target_xy[1]) - float(base_pose[1])
        yaw = float(base_pose[5])

        # Conversione world -> base frame (solo traslazione, no rotazione)
        ex_b = math.cos(yaw) * dx_w + math.sin(yaw) * dy_w
        ey_b = -math.sin(yaw) * dx_w + math.cos(yaw) * dy_w

        vx = float(self.approach_base_kp_x) * ex_b
        vy = float(self.approach_base_kp_y) * ey_b
        cmd = _sanitize_base_cmd(
            [vx, vy, 0.0],
            xy_abs_max=self.tp_base_cmd_xy_abs_max,
            wz_abs_max=self.tp_base_cmd_wz_abs_max,
            ramp=ramp,
        )
        reached = (dx_w * dx_w + dy_w * dy_w) <= float(self.approach_base_goal_tol) ** 2
        self.bb[f"{'SRM1' if side == 'left' else 'SRM2'}_base_goal_reached"] = bool(reached)
        return cmd, reached

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        d = float(a) - float(b)
        return (d + math.pi) % (2.0 * math.pi) - math.pi

    def _get_live_ee_by_side(
        self,
        left_arm_jp: List[float],
        right_arm_jp: List[float],
        left_base: List[float],
        right_base: List[float],
    ) -> Dict[str, Optional[np.ndarray]]:
        try:
            joint_pos, base_odom = self._build_tp_inputs_from_side_data(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            )
            self.robot.compute(joint_pos, base_odom)
            ee = self.robot.get_arm_ee_poses()
            if ee is None or len(ee) == 0:
                return {"left": None, "right": None}
            l_idx = int((self._arm_ee_index or {}).get("left", 0))
            r_idx = int((self._arm_ee_index or {}).get("right", 1))
            if l_idx < 0 or l_idx >= len(ee):
                l_idx = 0
            if r_idx < 0 or r_idx >= len(ee):
                r_idx = min(1, len(ee) - 1)
            return {
                "left": np.asarray(ee[l_idx], dtype=np.float32),
                "right": np.asarray(ee[r_idx], dtype=np.float32),
            }
        except Exception:
            return {"left": None, "right": None}

    def _is_side_arm_converged(
        self,
        side: str,
        arm_jp: List[float],
        live_ee_pose: Optional[np.ndarray],
    ) -> tuple[bool, float, float]:
        goal_j = self._approach_arm_goal_joints.get(side, None)
        goal_ee = self._approach_goal_ee.get(side, None)
        j_err = float("inf")
        ee_err = float("inf")
        joint_ok = False
        ee_ok = False

        if goal_j is not None and len(goal_j) == len(arm_jp):
            diffs = [abs(self._angle_diff(arm_jp[i], goal_j[i])) for i in range(len(arm_jp))]
            j_err = float(max(diffs)) if diffs else 0.0
            joint_ok = j_err <= float(self.approach_arm_joint_tol)

        if goal_ee is not None and live_ee_pose is not None and len(goal_ee) >= 3 and len(live_ee_pose) >= 3:
            ee_err = float(np.linalg.norm(np.asarray(live_ee_pose[:3], dtype=np.float32) - np.asarray(goal_ee[:3], dtype=np.float32)))
            ee_ok = ee_err <= float(self.approach_arm_ee_pos_tol)

        # Se non ho un tipo di misura, considero valida l'altra.
        if not np.isfinite(j_err):
            joint_ok = True
            j_err = -1.0
        if not np.isfinite(ee_err):
            ee_ok = True
            ee_err = -1.0

        return bool(joint_ok and ee_ok), float(j_err), float(ee_err)


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
CURRENT_BT_NODE_ATTRS = contextvars.ContextVar("CURRENT_BT_NODE_ATTRS", default={})


def get_current_bt_name() -> str:
    return CURRENT_BT_NAME.get("<unknown>")


def get_current_bt_attrs() -> Dict[str, str]:
    attrs = CURRENT_BT_NODE_ATTRS.get({})
    return attrs if isinstance(attrs, dict) else {}


def get_bt_attr(name: str, default: Optional[str] = None) -> Optional[str]:
    return get_current_bt_attrs().get(name, default)


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
                attrs = dict(xml_elem.attrib) if xml_elem.attrib else {}
                if attrs:
                    def _wrapped_func(_func=func_obj, _attrs=attrs):
                        token_attrs = CURRENT_BT_NODE_ATTRS.set(_attrs)
                        try:
                            return _func()
                        finally:
                            CURRENT_BT_NODE_ATTRS.reset(token_attrs)
                    leaf_func = _wrapped_func
                else:
                    leaf_func = func_obj
                parent_node._Tree__add_child("Action", depth, child_index, func_name, leaf_func)
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
        token_attrs = CURRENT_BT_NODE_ATTRS.set({})
        try:
            result = self.func_()  # True / False / None
        finally:
            CURRENT_BT_NODE_ATTRS.reset(token_attrs)
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
    timer_key = f"{get_current_bt_name()}_Sync"
    t0 = node.get_action_timer(timer_key)
    if t0 is None:
        node.get_logger().info(bt_fmt("[Sync] start"))
        t0 = node.start_action_timer(timer_key)

    if node.get_clock().now().nanoseconds/1e9 - t0 < 1.0:
        return None  # RUNNING
    node.clear_action_timer(timer_key)
    node.get_logger().info(bt_fmt("[Sync] done"))
    return True


def FindObj():
    """
    Identificazione pacco target.
    Mock: attende 1 s e restituisce SUCCESS.
    """
    node = _require_node()
    action_name = get_bt_attr("name", "FindObj")
    timer_key = f"{get_current_bt_name()}_FindObj"
    t0 = node.get_action_timer(timer_key)
    if t0 is None:
        node.get_logger().info(bt_fmt(f"[FindObj] start ({action_name})"))
        t0 = node.start_action_timer(timer_key)

    if node.get_clock().now().nanoseconds/1e9 - t0 < 1.0:
        return None
    node.clear_action_timer(timer_key)
    node.bb["target_found"] = True
    # Simula il riconoscimento pallet appena disponibili odometrie.
    tree_name = get_current_bt_name()
    if tree_name in ("SRM1", "SRM2"):
        _ = node._ensure_mock_pallet_pose()
    # SRM2 deve partire appena SRM1 ha stimato il pallet (prima della Sync).
    if tree_name == "SRM1" and "pallet" in str(action_name).lower():
        node.bb["srm1_data_to_srm2"] = bool(node.bb.get("pallet_info_ready", False))
        if node.bb["srm1_data_to_srm2"]:
            node.get_logger().info(bt_fmt("[FindObj] SRM1 payload published for SRM2"))
    node.get_logger().info(bt_fmt(f"[FindObj] target_found = True ({action_name})"))
    return True


def CalculateGoal():
    """
    Analisi posizione pacco / calcolo goal.
    Mock: SUCCESS immediato con side-effect sulla blackboard.
    """
    node = _require_node()
    goal_name = get_bt_attr("goal", "goal")
    node.bb["goal_computed"] = True
    node.bb["last_goal_name"] = goal_name
    node.get_logger().info(bt_fmt(f"[CalculateGoal] goal_computed = True ({goal_name})"))
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
    message = get_bt_attr("message", "(mock)")
    node.get_logger().info(bt_fmt(f"[SaySomething] {message}"))

    # Simula passaggio informazione SRM1 -> SRM2 sulla posizione del pallet.
    if get_current_bt_name() == "SRM1" and "pallet" in str(message).lower():
        if not node.bb.get("pallet_info_ready", False):
            _ = node._ensure_mock_pallet_pose()
        node.bb["srm1_data_to_srm2"] = bool(node.bb.get("pallet_info_ready", False))
    return True


def NearObj():
    """
    Condizione "vicino all'oggetto".
    In questa demo viene legata al completamento di ApproachObject.
    """
    node = _require_node()
    tree_name = get_current_bt_name()
    name_attr = str(get_bt_attr("name", "") or "").lower()
    if "srm1" in name_attr:
        key = "SRM1_near_object"
    elif "srm2" in name_attr:
        key = "SRM2_near_object"
    else:
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
    timer_key = f"{get_current_bt_name()}_DataReceived"
    t0 = node.get_action_timer(timer_key)
    if t0 is None:
        node.get_logger().info(bt_fmt("[DataReceived] waiting SRM1 payload"))
        t0 = node.start_action_timer(timer_key)

    # Attende esplicitamente il "messaggio" condiviso da SRM1 (mock visione).
    if not bool(node.bb.get("srm1_data_to_srm2", False)):
        return None
    node.clear_action_timer(timer_key)
    node.get_logger().info(bt_fmt("[DataReceived] done (SRM1 payload available)"))
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
    Avvicinamento in modalità Task Prioritization:
    - SRM1 e SRM2 lavorano separatamente prima della Sync
    - traiettoria EE target costruita da profilo giunti + target pallet
      (quindi TP genera comandi sia base che braccio)
    - al termine setta <TREE>_near_object=True
    """
    node = _require_node()
    tree_name = get_current_bt_name()
    side = node.side_from_bt_name(tree_name)
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

    # SRM2 procede solo dopo "messaggio" SRM1 (simulazione visione + data sharing).
    if side == "right" and not bool(node.bb.get("srm1_data_to_srm2", False)):
        node._info_throttled(
            f"{tree_name}_approach_wait_data",
            bt_fmt("[ApproachObject] waiting pallet info from SRM1"),
            period_s=1.0,
        )
        return None

    pallet_xy = node._ensure_mock_pallet_pose(left_base=left_base, right_base=right_base)
    pallet_xyz = node._ensure_mock_pallet_pose_xyz(left_base=left_base, right_base=right_base)

    t0 = node.get_action_timer(timer_key)
    if t0 is None:
        t0 = node.start_action_timer(timer_key)
        node.bb[near_key] = False
        node._approach_slot_assignment = None
        if side == "left":
            node.bb["SRM1_base_goal_reached"] = False
        elif side == "right":
            node.bb["SRM2_base_goal_reached"] = False
        node.get_logger().info(
            bt_fmt(
                f"[ApproachObject] start TP approach "
                f"(side={side or 'both'}) "
                f"(use_base={node.approach_use_base}) "
                f"(dur={node.approach_duration:.1f}s)"
            )
        )

    elapsed_from_start = node.get_clock().now().nanoseconds / 1e9 - t0
    pallet_source = str(node.bb.get("pallet_pose_source", "unset")).lower()
    if (
        node.approach_use_base
        and pallet_xy is not None
        and pallet_source == "estimated"
        and elapsed_from_start < float(node.approach_estimated_timeout)
    ):
        node._info_throttled(
            f"{tree_name}_approach_wait_pallet_source",
            bt_fmt(
                "[ApproachObject] waiting for pallet pose source "
                f"(current={pallet_source}, t={elapsed_from_start:.2f}/{node.approach_estimated_timeout:.2f}s)"
            ),
            period_s=0.5,
        )
        return None

    if not node._tp_approach_traj_initialized:
        ok = node._init_tp_approach_trajectory_from_live_state(
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
            pallet_xy=pallet_xy,
            pallet_xyz=pallet_xyz,
        )
        if not ok:
            return None  # RUNNING
    else:
        used_source = str(getattr(node, "_approach_pallet_source_used", "unset")).lower()
        if node.approach_use_base and used_source == "estimated" and pallet_source in ("env", "gazebo", "world_file"):
            node.get_logger().info(
                bt_fmt(
                    "[ApproachObject] reinitializing TP trajectory with upgraded pallet source "
                    f"({used_source} -> {pallet_source})"
                )
            )
            ok = node._init_tp_approach_trajectory_from_live_state(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
                pallet_xy=pallet_xy,
                pallet_xyz=pallet_xyz,
            )
            if not ok:
                return None

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

    left_base_reached = True
    right_base_reached = True
    left_target_xy = None
    right_target_xy = None
    left_base_cmd_vis = [0.0, 0.0, 0.0]
    right_base_cmd_vis = [0.0, 0.0, 0.0]
    if pallet_xy is not None:
        left_target_xy = node._compute_side_base_target_xy("left", left_base, right_base, pallet_xy)
        right_target_xy = node._compute_side_base_target_xy("right", left_base, right_base, pallet_xy)
        left_base_cmd_vis, left_base_reached = node._compute_base_cmd_from_pallet("left", left_base, left_target_xy, ramp=ramp)
        right_base_cmd_vis, right_base_reached = node._compute_base_cmd_from_pallet("right", right_base, right_target_xy, ramp=ramp)
    else:
        left_base_reached = False
        right_base_reached = False

    if node.approach_use_base:
        # Full-TP mode: anche la base è comandata da TP.
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
        left_base_reached = True
        right_base_reached = True
    left_base_dist = float("inf")
    right_base_dist = float("inf")
    if left_target_xy is not None:
        left_base_dist = float(math.hypot(float(left_target_xy[0]) - float(left_base[0]), float(left_target_xy[1]) - float(left_base[1])))
    if right_target_xy is not None:
        right_base_dist = float(math.hypot(float(right_target_xy[0]) - float(right_base[0]), float(right_target_xy[1]) - float(right_base[1])))

    left_arm_cmd_vals = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "left"), node.tp_arm_cmd_abs_max)
    right_arm_cmd_vals = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "right"), node.tp_arm_cmd_abs_max)
    ee_live = node._get_live_ee_by_side(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    left_arm_reached, left_j_err, left_ee_err = node._is_side_arm_converged("left", left_arm_jp, ee_live.get("left"))
    right_arm_reached, right_j_err, right_ee_err = node._is_side_arm_converged("right", right_arm_jp, ee_live.get("right"))
    left_tp_ee_z = float(ee_live["left"][2]) if isinstance(ee_live.get("left"), np.ndarray) and ee_live["left"].shape[0] >= 3 else float("nan")
    right_tp_ee_z = float(ee_live["right"][2]) if isinstance(ee_live.get("right"), np.ndarray) and ee_live["right"].shape[0] >= 3 else float("nan")
    left_base_z_used = float(left_base[2]) if isinstance(left_base, (list, tuple)) and len(left_base) >= 3 else float("nan")
    right_base_z_used = float(right_base[2]) if isinstance(right_base, (list, tuple)) and len(right_base) >= 3 else float("nan")
    left_base_gz = node._gazebo_base_pose_last.get("left", None)
    right_base_gz = node._gazebo_base_pose_last.get("right", None)
    left_base_z_gz = float(left_base_gz[2]) if isinstance(left_base_gz, (list, tuple)) and len(left_base_gz) >= 3 else float("nan")
    right_base_z_gz = float(right_base_gz[2]) if isinstance(right_base_gz, (list, tuple)) and len(right_base_gz) >= 3 else float("nan")
    left_gz_ee = node._gazebo_ee_pose_last.get("left", None)
    right_gz_ee = node._gazebo_ee_pose_last.get("right", None)
    left_gz_ee_z = float(left_gz_ee[2]) if isinstance(left_gz_ee, (list, tuple)) and len(left_gz_ee) >= 3 else float("nan")
    right_gz_ee_z = float(right_gz_ee[2]) if isinstance(right_gz_ee, (list, tuple)) and len(right_gz_ee) >= 3 else float("nan")
    ee_z_gap_tp = (left_tp_ee_z - right_tp_ee_z) if (math.isfinite(left_tp_ee_z) and math.isfinite(right_tp_ee_z)) else float("nan")
    ee_z_gap_gz = (left_gz_ee_z - right_gz_ee_z) if (math.isfinite(left_gz_ee_z) and math.isfinite(right_gz_ee_z)) else float("nan")
    left_ee_z_tp_gz = (left_tp_ee_z - left_gz_ee_z) if (math.isfinite(left_tp_ee_z) and math.isfinite(left_gz_ee_z)) else float("nan")
    right_ee_z_tp_gz = (right_tp_ee_z - right_gz_ee_z) if (math.isfinite(right_tp_ee_z) and math.isfinite(right_gz_ee_z)) else float("nan")

    if side == "left":
        base_cmd_vals = left_base_cmd_vals
        arm_cmd_vals = left_arm_cmd_vals
        arm_reached = left_arm_reached
        base_reached = left_base_reached
        arm_j_err = left_j_err
        arm_ee_err = left_ee_err
        base_dist = left_base_dist
    elif side == "right":
        base_cmd_vals = right_base_cmd_vals
        arm_cmd_vals = right_arm_cmd_vals
        arm_reached = right_arm_reached
        base_reached = right_base_reached
        arm_j_err = right_j_err
        arm_ee_err = right_ee_err
        base_dist = right_base_dist
    else:
        base_cmd_vals = [max(abs(v) for v in left_base_cmd_vals), max(abs(v) for v in right_base_cmd_vals), 0.0]
        arm_cmd_vals = [max(abs(v) for v in left_arm_cmd_vals), max(abs(v) for v in right_arm_cmd_vals)]
        arm_reached = bool(left_arm_reached and right_arm_reached)
        base_reached = bool(left_base_reached and right_base_reached)
        arm_j_err = max(left_j_err, right_j_err)
        arm_ee_err = max(left_ee_err, right_ee_err)
        base_dist = max(left_base_dist, right_base_dist)

    approach_converged = bool(arm_reached and base_reached and (elapsed >= float(node.approach_min_exec_time)))

    node._info_throttled(
        f"{tree_name}_approach_cmd",
        bt_fmt(
            "[ApproachObject] cmd "
            f"(side={side or 'both'}) "
            f"base=[{base_cmd_vals[0]:.4f},{base_cmd_vals[1]:.4f},{base_cmd_vals[2]:.4f}], "
            f"arm_max={max(abs(v) for v in arm_cmd_vals):.4f}, "
            f"arm_reached={arm_reached}, base_reached={base_reached}, "
            f"arm_j_err={arm_j_err:.4f}, arm_ee_err={arm_ee_err:.4f}, base_dist={base_dist:.3f}, "
            f"base_z_used=[L:{left_base_z_used:.3f},R:{right_base_z_used:.3f}], "
            f"base_z_gz=[L:{left_base_z_gz:.3f},R:{right_base_z_gz:.3f}], "
            f"ee_z_tp=[L:{left_tp_ee_z:.3f},R:{right_tp_ee_z:.3f}], "
            f"ee_z_gz=[L:{left_gz_ee_z:.3f},R:{right_gz_ee_z:.3f}], "
            f"ee_z_gap_tp={ee_z_gap_tp:.3f}, ee_z_gap_gz={ee_z_gap_gz:.3f}, "
            f"ee_z_tp-gz=[L:{left_ee_z_tp_gz:.3f},R:{right_ee_z_tp_gz:.3f}], "
            f"pallet_src={node.bb.get('pallet_pose_source', 'unset')}, "
            f"side_order={node.bb.get('approach_side_order', 'n/a')}, "
            f"slot_assign={node.bb.get('approach_slot_assignment', 'n/a')}, "
            f"dt={node.tp._delta_t:.4f}s, arm_clip={node.tp_arm_cmd_abs_max:.3f}, "
            f"base_clip_xy={node.tp_base_cmd_xy_abs_max:.3f}, base_clip_wz={node.tp_base_cmd_wz_abs_max:.3f}, "
            f"ramp={ramp:.2f}, base_ctrl={node.approach_base_ctrl}"
        ),
        period_s=1.0,
    )

    # Pubblica solo il lato gestito da questo BT locale.
    if side == "left":
        tl = Twist()
        la = Float64MultiArray()
        tl.linear.x, tl.linear.y, tl.angular.z = left_base_cmd_vals
        la.data = left_arm_cmd_vals
        node.left_base_pub.publish(tl)
        node.left_arm_pub.publish(la)
    elif side == "right":
        tr = Twist()
        ra = Float64MultiArray()
        tr.linear.x, tr.linear.y, tr.angular.z = right_base_cmd_vals
        ra.data = right_arm_cmd_vals
        node.right_base_pub.publish(tr)
        node.right_arm_pub.publish(ra)
    else:
        tl = Twist()
        tr = Twist()
        la = Float64MultiArray()
        ra = Float64MultiArray()
        tl.linear.x, tl.linear.y, tl.angular.z = left_base_cmd_vals
        tr.linear.x, tr.linear.y, tr.angular.z = right_base_cmd_vals
        la.data = left_arm_cmd_vals
        ra.data = right_arm_cmd_vals
        node.left_base_pub.publish(tl)
        node.right_base_pub.publish(tr)
        node.left_arm_pub.publish(la)
        node.right_arm_pub.publish(ra)

    if approach_converged:
        if side in ("left", "right"):
            node.stop_side_movement(side)
        else:
            node.stop_all_movement()
        node.clear_action_timer(timer_key)
        node.bb[near_key] = True
        if side == "left":
            node.bb["SRM1_base_goal_reached"] = True
        elif side == "right":
            node.bb["SRM2_base_goal_reached"] = True
        node.get_logger().info(
            bt_fmt(
                f"[ApproachObject] completed by convergence, {near_key}=True "
                f"(elapsed={elapsed:.2f}s)"
            )
        )
        return True

    if elapsed >= node.approach_duration:
        node._warn_throttled(
            f"{tree_name}_approach_not_converged",
            bt_fmt(
                "[ApproachObject] elapsed nominal duration without convergence "
                f"(elapsed={elapsed:.2f}s, arm_reached={arm_reached}, base_reached={base_reached})"
            ),
            period_s=2.0,
        )

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
            bt_fmt(
                f"[LiftObj] paused "
                f"(SIMOD_APPROACH_ONLY='{getattr(node, 'approach_only_raw', '1')}', "
                f"parsed={int(bool(node.approach_only_mode))})"
            ),
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
