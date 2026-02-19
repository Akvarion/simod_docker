#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import math
import os
from typing import List, Optional

from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates


def bt_fmt(message: str) -> str:
    return str(message)


class GazeboBridgeMixin:
    def _split_model_candidates(self, raw: str) -> List[str]:
        txt = str(raw or "").replace(";", ",")
        return [p.strip() for p in txt.split(",") if p and p.strip()]

    def _pick_model_index(self, model_names: List[str], raw_candidates: List[str]) -> Optional[int]:
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

    def _get_model_pose_xyz_by_candidates(self, raw_candidates, prefer_start: bool = True):
        """
        Cerca la posa xyz di un modello dai candidate names usando la cache da /model_states.
        Ritorna (xyz, model_name) oppure (None, None).
        """
        if isinstance(raw_candidates, str):
            candidates = self._split_model_candidates(raw_candidates)
        elif isinstance(raw_candidates, (list, tuple)):
            candidates = [str(c).strip() for c in raw_candidates if str(c).strip()]
        else:
            candidates = []
        if not candidates:
            return None, None

        start_map = getattr(self, "_gazebo_model_pose_start_xyz", None) or {}
        last_map = getattr(self, "_gazebo_model_pose_last_xyz", None) or {}

        def _try(src_map):
            if not src_map:
                return None, None
            names = list(src_map.keys())
            idx = self._pick_model_index(names, candidates)
            if idx is None or idx < 0 or idx >= len(names):
                return None, None
            name = names[idx]
            pose = src_map.get(name, None)
            if not isinstance(pose, (list, tuple)) or len(pose) < 3:
                return None, None
            return [float(pose[0]), float(pose[1]), float(pose[2])], str(name)

        if prefer_start:
            pose, name = _try(start_map)
            if pose is not None:
                return pose, name
            return _try(last_map)

        pose, name = _try(last_map)
        if pose is not None:
            return pose, name
        return _try(start_map)

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

    def _pick_side_ee_link_index(self, link_names: List[str], side: str) -> Optional[int]:
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

    def _pose6_from_gazebo_pose_msg(self, pose_msg) -> List[float]:
        p = pose_msg.position
        q = pose_msg.orientation
        roll, pitch, yaw = self._quat_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))
        return [float(p.x), float(p.y), float(p.z), float(roll), float(pitch), float(yaw)]

    def _model_states_cb(self, msg: ModelStates):
        try:
            names = list(getattr(msg, "name", []) or [])
            poses = list(getattr(msg, "pose", []) or [])
            if not names or len(names) != len(poses):
                return

            # Generic model cache for destination lookups (e.g. pacco_clone_2).
            if not hasattr(self, "_gazebo_model_pose_last_xyz"):
                self._gazebo_model_pose_last_xyz = {}
            if not hasattr(self, "_gazebo_model_pose_start_xyz"):
                self._gazebo_model_pose_start_xyz = {}
            for i, mname in enumerate(names):
                p = poses[i].position
                xyz = [float(p.x), float(p.y), float(p.z)]
                self._gazebo_model_pose_last_xyz[str(mname)] = xyz
                if str(mname) not in self._gazebo_model_pose_start_xyz:
                    self._gazebo_model_pose_start_xyz[str(mname)] = list(xyz)

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

    def _parse_pose_xy_text(self, pose_text: str) -> Optional[List[float]]:
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

    def _parse_pose_xyz_text(self, pose_text: str) -> Optional[List[float]]:
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
        env_path = str(getattr(self, "approach_mock_world_file", "") or "").strip()
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
            name_cands = ["pacco"]

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

    def _quat_to_rpy(self, qx, qy, qz, qw):
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

    def _odom_msg_to_pose6(self, msg: Odometry) -> List[float]:
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        pz = float(msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))
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
