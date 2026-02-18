#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import math
import os
import json
from typing import Dict, List, Optional

import numpy as np

from TaskPrioritization.Trajectories.trajectory import Trajectory

from bt_xml_demo.cmd_utils import sanitize_base_cmd as _sanitize_base_cmd


def bt_fmt(message: str) -> str:
    return str(message)


class ApproachPlanningMixin:
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
                    q_left_goal = self._build_profile_goal(np.asarray(left_arm_jp, dtype=np.float32), self.left_arm_vel_profile)
                    left_from_profile = True
                    self._approach_arm_goal_source["left"] = "live_profile"
                else:
                    q_left_goal = np.asarray(q_left_goal, dtype=np.float32)
                if q_right_goal is None or len(q_right_goal) != len(right_arm_jp):
                    q_right_goal = self._build_profile_goal(np.asarray(right_arm_jp, dtype=np.float32), self.right_arm_vel_profile)
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
            left_home_goal, left_home_src = self._resolve_side_home_joint_goal("left", left_arm_jp)
            right_home_goal, right_home_src = self._resolve_side_home_joint_goal("right", right_arm_jp)
            self._approach_home_goal_joints["left"] = np.asarray(left_home_goal, dtype=np.float32)
            self._approach_home_goal_joints["right"] = np.asarray(right_home_goal, dtype=np.float32)
            self._approach_home_goal_source["left"] = str(left_home_src)
            self._approach_home_goal_source["right"] = str(right_home_src)
            delay_arm_until_close = bool(mode == "hybrid" and self.approach_arm_delay_enable and self.approach_use_base)
            self._approach_arm_delay_active = bool(delay_arm_until_close)
            self._approach_arm_motion_enabled = not bool(delay_arm_until_close)
            self._approach_arm_enable_ts = None
            self._approach_gate_dist = float("inf")

            if mode in ("joint", "hybrid") and self.approach_jtc_task is not None:
                jtc_arm_active = bool(mode == "joint" or delay_arm_until_close)
                jtc_arm_targets = current_joint_targets
                jtc_arm_period = float(self.approach_traj_time)
                if mode == "joint":
                    jtc_arm_targets = [
                        np.asarray(q_left_goal, dtype=np.float32),
                        np.asarray(q_right_goal, dtype=np.float32),
                    ]
                elif delay_arm_until_close:
                    jtc_arm_targets = [
                        np.asarray(left_home_goal, dtype=np.float32),
                        np.asarray(right_home_goal, dtype=np.float32),
                    ]
                    jtc_arm_period = float(self.approach_arm_home_traj_time)
                try:
                    self.approach_jtc_task.activate()
                    self.approach_jtc_task.set_activation("base", bool(self.approach_use_base))
                    self.approach_jtc_task.set_activation("arm", bool(jtc_arm_active))
                except Exception:
                    pass
                self.approach_jtc_task.set_omni_trajectories_pnts(
                    target_cart_positions=base_targets,
                    K_omni=k_omni,
                    period=float(self.approach_traj_time),
                )
                self.approach_jtc_task.set_arm_trajectories_pnts(
                    target_joint_positions=jtc_arm_targets,
                    K_arm=k_arm,
                    period=float(jtc_arm_period),
                )
                pallet_xy_log = [float(pallet_xy[0]), float(pallet_xy[1])] if pallet_xy is not None else None
                self.get_logger().info(
                    f"Approach TP base targets ({mode}-mode): "
                    f"left=[x={base_targets[0][0]:.3f}, y={base_targets[0][1]:.3f}, yaw={base_targets[0][2]:.3f}], "
                    f"right=[x={base_targets[1][0]:.3f}, y={base_targets[1][1]:.3f}, yaw={base_targets[1][2]:.3f}], "
                    f"pallet={pallet_xy_log}, slot_assign={self._approach_slot_assignment}, kp_yaw={base_kp_yaw:.2f}, "
                    f"jtc_arm_active={int(jtc_arm_active)}, arm_delay_active={int(delay_arm_until_close)}"
                )
                if delay_arm_until_close:
                    self.get_logger().info(
                        "Approach arm pre-phase: holding home joints until base is near target "
                        f"(dist<= {self.approach_arm_enable_dist:.2f}m), "
                        f"home_source=L:{left_home_src}/R:{right_home_src}, "
                        f"home_period={self.approach_arm_home_traj_time:.2f}s"
                    )
                    self.get_logger().info(
                        "Approach arm home goals (joint): "
                        f"left={np.round(np.asarray(left_home_goal, dtype=np.float32), 3).tolist()}, "
                        f"right={np.round(np.asarray(right_home_goal, dtype=np.float32), 3).tolist()}"
                    )
            else:
                try:
                    if self.approach_jtc_task is not None:
                        self.approach_jtc_task.deactivate()
                except Exception:
                    pass

            if mode in ("ee", "hybrid"):
                if delay_arm_until_close:
                    self.tr_left = None
                    self.tr_right = None
                    if self.ee_task is not None:
                        try:
                            self.ee_task.deactivate()
                        except Exception:
                            pass
                        self.ee_task.set_trajectory([None, None])
                else:
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
                f"arm_delay_active={int(self._approach_arm_delay_active)}, "
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

    def _ensure_mock_pallet_pose(
        self,
        left_base: Optional[List[float]] = None,
        right_base: Optional[List[float]] = None,
    ) -> Optional[List[float]]:
        """
        Simula il riconoscimento visivo pallet:
        - se presenti approach.mock_pallet_x/y usa quelli
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
                    source not in ("config", "env")
                    or not (isinstance(pose, (list, tuple)) and len(pose) >= 2)
                    or abs(float(pose[0]) - px) > 1e-6
                    or abs(float(pose[1]) - py) > 1e-6
                )
                if needs_update:
                    self.bb["pallet_pose_world"] = [px, py]
                    self.bb["pallet_pose_source"] = "config"
                    self.bb["pallet_info_ready"] = True
                    self.get_logger().info(
                        bt_fmt(f"[VisionMock] pallet pose from config: x={px:.3f}, y={py:.3f}")
                    )
                return [px, py]
            except Exception as exc:
                self._warn_throttled("vision_mock_config_invalid", f"[VisionMock] invalid config pallet pose: {exc}")

        # Default mock: usa la posa iniziale del modello pacco/pallet in Gazebo.
        if self._gazebo_pallet_pose_start_xy is not None and source not in ("config", "env"):
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
        if source not in ("config", "env"):
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
        if source in ("config", "env") and self.mock_pallet_z_env:
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
        if str(self.approach_ee_orient_mode).lower() == "fixed":
            rpy_goal = self.approach_ee_left_rpy_goal if side == "left" else self.approach_ee_right_rpy_goal
            if rpy_goal is not None and goal.shape[0] >= 6:
                goal[3] = float(rpy_goal[0])
                goal[4] = float(rpy_goal[1])
                goal[5] = float(rpy_goal[2])
        return goal

    def _resolve_side_home_joint_goal(self, side: str, live_arm_jp: List[float]) -> tuple[np.ndarray, str]:
        side = str(side).lower()
        live = np.asarray(live_arm_jp, dtype=np.float32)
        n = int(len(live))
        config_goal = self._approach_home_env_goal.get(side, None)
        if config_goal is not None and len(config_goal) == n:
            return np.asarray(config_goal, dtype=np.float32), "config"
        cfg_goal = self._approach_cfg_start_joints.get(side, None)
        if cfg_goal is not None and len(cfg_goal) == n:
            return np.asarray(cfg_goal, dtype=np.float32), "config"
        return live, "live"

    def _enable_approach_arm_motion_from_current_state(
        self,
        left_arm_jp: List[float],
        right_arm_jp: List[float],
        left_base: List[float],
        right_base: List[float],
    ) -> bool:
        if bool(self._approach_arm_motion_enabled):
            return True

        left_goal_ee = self._approach_goal_ee.get("left", None)
        right_goal_ee = self._approach_goal_ee.get("right", None)
        if left_goal_ee is None or right_goal_ee is None:
            self._warn_throttled(
                "approach_arm_enable_missing_goal",
                "Cannot enable arm motion: EE goals are not available.",
                period_s=2.0,
            )
            return False

        try:
            joint_pos_live, base_odom_live = self._build_tp_inputs_from_side_data(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            )
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
            left_goal = np.asarray(left_goal_ee, dtype=np.float32)
            right_goal = np.asarray(right_goal_ee, dtype=np.float32)

            self.tr_left = Trajectory()
            self.tr_right = Trajectory()
            self.tr_left.poly5(
                p_i=left_ee,
                p_f=left_goal,
                period=float(self.approach_arm_enable_traj_time),
            )
            self.tr_right.poly5(
                p_i=right_ee,
                p_f=right_goal,
                period=float(self.approach_arm_enable_traj_time),
            )
            if self.ee_task is not None:
                try:
                    self.ee_task.activate()
                except Exception:
                    pass
                ee_use_base = bool(self.approach_use_base and str(self.approach_task_mode or "").lower() == "ee")
                if hasattr(self.ee_task, "set_use_base"):
                    self.ee_task.set_use_base(ee_use_base)
                else:
                    self.ee_task.use_base = ee_use_base
                self.ee_task.set_trajectory([self.tr_left, self.tr_right])

            if self.approach_jtc_task is not None:
                try:
                    self.approach_jtc_task.activate()
                    self.approach_jtc_task.set_activation("base", bool(self.approach_use_base))
                    self.approach_jtc_task.set_activation("arm", False)
                except Exception:
                    pass

            self._approach_arm_motion_enabled = True
            self._approach_arm_delay_active = False
            self._approach_arm_enable_ts = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                "Approach arm motion enabled: "
                f"traj={self.approach_arm_enable_traj_time:.2f}s, "
                f"left_now={np.round(left_ee[:3], 3).tolist()} -> left_goal={np.round(left_goal[:3], 3).tolist()}, "
                f"right_now={np.round(right_ee[:3], 3).tolist()} -> right_goal={np.round(right_goal[:3], 3).tolist()}"
            )
            return True
        except Exception as exc:
            self._warn_throttled(
                "approach_arm_enable_failed",
                f"Unable to enable arm motion: {exc}",
                period_s=2.0,
            )
            return False

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

    def _angle_diff(self, a: float, b: float) -> float:
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
