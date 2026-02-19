#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

from bt_xml_demo.cmd_utils import cmd_slice as _cmd_slice
from sensor_msgs.msg import JointState


class TPControlMixin:
    def _infer_side_from_text(self, text: str) -> Optional[str]:
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

    def get_arm_joint_efforts(self, side: str):
        """
        Return ordered joint efforts (same joint order as get_arm_joint_positions)
        or None if effort is unavailable.
        """
        joint_states = self._last_joint_states.get(side)
        if joint_states is None:
            joint_states = self._last_joint_states.get("global")
            if joint_states is not None:
                self._warn_throttled(
                    f"{side}_effort_global_fallback",
                    f"[{side}] using /joint_states fallback for effort (side topic missing)"
                )
        if joint_states is None:
            return None

        expected_names = self._expected_arm_joint_names.get(side, [])
        if not hasattr(joint_states, "effort") or joint_states.effort is None or len(joint_states.effort) == 0:
            return None

        if not expected_names:
            if len(joint_states.effort) >= 6:
                return [float(v) for v in joint_states.effort[:6]]
            return None

        if len(joint_states.name) != len(joint_states.effort):
            self._warn_throttled(
                f"{side}_effort_name_len_mismatch",
                f"[{side}] joint_states name/effort length mismatch: {len(joint_states.name)} != {len(joint_states.effort)}"
            )
            return None

        name_to_eff = {name: eff for name, eff in zip(joint_states.name, joint_states.effort)}
        norm_to_eff = {
            self._normalize_joint_name(name): eff
            for name, eff in zip(joint_states.name, joint_states.effort)
        }

        ordered = []
        for exp_name in expected_names:
            if exp_name in name_to_eff:
                ordered.append(float(name_to_eff[exp_name]))
                continue
            norm_name = self._normalize_joint_name(exp_name)
            if norm_name in norm_to_eff:
                ordered.append(float(norm_to_eff[norm_name]))
                continue
            return None

        return ordered
