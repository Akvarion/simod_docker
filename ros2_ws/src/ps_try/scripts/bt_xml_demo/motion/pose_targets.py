#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Pose/target resolution helpers for BT motion actions."""

from __future__ import annotations

import math

import numpy as np

from bt_xml_demo.bt_action_context import (
    bt_fmt,
)
from bt_xml_demo.motion.hold_state import (
    _ensure_pkg_hold_state,
)


def _get_live_package_xyz(node):
    """Retrieve live package pose from available sources."""
    p = getattr(node, "_gazebo_pallet_pose_last_xyz", None)
    if isinstance(p, (list, tuple)) and len(p) >= 3:
        return [float(p[0]), float(p[1]), float(p[2])]
    p = getattr(node, "_gazebo_pallet_pose_start_xyz", None)
    if isinstance(p, (list, tuple)) and len(p) >= 3:
        return [float(p[0]), float(p[1]), float(p[2])]
    p = node.bb.get("pallet_pose_world_xyz", None)
    if isinstance(p, (list, tuple)) and len(p) >= 3:
        return [float(p[0]), float(p[1]), float(p[2])]
    p = node._ensure_mock_pallet_pose_xyz()
    if isinstance(p, (list, tuple)) and len(p) >= 3:
        return [float(p[0]), float(p[1]), float(p[2])]
    return None


def _normalize_yaw(yaw: float) -> float:
    y = float(yaw)
    return (y + math.pi) % (2.0 * math.pi) - math.pi


def _offset_xy_in_target_frame(target_xy, off_x: float, off_y: float, target_yaw: float) -> list[float]:
    c = math.cos(float(target_yaw))
    s = math.sin(float(target_yaw))
    dx = c * float(off_x) - s * float(off_y)
    dy = s * float(off_x) + c * float(off_y)
    return [float(target_xy[0]) + dx, float(target_xy[1]) + dy]


def _align_pair_vector_to_target_yaw(
    pair_dx: float,
    pair_dy: float,
    target_yaw: float,
    blend: float,
) -> tuple[float, float]:
    """Rotate the L->R base-pair direction toward target yaw preserving pair norm."""
    norm = float(math.hypot(float(pair_dx), float(pair_dy)))
    if norm <= 1e-6:
        return float(pair_dx), float(pair_dy)

    b = float(max(0.0, min(1.0, float(blend))))
    if b <= 1e-6:
        return float(pair_dx), float(pair_dy)

    yaw = float(_normalize_yaw(float(target_yaw)))
    des_dx = norm * math.cos(yaw)
    des_dy = norm * math.sin(yaw)

    out_dx = (1.0 - b) * float(pair_dx) + b * float(des_dx)
    out_dy = (1.0 - b) * float(pair_dy) + b * float(des_dy)

    out_norm = float(math.hypot(out_dx, out_dy))
    if out_norm > 1e-6:
        scale = norm / out_norm
        out_dx *= scale
        out_dy *= scale
    return float(out_dx), float(out_dy)


def _get_live_package_pose6(node):
    """Retrieve live package pose [x,y,z,roll,pitch,yaw] when available."""
    p = getattr(node, "_gazebo_pallet_pose_last_pose6", None)
    if isinstance(p, (list, tuple)) and len(p) >= 6:
        out = [float(v) for v in p[:6]]
        out[5] = _normalize_yaw(out[5])
        return out
    p = getattr(node, "_gazebo_pallet_pose_start_pose6", None)
    if isinstance(p, (list, tuple)) and len(p) >= 6:
        out = [float(v) for v in p[:6]]
        out[5] = _normalize_yaw(out[5])
        return out
    p = node.bb.get("pallet_pose_world_pose6", None)
    if isinstance(p, (list, tuple)) and len(p) >= 6:
        out = [float(v) for v in p[:6]]
        out[5] = _normalize_yaw(out[5])
        return out
    p = _get_live_package_xyz(node)
    if isinstance(p, (list, tuple)) and len(p) >= 3:
        return [float(p[0]), float(p[1]), float(p[2]), 0.0, 0.0, 0.0]
    return None


def _resolve_transport_destination_xy(node):
    """Determine transport destination (world-frame XY coordinate)."""
    man_cfg = node.cfg.manipulation
    raw = str(getattr(man_cfg, "transport_destination_model", "pacco_clone_2")).strip() or "pacco_clone_2"
    cands = node._split_model_candidates(raw) if hasattr(node, "_split_model_candidates") else [raw]
    if not cands:
        cands = ["pacco_clone_2"]

    dst_pose6 = None
    dst_model = None
    if hasattr(node, "_get_model_pose6_by_candidates"):
        try:
            dst_pose6, dst_model = node._get_model_pose6_by_candidates(cands, prefer_start=True)
        except Exception:
            dst_pose6, dst_model = None, None
    if isinstance(dst_pose6, (list, tuple)) and len(dst_pose6) >= 6:
        dst_yaw = _normalize_yaw(float(dst_pose6[5]))
        dst_xy = _offset_xy_in_target_frame(
            [float(dst_pose6[0]), float(dst_pose6[1])],
            float(getattr(man_cfg, "transport_destination_offset_x", 0.0)),
            float(getattr(man_cfg, "transport_destination_offset_y", -2.0)),
            dst_yaw,
        )
        return [float(dst_xy[0]), float(dst_xy[1])], f"model:{dst_model or cands[0]}"

    dst_xyz = None
    if hasattr(node, "_get_model_pose_xyz_by_candidates"):
        try:
            dst_xyz, dst_model = node._get_model_pose_xyz_by_candidates(cands, prefer_start=True)
        except Exception:
            dst_xyz, dst_model = None, None

    if isinstance(dst_xyz, (list, tuple)) and len(dst_xyz) >= 2:
        return [
            float(dst_xyz[0]) + float(getattr(man_cfg, "transport_destination_offset_x", 0.0)),
            float(dst_xyz[1]) + float(getattr(man_cfg, "transport_destination_offset_y", -2.0)),
        ], f"model:{dst_model or cands[0]}"

    pkg_xyz = _get_live_package_xyz(node)
    if isinstance(pkg_xyz, (list, tuple)) and len(pkg_xyz) >= 2:
        node._warn_throttled(
            "transport_dst_fallback_pkg",
            bt_fmt(
                f"[MoveBase] destination model not found ({cands}), "
                "fallback to current package XY"
            ),
            period_s=3.0,
        )
        return [float(pkg_xyz[0]), float(pkg_xyz[1])], "fallback:pkg"

    return None, "unavailable"


def _resolve_drop_target_xyz(node):
    """Determine drop/release target location (world-frame XYZ coordinate)."""
    man_cfg = node.cfg.manipulation
    raw = str(getattr(man_cfg, "drop_target_model", "")).strip()
    if not raw:
        raw = str(getattr(man_cfg, "transport_destination_model", "pacco_clone_2")).strip() or "pacco_clone_2"
    cands = node._split_model_candidates(raw) if hasattr(node, "_split_model_candidates") else [raw]
    if not cands:
        cands = ["pacco_clone_2"]

    dst_pose6 = None
    dst_model = None
    if hasattr(node, "_get_model_pose6_by_candidates"):
        try:
            dst_pose6, dst_model = node._get_model_pose6_by_candidates(cands, prefer_start=False)
            if dst_pose6 is None:
                dst_pose6, dst_model = node._get_model_pose6_by_candidates(cands, prefer_start=True)
        except Exception:
            dst_pose6, dst_model = None, None
    if isinstance(dst_pose6, (list, tuple)) and len(dst_pose6) >= 6:
        dst_yaw = _normalize_yaw(float(dst_pose6[5]))
        dst_xy = _offset_xy_in_target_frame(
            [float(dst_pose6[0]), float(dst_pose6[1])],
            float(getattr(man_cfg, "drop_target_offset_x", 0.0)),
            float(getattr(man_cfg, "drop_target_offset_y", 0.0)),
            dst_yaw,
        )
        out = [
            float(dst_xy[0]),
            float(dst_xy[1]),
            float(dst_pose6[2]) + float(getattr(man_cfg, "drop_target_offset_z", 0.0)),
        ]
        node.bb["drop_target_yaw"] = float(dst_yaw)
        node.bb["drop_target_pose6"] = [
            float(dst_pose6[0]),
            float(dst_pose6[1]),
            float(dst_pose6[2]),
            float(dst_pose6[3]),
            float(dst_pose6[4]),
            float(dst_yaw),
        ]
        return out, f"model:{dst_model or cands[0]}"

    dst_xyz = None
    if hasattr(node, "_get_model_pose_xyz_by_candidates"):
        try:
            dst_xyz, dst_model = node._get_model_pose_xyz_by_candidates(cands, prefer_start=False)
            if dst_xyz is None:
                dst_xyz, dst_model = node._get_model_pose_xyz_by_candidates(cands, prefer_start=True)
        except Exception:
            dst_xyz, dst_model = None, None

    if isinstance(dst_xyz, (list, tuple)) and len(dst_xyz) >= 3:
        node.bb["drop_target_yaw"] = 0.0
        node.bb["drop_target_pose6"] = [
            float(dst_xyz[0]) + float(getattr(man_cfg, "drop_target_offset_x", 0.0)),
            float(dst_xyz[1]) + float(getattr(man_cfg, "drop_target_offset_y", 0.0)),
            float(dst_xyz[2]) + float(getattr(man_cfg, "drop_target_offset_z", 0.0)),
            0.0,
            0.0,
            float(node.bb.get("drop_target_yaw", 0.0)),
        ]
        return node.bb["drop_target_pose6"][:3], f"model:{dst_model or cands[0]}"

    pkg_pose6 = _get_live_package_pose6(node)
    if isinstance(pkg_pose6, (list, tuple)) and len(pkg_pose6) >= 6:
        node.bb["drop_target_yaw"] = float(_normalize_yaw(float(pkg_pose6[5])))
        node.bb["drop_target_pose6"] = [
            float(pkg_pose6[0]),
            float(pkg_pose6[1]),
            float(pkg_pose6[2]),
            float(pkg_pose6[3]),
            float(pkg_pose6[4]),
            float(pkg_pose6[5]),
        ]
        node._warn_throttled(
            "drop_target_fallback_pkg",
            bt_fmt(f"[Drop] target model not found ({cands}), fallback to current package pose"),
            period_s=3.0,
        )
        return [float(pkg_pose6[0]), float(pkg_pose6[1]), float(pkg_pose6[2])], "fallback:pkg"

    return None, "unavailable"


def _get_live_tp_state(node):
    """Retrieve current live system state for TP input."""
    left_arm_jp = node.get_arm_joint_positions("left")
    right_arm_jp = node.get_arm_joint_positions("right")
    left_base = node.get_base_pose("left")
    right_base = node.get_base_pose("right")
    if None in [left_arm_jp, right_arm_jp, left_base, right_base]:
        return None
    return left_arm_jp, right_arm_jp, left_base, right_base


def _default_side_pkg_offset(node, side: str):
    """Nominal EE-package offset by side, read from manipulation hold params."""
    side = str(side).lower()
    man_cfg = node.cfg.manipulation
    if side == "left":
        x = float(man_cfg.hold_left_offset_x)
        y = float(man_cfg.hold_left_offset_y)
        z = float(man_cfg.hold_left_offset_z)
    else:
        x = float(man_cfg.hold_right_offset_x)
        y = float(man_cfg.hold_right_offset_y)
        z = float(man_cfg.hold_right_offset_z)
    return np.asarray([x, y, z], dtype=np.float32)


def _resolve_hold_reference_mode(node) -> str:
    """Select reference for package estimation during hold."""
    man_cfg = node.cfg.manipulation
    ref = str(getattr(man_cfg, "hold_reference", "auto")).strip().lower()
    if ref not in ("auto", "package", "left_ee", "right_ee"):
        ref = "auto"
    if ref != "auto":
        return ref
    mode = str(getattr(man_cfg, "attach_mode", "single_left")).strip().lower()
    if mode == "single_right":
        return "right_ee"
    if mode == "dual":
        return "package"
    return "left_ee"


def _resolve_pkg_reference_xyz(node, left_arm_jp, right_arm_jp, left_base, right_base):
    """Estimate package pose from EE position minus grasp offset."""
    ref = _resolve_hold_reference_mode(node)
    if ref == "package":
        return _get_live_package_xyz(node)

    side = "left" if ref == "left_ee" else "right"
    ee_live = node._get_live_ee_by_side(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    ee = ee_live.get(side, None)
    if ee is None or len(ee) < 3:
        return _get_live_package_xyz(node)

    _ensure_pkg_hold_state(node)
    off = node._pkg_hold_offsets.get(side, None)
    if off is None:
        off = _default_side_pkg_offset(node, side)
    off = np.asarray(off, dtype=np.float32)
    pkg = np.asarray(ee[:3], dtype=np.float32) - np.asarray(off[:3], dtype=np.float32)
    return [float(pkg[0]), float(pkg[1]), float(pkg[2])]


def _pkg_xyz_for_alignment(node, left_arm_jp, right_arm_jp, left_base, right_base):
    """Select preferred package pose for base alignment (drop/placement)."""
    pkg_live = _get_live_package_xyz(node)
    pkg_ref = _resolve_pkg_reference_xyz(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    if bool(node.bb.get("package_attached", False)):
        return pkg_ref if pkg_ref is not None else pkg_live
    return pkg_live if pkg_live is not None else pkg_ref


def _compute_rigid_pkg_base_targets(
    node,
    man_cfg,
    left_base,
    right_base,
    pkg_xy,
    drop_target_xy,
    pair_xy=None,
):
    """Compute rigid-package base targets with optional yaw alignment."""
    dx = float(drop_target_xy[0]) - float(pkg_xy[0])
    dy = float(drop_target_xy[1]) - float(pkg_xy[1])

    if isinstance(pair_xy, (list, tuple)) and len(pair_xy) >= 2:
        pair_dx = float(pair_xy[0])
        pair_dy = float(pair_xy[1])
    else:
        pair_dx = float(right_base[0]) - float(left_base[0])
        pair_dy = float(right_base[1]) - float(left_base[1])

    if bool(getattr(man_cfg, "drop_rigid_pkg_align_yaw", True)):
        drop_target_yaw = float(_normalize_yaw(float(node.bb.get("drop_target_yaw", 0.0))))
        pair_dx, pair_dy = _align_pair_vector_to_target_yaw(
            pair_dx,
            pair_dy,
            drop_target_yaw,
            float(getattr(man_cfg, "drop_rigid_pkg_yaw_blend", 1.0)),
        )

    center_x = 0.5 * (float(left_base[0]) + float(right_base[0])) + dx
    center_y = 0.5 * (float(left_base[1]) + float(right_base[1])) + dy
    left_target_xy = [center_x - 0.5 * pair_dx, center_y - 0.5 * pair_dy]
    right_target_xy = [center_x + 0.5 * pair_dx, center_y + 0.5 * pair_dy]
    pair_out = [float(pair_dx), float(pair_dy)]
    return left_target_xy, right_target_xy, pair_out, float(dx), float(dy)


__all__ = [
    "_get_live_package_xyz",
    "_normalize_yaw",
    "_offset_xy_in_target_frame",
    "_align_pair_vector_to_target_yaw",
    "_get_live_package_pose6",
    "_resolve_transport_destination_xy",
    "_resolve_drop_target_xyz",
    "_get_live_tp_state",
    "_default_side_pkg_offset",
    "_resolve_hold_reference_mode",
    "_resolve_pkg_reference_xyz",
    "_pkg_xyz_for_alignment",
    "_compute_rigid_pkg_base_targets",
]
