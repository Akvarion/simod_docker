#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Package-centric hold helpers for BT motion actions."""

from __future__ import annotations

import numpy as np
import rclpy

from linkattacher_msgs.srv import AttachLink, DetachLink
from TaskPrioritization.Trajectories.trajectory import Trajectory

from bt_xml_demo.bt_action_context import (
    bt_fmt,
)
from bt_xml_demo.motion.hold_state import (
    _ensure_pkg_hold_state,
)
from bt_xml_demo.motion.io_timing import (
    _split_package_model_link,
)
from bt_xml_demo.motion.pose_targets import (
    _default_side_pkg_offset,
    _get_live_package_xyz,
    _resolve_pkg_reference_xyz,
)


def _capture_pkg_grasp_offsets(node, left_arm_jp, right_arm_jp, left_base, right_base):
    """Capture and store EE-to-package grasp offsets at moment of attachment."""
    _ensure_pkg_hold_state(node)
    pkg_xyz = _resolve_pkg_reference_xyz(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    if pkg_xyz is None:
        return False
    ee_live = node._get_live_ee_by_side(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )

    use_capture = bool(getattr(node.cfg.manipulation, "hold_use_captured_offsets", False))
    for side in ("left", "right"):
        ee = ee_live.get(side, None)
        if use_capture and isinstance(ee, np.ndarray) and ee.shape[0] >= 3:
            node._pkg_hold_offsets[side] = np.asarray(ee[:3], dtype=np.float32) - np.asarray(pkg_xyz[:3], dtype=np.float32)
            if ee.shape[0] >= 6:
                node._pkg_hold_rpy[side] = np.asarray(ee[3:6], dtype=np.float32)
        else:
            node._pkg_hold_offsets[side] = _default_side_pkg_offset(node, side)
            if isinstance(ee, np.ndarray) and ee.shape[0] >= 6:
                node._pkg_hold_rpy[side] = np.asarray(ee[3:6], dtype=np.float32)

    left_off = np.asarray(node._pkg_hold_offsets["left"], dtype=np.float32)
    right_off = np.asarray(node._pkg_hold_offsets["right"], dtype=np.float32)
    node._pkg_hold_nominal_dist = float(np.linalg.norm(left_off - right_off))
    node._pkg_hold_start_z = float(pkg_xyz[2])
    node._pkg_hold_target_z = float(pkg_xyz[2])
    node.get_logger().info(
        bt_fmt(
            "[PkgHold] captured grasp offsets "
            f"L={np.round(left_off, 3).tolist()}, R={np.round(right_off, 3).tolist()}, "
            f"nominal_dist={node._pkg_hold_nominal_dist:.3f}m"
        )
    )
    return True


def _pkg_hold_goal_pose(node, side: str, pkg_xyz, pkg_z_target=None):
    """Construct EE goal pose [xyz+rpy] for object-centric hold."""
    _ensure_pkg_hold_state(node)
    side = str(side).lower()
    off = node._pkg_hold_offsets.get(side, None)
    if off is None:
        off = _default_side_pkg_offset(node, side)
    off = np.asarray(off, dtype=np.float32)
    z_pkg = float(pkg_z_target) if pkg_z_target is not None else float(pkg_xyz[2])
    goal = np.zeros((6,), dtype=np.float32)
    goal[0] = float(pkg_xyz[0]) + float(off[0])
    goal[1] = float(pkg_xyz[1]) + float(off[1])
    goal[2] = z_pkg + float(off[2])

    man_cfg = node.cfg.manipulation
    ori_mode = str(getattr(man_cfg, "hold_orientation_mode", "captured")).strip().lower()
    if ori_mode == "fixed" and str(node.approach_ee_orient_mode).lower() == "fixed":
        rpy_goal = node.approach_ee_left_rpy_goal if side == "left" else node.approach_ee_right_rpy_goal
        if rpy_goal is not None and len(rpy_goal) == 3:
            goal[3] = float(rpy_goal[0])
            goal[4] = float(rpy_goal[1])
            goal[5] = float(rpy_goal[2])
            return goal

    rpy_live = node._pkg_hold_rpy.get(side, None)
    if isinstance(rpy_live, np.ndarray) and rpy_live.shape[0] >= 3:
        goal[3] = float(rpy_live[0])
        goal[4] = float(rpy_live[1])
        goal[5] = float(rpy_live[2])
    return goal


def _replan_pkg_hold_tp(
    node,
    left_arm_jp,
    right_arm_jp,
    left_base,
    right_base,
    pkg_xy_target=None,
    pkg_z_target=None,
    force=False,
    preserve_jtc_base=False,
):
    """Replan object-centric hold trajectories for package transport."""
    _ensure_pkg_hold_state(node)
    man_cfg = node.cfg.manipulation
    now = node.get_clock().now().nanoseconds / 1e9
    if (not force) and (now - float(node._pkg_hold_last_replan)) < max(float(man_cfg.hold_replan_period), 1e-3):
        return True

    pkg_xyz = _resolve_pkg_reference_xyz(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    if pkg_xyz is None:
        pkg_xyz = _get_live_package_xyz(node)
    if pkg_xyz is None:
        return False
    pkg_xyz = [float(pkg_xyz[0]), float(pkg_xyz[1]), float(pkg_xyz[2])]
    if isinstance(pkg_xy_target, (list, tuple)) and len(pkg_xy_target) >= 2:
        pkg_xyz[0] = float(pkg_xy_target[0])
        pkg_xyz[1] = float(pkg_xy_target[1])

    if node._pkg_hold_offsets.get("left", None) is None or node._pkg_hold_offsets.get("right", None) is None:
        if not _capture_pkg_grasp_offsets(node, left_arm_jp, right_arm_jp, left_base, right_base):
            return False

    ee_live = node._get_live_ee_by_side(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    left_ee = ee_live.get("left", None)
    right_ee = ee_live.get("right", None)
    if left_ee is None or right_ee is None:
        return False

    left_goal = _pkg_hold_goal_pose(node, "left", pkg_xyz, pkg_z_target=pkg_z_target)
    right_goal = _pkg_hold_goal_pose(node, "right", pkg_xyz, pkg_z_target=pkg_z_target)

    tr_left = Trajectory()
    tr_right = Trajectory()
    tr_left.poly5(
        p_i=np.asarray(left_ee, dtype=np.float32),
        p_f=np.asarray(left_goal, dtype=np.float32),
        period=float(max(man_cfg.hold_traj_time, 0.2)),
    )
    tr_right.poly5(
        p_i=np.asarray(right_ee, dtype=np.float32),
        p_f=np.asarray(right_goal, dtype=np.float32),
        period=float(max(man_cfg.hold_traj_time, 0.2)),
    )

    if node.approach_jtc_task is not None and (not bool(preserve_jtc_base)):
        try:
            node.approach_jtc_task.activate()
            node.approach_jtc_task.set_activation("base", False)
            node.approach_jtc_task.set_activation("arm", False)
        except Exception:
            pass

    if node.ee_task is not None:
        try:
            node.ee_task.activate()
        except Exception:
            pass
        if hasattr(node.ee_task, "set_use_base"):
            node.ee_task.set_use_base(False)
        else:
            node.ee_task.use_base = False
        node.ee_task.set_trajectory([tr_left, tr_right])

    node._pkg_hold_last_replan = now
    return True


def _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, label: str):
    """Log package hold quality metrics: EE distance, z-gap, nominal offset errors."""
    _ensure_pkg_hold_state(node)
    man_cfg = node.cfg.manipulation
    ee_live = node._get_live_ee_by_side(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    left_ee = ee_live.get("left", None)
    right_ee = ee_live.get("right", None)
    if left_ee is None or right_ee is None:
        return
    dist = float(np.linalg.norm(np.asarray(left_ee[:3], dtype=np.float32) - np.asarray(right_ee[:3], dtype=np.float32)))
    nominal = float(node._pkg_hold_nominal_dist) if np.isfinite(node._pkg_hold_nominal_dist) else dist
    dist_err = abs(dist - nominal)
    z_gap = abs(float(left_ee[2]) - float(right_ee[2]))
    ok = bool(z_gap <= float(man_cfg.hold_z_tol) and dist_err <= float(man_cfg.hold_dist_tol))
    node.bb["pkg_hold_ok"] = ok
    node.bb["pkg_hold_dist_err"] = float(dist_err)
    node.bb["pkg_hold_z_gap"] = float(z_gap)
    node._info_throttled(
        f"pkg_hold_{label}",
        bt_fmt(
            f"[PkgHold:{label}] ok={ok}, dist={dist:.3f}m, nominal={nominal:.3f}m, "
            f"dist_err={dist_err:.3f}m, z_gap={z_gap:.3f}m"
        ),
        period_s=float(max(man_cfg.hold_log_period, 0.2)),
    )


def _get_robot_model_name_for_side(node, side: str) -> str:
    """Robust resolution of Gazebo model name for left/right side."""
    side = str(side).lower()
    try:
        model = (node._gazebo_base_pose_start_model or {}).get(side, None)
    except Exception:
        model = None
    if isinstance(model, str) and model.strip():
        return model.strip()
    return "left_robot" if side == "left" else "right_robot"


def _get_arm_attach_link_name(side: str) -> str:
    """Side-specific attach link for link-attacher."""
    return "ur_left_wrist_3_link" if str(side).lower() == "left" else "ur_right_wrist_3_link"


def _attach_package_to_arms(node) -> bool:
    """Request package attachment via link-attacher service."""
    if not node.attach_cli.service_is_ready():
        try:
            node.attach_cli.wait_for_service(timeout_sec=1.0)
        except Exception:
            pass
    if not node.attach_cli.service_is_ready():
        return False

    pkg_model, pkg_link = _split_package_model_link(node)
    mode = str(getattr(node.cfg.manipulation, "attach_mode", "single_left")).strip().lower()
    if mode not in ("single_left", "single_right", "dual"):
        mode = "single_left"
    if mode == "dual":
        sides = ("left", "right")
    elif mode == "single_right":
        sides = ("right",)
    else:
        sides = ("left",)
    sent = False
    for side in sides:
        req = AttachLink.Request()
        setattr(req, "model1_name", _get_robot_model_name_for_side(node, side))
        setattr(req, "link1_name", _get_arm_attach_link_name(side))
        setattr(req, "model2_name", pkg_model)
        setattr(req, "link2_name", pkg_link)
        node.attach_cli.call_async(req)
        sent = True
    if sent:
        node.get_logger().info(
            bt_fmt(
                f"[PkgAttach] attach requests sent mode={mode} for model={pkg_model}, link={pkg_link}"
            )
        )
    return bool(sent)


def _detach_package_from_arms(node) -> bool:
    """Request package detachment and wait for service confirmation."""
    if not node.detach_cli.service_is_ready():
        try:
            node.detach_cli.wait_for_service(timeout_sec=1.0)
        except Exception:
            pass
    if not node.detach_cli.service_is_ready():
        return False

    pkg_model, pkg_link = _split_package_model_link(node)
    mode = str(getattr(node.cfg.manipulation, "attach_mode", "single_left")).strip().lower()
    if mode not in ("single_left", "single_right", "dual"):
        mode = "single_left"
    if mode == "dual":
        sides = ("left", "right")
    elif mode == "single_right":
        sides = ("right",)
    else:
        sides = ("left",)
    all_ok = True
    for side in sides:
        req = DetachLink.Request()
        setattr(req, "model1_name", _get_robot_model_name_for_side(node, side))
        setattr(req, "link1_name", _get_arm_attach_link_name(side))
        setattr(req, "model2_name", pkg_model)
        setattr(req, "link2_name", pkg_link)
        future = node.detach_cli.call_async(req)
        try:
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        except Exception:
            pass
        if future.done() and future.result() is not None:
            ok = getattr(future.result(), "success", False)
            if not ok:
                node.get_logger().warn(
                    bt_fmt(f"[PkgAttach] detach failed for side={side}: "
                           f"{getattr(future.result(), 'message', '')}")
                )
                all_ok = False
        else:
            node.get_logger().warn(
                bt_fmt(f"[PkgAttach] detach timeout for side={side}")
            )
            all_ok = False
    node.get_logger().info(
        bt_fmt(
            f"[PkgAttach] detach requests sent mode={mode} for model={pkg_model}, link={pkg_link}"
        )
    )
    return all_ok


def _log_force_proxy(node, label: str, period_s: float = 1.0):
    """Log joint efforts as force proxy (RMS and MAX per side)."""
    left_eff = node.get_arm_joint_efforts("left")
    right_eff = node.get_arm_joint_efforts("right")
    if left_eff is None and right_eff is None:
        node._info_throttled(
            f"force_proxy_{label}_missing",
            bt_fmt(f"[ForceProxy:{label}] no joint effort data available on joint_states"),
            period_s=max(float(period_s), 5.0),
        )
        return

    def _eff_metrics(eff):
        if eff is None or len(eff) == 0:
            return float("nan"), float("nan")
        arr = np.asarray(eff, dtype=np.float32)
        return float(np.sqrt(np.mean(arr * arr))), float(np.max(np.abs(arr)))

    l_rms, l_max = _eff_metrics(left_eff)
    r_rms, r_max = _eff_metrics(right_eff)
    node.bb["force_proxy_left_rms"] = l_rms
    node.bb["force_proxy_right_rms"] = r_rms
    node.bb["force_proxy_left_max"] = l_max
    node.bb["force_proxy_right_max"] = r_max
    node._info_throttled(
        f"force_proxy_{label}",
        bt_fmt(
            f"[ForceProxy:{label}] "
            f"L_rms={l_rms:.3f}, L_max={l_max:.3f}, "
            f"R_rms={r_rms:.3f}, R_max={r_max:.3f}"
        ),
        period_s=float(max(period_s, 0.2)),
    )


__all__ = [
    "_ensure_pkg_hold_state",
    "_capture_pkg_grasp_offsets",
    "_pkg_hold_goal_pose",
    "_replan_pkg_hold_tp",
    "_log_pkg_hold_quality",
    "_get_robot_model_name_for_side",
    "_get_arm_attach_link_name",
    "_attach_package_to_arms",
    "_detach_package_from_arms",
    "_log_force_proxy",
]
