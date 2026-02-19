#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Motion BT actions (approach/lift/transport/drop/release)."""

from __future__ import annotations

import math
import sys

import numpy as np
import rclpy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from linkattacher_msgs.srv import AttachLink, DetachLink
from TaskPrioritization.Trajectories.trajectory import Trajectory

from bt_xml_demo.core import (
    set_package_gravity,
)
from bt_xml_demo.bt_action_context import (
    bt_fmt,
    get_current_bt_name,
    require_node,
)
from bt_xml_demo.cmd_utils import (
    sanitize_arm_cmd as _sanitize_arm_cmd,
    sanitize_base_cmd as _sanitize_base_cmd,
)

# Backward-compatible alias used by extracted action bodies.
_require_node = require_node


def _float_vec(values):
    return [float(v) for v in values]


def _ros_now_s(node) -> float:
    return float(node.get_clock().now().nanoseconds) / 1e9


def _dt_from_ros_time(now_s: float, last_s: float, default_dt: float = 1.0 / 30.0) -> float:
    if last_s is None:
        return float(default_dt)
    dt = float(now_s) - float(last_s)
    # Robustezza su reset/salti del clock simulato.
    if (not np.isfinite(dt)) or dt <= 0.0:
        return float(default_dt)
    return float(dt)


def _publish_base_cmd(node, left_cmd=None, right_cmd=None):
    if left_cmd is not None:
        tl = Twist()
        tl.linear.x, tl.linear.y, tl.angular.z = left_cmd
        node.left_base_pub.publish(tl)
    if right_cmd is not None:
        tr = Twist()
        tr.linear.x, tr.linear.y, tr.angular.z = right_cmd
        node.right_base_pub.publish(tr)


def _publish_arm_cmd(node, left_cmd=None, right_cmd=None):
    if left_cmd is not None:
        la = Float64MultiArray()
        la.data = left_cmd
        node.left_arm_pub.publish(la)
    if right_cmd is not None:
        ra = Float64MultiArray()
        ra.data = right_cmd
        node.right_arm_pub.publish(ra)


def _publish_base_xy(node, left_xy=None, right_xy=None):
    left_cmd = [left_xy[0], left_xy[1], 0.0] if left_xy is not None else None
    right_cmd = [right_xy[0], right_xy[1], 0.0] if right_xy is not None else None
    _publish_base_cmd(node, left_cmd=left_cmd, right_cmd=right_cmd)


def _scaled_xy(xy, scale: float):
    if xy is None:
        return None
    s = float(scale)
    return [float(xy[0]) * s, float(xy[1]) * s]


def _predict_world_target_from_body_velocity(base_pose, vel_xy, duration_s: float):
    """
    Converte una velocita' (vx,vy) in frame base in un target world dopo duration_s.
    Utile per migrare profili open-loop legacy verso target TP point-to-point.
    """
    yaw = float(base_pose[5])
    vx_b = float(vel_xy[0])
    vy_b = float(vel_xy[1])
    dt = float(max(duration_s, 0.0))
    dx_w = (math.cos(yaw) * vx_b - math.sin(yaw) * vy_b) * dt
    dy_w = (math.sin(yaw) * vx_b + math.cos(yaw) * vy_b) * dt
    return [float(base_pose[0]) + dx_w, float(base_pose[1]) + dy_w]


def _split_package_model_link(node):
    raw = str(getattr(node.cfg.package, "link_name", "")).strip()
    if "::" in raw:
        model = raw.split("::", 1)[0].strip()
        return model, raw
    model = str(getattr(node, "_gazebo_pallet_pose_start_model", "") or "").strip()
    if not model:
        model = "pacco_clone_1"
    return model, raw or "pacco_clone_1::link_1"


def _ensure_pkg_hold_state(node):
    if not hasattr(node, "_pkg_hold_offsets"):
        node._pkg_hold_offsets = {"left": None, "right": None}
    if not hasattr(node, "_pkg_hold_rpy"):
        node._pkg_hold_rpy = {"left": None, "right": None}
    if not hasattr(node, "_pkg_hold_nominal_dist"):
        node._pkg_hold_nominal_dist = float("nan")
    if not hasattr(node, "_pkg_hold_last_replan"):
        node._pkg_hold_last_replan = 0.0
    if not hasattr(node, "_pkg_hold_start_z"):
        node._pkg_hold_start_z = None
    if not hasattr(node, "_pkg_hold_target_z"):
        node._pkg_hold_target_z = None
    if not hasattr(node, "_tp_manip_last_exec_monotonic"):
        node._tp_manip_last_exec_monotonic = None


def _get_live_package_xyz(node):
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


def _resolve_transport_destination_xy(node):
    """
    Destinazione pacco in world per il trasporto.
    Priorita':
    1) Gazebo model states (candidate model names)
    2) fallback su pacco corrente (se disponibile)
    """
    man_cfg = node.cfg.manipulation
    raw = str(getattr(man_cfg, "transport_destination_model", "pacco_clone_2")).strip() or "pacco_clone_2"
    cands = node._split_model_candidates(raw) if hasattr(node, "_split_model_candidates") else [raw]
    if not cands:
        cands = ["pacco_clone_2"]

    dst_xyz = None
    dst_model = None
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


def _get_live_tp_state(node):
    left_arm_jp = node.get_arm_joint_positions("left")
    right_arm_jp = node.get_arm_joint_positions("right")
    left_base = node.get_base_pose("left")
    right_base = node.get_base_pose("right")
    if None in [left_arm_jp, right_arm_jp, left_base, right_base]:
        return None
    return left_arm_jp, right_arm_jp, left_base, right_base


def _default_side_pkg_offset(node, side: str):
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
    """
    Restituisce la posa xyz del pacco usata come riferimento del hold:
    - da Gazebo (package)
    - oppure stimata da EE lato attach (left_ee/right_ee), per ridurre lag/strappi.
    """
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


def _build_base_cmd_to_xy(
    node,
    base_pose,
    target_xy,
    kp_x: float,
    kp_y: float,
    xy_abs_max: float,
    goal_tol: float | None = None,
):
    dx_w = float(target_xy[0]) - float(base_pose[0])
    dy_w = float(target_xy[1]) - float(base_pose[1])
    yaw = float(base_pose[5])
    ex_b = math.cos(yaw) * dx_w + math.sin(yaw) * dy_w
    ey_b = -math.sin(yaw) * dx_w + math.cos(yaw) * dy_w
    vx = float(kp_x) * ex_b
    vy = float(kp_y) * ey_b
    cmd = _sanitize_base_cmd(
        [vx, vy, 0.0],
        xy_abs_max=float(max(1e-4, xy_abs_max)),
        wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
        ramp=1.0,
    )
    if goal_tol is None:
        goal_tol = float(getattr(node.cfg.manipulation, "pre_transport_base_goal_tol", 0.10))
    reached = (dx_w * dx_w + dy_w * dy_w) <= float(max(1e-4, goal_tol)) ** 2
    return cmd, bool(reached), float(math.hypot(dx_w, dy_w))


def _capture_pkg_grasp_offsets(node, left_arm_jp, right_arm_jp, left_base, right_base):
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
    pkg_z_target=None,
    force=False,
    preserve_jtc_base=False,
):
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


def _execute_tp_arm_control(node, left_arm_jp, right_arm_jp, left_base, right_base, arm_clip_abs: float):
    joint_pos, base_odom = node._build_tp_inputs_from_side_data(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    now_ros_s = _ros_now_s(node)
    dt = _dt_from_ros_time(now_ros_s, node._tp_manip_last_exec_monotonic, default_dt=1.0 / 30.0)
    node._tp_manip_last_exec_monotonic = now_ros_s
    node.tp._delta_t = max(1e-3, min(float(dt), 0.2))
    cmd = node.tp.execute(joint_pos=joint_pos, base_odom=base_odom)
    clip = float(max(1e-4, arm_clip_abs))
    left_arm_cmd = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "left"), clip)
    right_arm_cmd = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "right"), clip)
    _publish_arm_cmd(node, left_cmd=left_arm_cmd, right_cmd=right_arm_cmd)
    return left_arm_cmd, right_arm_cmd


def _execute_tp_full_control(
    node,
    left_arm_jp,
    right_arm_jp,
    left_base,
    right_base,
    arm_clip_abs: float,
    base_xy_abs_max: float | None = None,
    base_wz_abs_max: float | None = None,
):
    """
    Esegue TP e pubblica sia comandi base che braccio.
    Usare per fasi "TP puro" senza profili open-loop.
    """
    joint_pos, base_odom = node._build_tp_inputs_from_side_data(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    now_ros_s = _ros_now_s(node)
    dt = _dt_from_ros_time(now_ros_s, node._tp_manip_last_exec_monotonic, default_dt=1.0 / 30.0)
    node._tp_manip_last_exec_monotonic = now_ros_s
    node.tp._delta_t = max(1e-3, min(float(dt), 0.2))
    cmd = node.tp.execute(joint_pos=joint_pos, base_odom=base_odom)

    arm_clip = float(max(1e-4, arm_clip_abs))
    xy_clip = float(node.tp_base_cmd_xy_abs_max if base_xy_abs_max is None else max(1e-4, base_xy_abs_max))
    wz_clip = float(node.tp_base_cmd_wz_abs_max if base_wz_abs_max is None else max(1e-4, base_wz_abs_max))

    left_base_cmd = _sanitize_base_cmd(node._get_base_cmd_values(cmd, "left"), xy_abs_max=xy_clip, wz_abs_max=wz_clip, ramp=1.0)
    right_base_cmd = _sanitize_base_cmd(node._get_base_cmd_values(cmd, "right"), xy_abs_max=xy_clip, wz_abs_max=wz_clip, ramp=1.0)
    left_arm_cmd = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "left"), arm_clip)
    right_arm_cmd = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "right"), arm_clip)

    _publish_base_cmd(node, left_cmd=left_base_cmd, right_cmd=right_base_cmd)
    _publish_arm_cmd(node, left_cmd=left_arm_cmd, right_cmd=right_arm_cmd)
    return left_base_cmd, right_base_cmd, left_arm_cmd, right_arm_cmd


def _execute_tp_arm_hold(node, left_arm_jp, right_arm_jp, left_base, right_base):
    hold_clip = float(getattr(node.cfg.manipulation, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max))
    return _execute_tp_arm_control(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
        arm_clip_abs=hold_clip,
    )


def _init_tp_base_stage(node, left_base_goal_xy, right_base_goal_xy, period_s: float, kp_xy: float = 1.0, kp_yaw: float = 0.0) -> bool:
    if node.approach_jtc_task is None:
        return False
    try:
        left_now = node.get_base_pose("left")
        right_now = node.get_base_pose("right")
        if left_now is None or right_now is None:
            return False
        l_goal = np.asarray([float(left_base_goal_xy[0]), float(left_base_goal_xy[1]), float(left_now[5])], dtype=np.float32)
        r_goal = np.asarray([float(right_base_goal_xy[0]), float(right_base_goal_xy[1]), float(right_now[5])], dtype=np.float32)
        k_omni = np.asarray([[float(kp_xy), float(kp_xy), float(kp_yaw)], [float(kp_xy), float(kp_xy), float(kp_yaw)]], dtype=np.float32)
        node.approach_jtc_task.activate()
        node.approach_jtc_task.set_activation("base", True)
        node.approach_jtc_task.set_activation("arm", False)
        node.approach_jtc_task.set_omni_trajectories_pnts(
            target_cart_positions=[l_goal, r_goal],
            K_omni=k_omni,
            period=float(max(period_s, 0.2)),
        )
        return True
    except Exception as exc:
        node._warn_throttled("tp_base_stage_init_fail", bt_fmt(f"[TP] base-stage init failed: {exc}"), period_s=2.0)
        return False


def _init_tp_arm_joint_stage(node, left_arm_goal, right_arm_goal, period_s: float, kp_arm: float | None = None) -> bool:
    if node.approach_jtc_task is None:
        return False
    try:
        ql = np.asarray(left_arm_goal, dtype=np.float32)
        qr = np.asarray(right_arm_goal, dtype=np.float32)
        ql, qr = _clip_descend_pick_joint_goals(node, ql, qr)
        k = float(node.approach_jtc_arm_kp if kp_arm is None else kp_arm)
        k_arm = np.full((len(ql) + len(qr),), k, dtype=np.float32)
        node.approach_jtc_task.activate()
        node.approach_jtc_task.set_activation("base", False)
        node.approach_jtc_task.set_activation("arm", True)
        node.approach_jtc_task.set_arm_trajectories_pnts(
            target_joint_positions=[ql, qr],
            K_arm=k_arm,
            period=float(max(period_s, 0.2)),
        )
        return True
    except Exception as exc:
        node._warn_throttled("tp_arm_stage_init_fail", bt_fmt(f"[TP] arm-stage init failed: {exc}"), period_s=2.0)
        return False


def _set_tp_ee_traj(node, left_ee_now, right_ee_now, left_ee_goal, right_ee_goal, traj_time: float) -> bool:
    tr_left = Trajectory()
    tr_right = Trajectory()
    period = float(max(traj_time, 0.2))
    tr_left.poly5(
        p_i=np.asarray(left_ee_now, dtype=np.float32),
        p_f=np.asarray(left_ee_goal, dtype=np.float32),
        period=period,
    )
    tr_right.poly5(
        p_i=np.asarray(right_ee_now, dtype=np.float32),
        p_f=np.asarray(right_ee_goal, dtype=np.float32),
        period=period,
    )

    if node.approach_jtc_task is not None:
        try:
            node.approach_jtc_task.activate()
            node.approach_jtc_task.set_activation("base", False)
            node.approach_jtc_task.set_activation("arm", False)
        except Exception:
            pass

    if node.ee_task is None:
        return False

    try:
        node.ee_task.activate()
    except Exception:
        pass
    if hasattr(node.ee_task, "set_use_base"):
        node.ee_task.set_use_base(False)
    else:
        node.ee_task.use_base = False
    node.ee_task.set_trajectory([tr_left, tr_right])
    node._tp_manip_last_exec_monotonic = None
    return True


def _pick_open_axis_idx(axis_name: str) -> int:
    key = str(axis_name).strip().lower()
    if key == "roll":
        return 0
    if key == "yaw":
        return 2
    return 1


def _ee_goal_reached(node, ee_live, ee_goal, pos_tol: float, ori_tol: float):
    if ee_live is None or ee_goal is None:
        return False, float("inf"), float("inf")
    if len(ee_live) < 6 or len(ee_goal) < 6:
        return False, float("inf"), float("inf")
    pos_err = float(np.linalg.norm(np.asarray(ee_live[:3], dtype=np.float32) - np.asarray(ee_goal[:3], dtype=np.float32)))
    ori_err = max(
        abs(float(node._angle_diff(float(ee_live[3]), float(ee_goal[3])))),
        abs(float(node._angle_diff(float(ee_live[4]), float(ee_goal[4])))),
        abs(float(node._angle_diff(float(ee_live[5]), float(ee_goal[5])))),
    )
    return bool(pos_err <= float(pos_tol) and ori_err <= float(ori_tol)), pos_err, ori_err


def _default_rpy_for_side(node, side: str, ee_live):
    if str(node.approach_ee_orient_mode).lower() == "fixed":
        rpy_goal = node.approach_ee_left_rpy_goal if side == "left" else node.approach_ee_right_rpy_goal
        if rpy_goal is not None and len(rpy_goal) == 3:
            return np.asarray(rpy_goal, dtype=np.float32)
    if isinstance(ee_live, np.ndarray) and ee_live.shape[0] >= 6:
        return np.asarray(ee_live[3:6], dtype=np.float32)
    return np.zeros((3,), dtype=np.float32)


def _build_pick_waypoint_stage_plan(node, left_arm_jp, right_arm_jp, left_base, right_base):
    man_cfg = node.cfg.manipulation
    pkg_xyz = _get_live_package_xyz(node)
    if pkg_xyz is None:
        return None

    ee_live = node._get_live_ee_by_side(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    left_ee = ee_live.get("left", None)
    right_ee = ee_live.get("right", None)

    left_rpy_closed = _default_rpy_for_side(node, "left", left_ee)
    right_rpy_closed = _default_rpy_for_side(node, "right", right_ee)

    axis_idx = _pick_open_axis_idx(getattr(man_cfg, "pick_open_axis", "pitch"))
    open_angle_left = float(getattr(man_cfg, "pick_open_angle_rad_left", math.pi / 2.0))
    open_angle_right = float(getattr(man_cfg, "pick_open_angle_rad_right", math.pi / 2.0))
    left_rpy_open = np.asarray(left_rpy_closed, dtype=np.float32).copy()
    right_rpy_open = np.asarray(right_rpy_closed, dtype=np.float32).copy()
    left_rpy_open[axis_idx] = float(left_rpy_open[axis_idx]) + open_angle_left
    right_rpy_open[axis_idx] = float(right_rpy_open[axis_idx]) + open_angle_right

    def _goal(x_off: float, y_off: float, z_off: float, rpy):
        g = np.zeros((6,), dtype=np.float32)
        g[0] = float(pkg_xyz[0]) + float(x_off)
        g[1] = float(pkg_xyz[1]) + float(y_off)
        g[2] = float(pkg_xyz[2]) + float(z_off)
        g[3:6] = np.asarray(rpy, dtype=np.float32)
        return g

    stages = [
        {
            "name": "start_open",
            "traj_time": float(max(getattr(man_cfg, "pick_start_traj_time", 2.5), 0.2)),
            "left_goal": _goal(
                float(getattr(man_cfg, "pick_start_left_offset_x", -0.40)),
                float(getattr(man_cfg, "pick_start_offset_y", 0.0)),
                float(getattr(man_cfg, "pick_start_offset_z", 0.40)),
                left_rpy_open,
            ),
            "right_goal": _goal(
                float(getattr(man_cfg, "pick_start_right_offset_x", 0.40)),
                float(getattr(man_cfg, "pick_start_offset_y", 0.0)),
                float(getattr(man_cfg, "pick_start_offset_z", 0.40)),
                right_rpy_open,
            ),
        },
        {
            "name": "mid_open",
            "traj_time": float(max(getattr(man_cfg, "pick_mid_traj_time", 2.0), 0.2)),
            "left_goal": _goal(
                float(getattr(man_cfg, "pick_mid_left_offset_x", -0.45)),
                float(getattr(man_cfg, "pick_mid_offset_y", 0.0)),
                float(getattr(man_cfg, "pick_mid_offset_z", 0.15)),
                left_rpy_open,
            ),
            "right_goal": _goal(
                float(getattr(man_cfg, "pick_mid_right_offset_x", 0.45)),
                float(getattr(man_cfg, "pick_mid_offset_y", 0.0)),
                float(getattr(man_cfg, "pick_mid_offset_z", 0.15)),
                right_rpy_open,
            ),
        },
        {
            "name": "grasp_close",
            "traj_time": float(max(getattr(man_cfg, "pick_grasp_traj_time", 2.0), 0.2)),
            "left_goal": _goal(
                float(getattr(man_cfg, "pick_grasp_left_offset_x", -0.35)),
                float(getattr(man_cfg, "pick_grasp_offset_y", 0.0)),
                float(getattr(man_cfg, "pick_grasp_offset_z", 0.0)),
                left_rpy_closed,
            ),
            "right_goal": _goal(
                float(getattr(man_cfg, "pick_grasp_right_offset_x", 0.35)),
                float(getattr(man_cfg, "pick_grasp_offset_y", 0.0)),
                float(getattr(man_cfg, "pick_grasp_offset_z", 0.0)),
                right_rpy_closed,
            ),
        },
    ]
    return stages


def _reset_pick_waypoint_runtime(node):
    node.bb.pop("lift_pick_waypoints", None)
    node.bb.pop("lift_pick_stage_idx", None)
    node.bb.pop("lift_pick_stage_active", None)
    node.bb.pop("lift_pick_stage_start_s", None)


def _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, label: str):
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
    side = str(side).lower()
    try:
        model = (node._gazebo_base_pose_start_model or {}).get(side, None)
    except Exception:
        model = None
    if isinstance(model, str) and model.strip():
        return model.strip()
    return "left_robot" if side == "left" else "right_robot"


def _get_arm_attach_link_name(side: str) -> str:
    return "ur_left_wrist_3_link" if str(side).lower() == "left" else "ur_right_wrist_3_link"


def _attach_package_to_arms(node) -> bool:
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
    sent = False
    for side in sides:
        req = DetachLink.Request()
        setattr(req, "model1_name", _get_robot_model_name_for_side(node, side))
        setattr(req, "link1_name", _get_arm_attach_link_name(side))
        setattr(req, "model2_name", pkg_model)
        setattr(req, "link2_name", pkg_link)
        node.detach_cli.call_async(req)
        sent = True
    if sent:
        node.get_logger().info(
            bt_fmt(
                f"[PkgAttach] detach requests sent mode={mode} for model={pkg_model}, link={pkg_link}"
            )
        )
    return bool(sent)


def _reset_pkg_hold_runtime(node):
    _ensure_pkg_hold_state(node)
    node._pkg_hold_offsets = {"left": None, "right": None}
    node._pkg_hold_rpy = {"left": None, "right": None}
    node._pkg_hold_nominal_dist = float("nan")
    node._pkg_hold_last_replan = 0.0
    node._pkg_hold_start_z = None
    node._pkg_hold_target_z = None
    node._tp_manip_last_exec_monotonic = None


def _phase_pause_key(phase_name: str) -> str:
    return f"phase_pause_approved::{str(phase_name)}"


def _phase_pause_gate(node, phase_name: str) -> bool:
    """
    Se abilitato in config, mette in pausa prima della fase e richiede conferma
    utente da stdin (solo su tree Supervisor per evitare prompt duplicati).
    """
    if not bool(getattr(node, "pause_between_phases", False)):
        return True
    if get_current_bt_name() != "Supervisor":
        return True

    key = _phase_pause_key(phase_name)
    if bool(node.bb.get(key, False)):
        return True

    # Ferma i comandi residui durante la pausa tra fasi.
    node.stop_all_movement()

    prompt = (
        f"\n[PHASE PAUSE] '{phase_name}' pronta.\n"
        "Premi INVIO per proseguire (oppure digita 'q' per rimanere in pausa): "
    )
    if not bool(getattr(sys.stdin, "isatty", lambda: False)()):
        node.get_logger().warn(
            bt_fmt(
                f"[PhasePause] stdin non interattivo: auto-continue phase '{phase_name}' "
                "(disabilita approach.pause_between_phases in contesti non TTY)"
            )
        )
        node.bb[key] = True
        return True

    node.get_logger().info(bt_fmt(f"[PhasePause] waiting user confirmation for '{phase_name}'"))
    try:
        ans = input(prompt).strip().lower()
    except Exception:
        ans = ""

    if ans in ("q", "quit", "n", "no", "stop"):
        node.get_logger().info(bt_fmt(f"[PhasePause] '{phase_name}' remains paused"))
        return False

    node.bb[key] = True
    node.get_logger().info(bt_fmt(f"[PhasePause] '{phase_name}' confirmed, continuing"))
    return True


def _phase_pause_reset(node, phase_name: str):
    node.bb.pop(_phase_pause_key(phase_name), None)


def _reset_movebase_runtime(node):
    node.bb.pop("movebase_stage", None)
    node.bb.pop("movebase_retreat_left_target_xy", None)
    node.bb.pop("movebase_retreat_right_target_xy", None)
    node.bb.pop("movebase_retreat_goal_name", None)
    node.bb.pop("movebase_transport_left_target_xy", None)
    node.bb.pop("movebase_transport_right_target_xy", None)
    node.bb.pop("movebase_transport_dst_pkg_xy", None)
    node.bb.pop("movebase_transport_source", None)
    node.bb.pop("movebase_transport_arm_goal_left", None)
    node.bb.pop("movebase_transport_arm_goal_right", None)


def _max_joint_error(node, q_now, q_goal) -> float:
    if q_now is None or q_goal is None:
        return float("inf")
    if len(q_now) != len(q_goal):
        return float("inf")
    errs = [abs(float(node._angle_diff(float(q_now[i]), float(q_goal[i])))) for i in range(len(q_now))]
    return float(max(errs)) if errs else 0.0


def _clip_descend_pick_joint_goals(node, q_left_goal: np.ndarray, q_right_goal: np.ndarray):
    try:
        j_up_all, j_low_all = node.robot.get_arm_joint_limits_pos()
        l_idx = int((node._arm_ee_index or {}).get("left", 0))
        r_idx = int((node._arm_ee_index or {}).get("right", 1))
        if isinstance(j_up_all, list) and isinstance(j_low_all, list):
            if 0 <= l_idx < len(j_up_all) and 0 <= l_idx < len(j_low_all):
                q_left_goal = np.clip(
                    q_left_goal,
                    np.asarray(j_low_all[l_idx], dtype=np.float32),
                    np.asarray(j_up_all[l_idx], dtype=np.float32),
                )
            if 0 <= r_idx < len(j_up_all) and 0 <= r_idx < len(j_low_all):
                q_right_goal = np.clip(
                    q_right_goal,
                    np.asarray(j_low_all[r_idx], dtype=np.float32),
                    np.asarray(j_up_all[r_idx], dtype=np.float32),
                )
    except Exception:
        pass
    return q_left_goal, q_right_goal


def _init_descend_pick_tp(
    node,
    left_arm_jp,
    right_arm_jp,
    descend_and_pick_time: float,
    left_arm_pick,
    right_arm_pick,
) -> bool:
    if node.approach_jtc_task is None:
        return False

    # left_arm_pick/right_arm_pick are configured as direct joint deltas.
    q_left_goal = np.asarray(left_arm_jp, dtype=np.float32) + np.asarray(left_arm_pick, dtype=np.float32)
    q_right_goal = np.asarray(right_arm_jp, dtype=np.float32) + np.asarray(right_arm_pick, dtype=np.float32)
    q_left_goal, q_right_goal = _clip_descend_pick_joint_goals(node, q_left_goal, q_right_goal)

    k_arm = np.full((len(q_left_goal) + len(q_right_goal),), float(node.approach_jtc_arm_kp), dtype=np.float32)
    try:
        node.approach_jtc_task.activate()
        node.approach_jtc_task.set_activation("base", False)
        node.approach_jtc_task.set_activation("arm", True)
        node.approach_jtc_task.set_arm_trajectories_pnts(
            target_joint_positions=[q_left_goal, q_right_goal],
            K_arm=k_arm,
            period=float(max(descend_and_pick_time, 0.2)),
        )
    except Exception as exc:
        node._warn_throttled(
            "lift_descend_tp_init_fail",
            bt_fmt(f"[LiftObj] descend_pick TP init failed: {exc}"),
            period_s=2.0,
        )
        return False

    node.bb["lift_descend_goal_left"] = [float(v) for v in q_left_goal.tolist()]
    node.bb["lift_descend_goal_right"] = [float(v) for v in q_right_goal.tolist()]
    node.bb["lift_descend_tp_ready"] = True
    node._tp_manip_last_exec_monotonic = None
    node.get_logger().info(
        bt_fmt(
            "[LiftObj] descend_pick TP initialized "
            f"(T={descend_and_pick_time:.2f}s, "
            f"L_goal={np.round(q_left_goal, 3).tolist()}, "
            f"R_goal={np.round(q_right_goal, 3).tolist()})"
        )
    )
    return True


def _init_pre_transport_tp(node, left_arm_jp, right_arm_jp) -> bool:
    if node.approach_jtc_task is None:
        return False

    man_cfg = node.cfg.manipulation
    q_left_goal = np.asarray(getattr(man_cfg, "pre_transport_left_arm_goal", left_arm_jp), dtype=np.float32)
    q_right_goal = np.asarray(getattr(man_cfg, "pre_transport_right_arm_goal", right_arm_jp), dtype=np.float32)

    if len(q_left_goal) != len(left_arm_jp) or len(q_right_goal) != len(right_arm_jp):
        node._warn_throttled(
            "lift_pre_transport_goal_size",
            bt_fmt("[LiftObj] invalid pre_transport joint goal size, skipping pre-transport pose"),
            period_s=2.0,
        )
        return False

    q_left_goal, q_right_goal = _clip_descend_pick_joint_goals(node, q_left_goal, q_right_goal)

    k_arm = np.full((len(q_left_goal) + len(q_right_goal),), float(node.approach_jtc_arm_kp), dtype=np.float32)
    traj_time = float(max(getattr(man_cfg, "pre_transport_traj_time", 2.5), 0.2))
    try:
        node.approach_jtc_task.activate()
        node.approach_jtc_task.set_activation("base", False)
        node.approach_jtc_task.set_activation("arm", True)
        node.approach_jtc_task.set_arm_trajectories_pnts(
            target_joint_positions=[q_left_goal, q_right_goal],
            K_arm=k_arm,
            period=traj_time,
        )
    except Exception as exc:
        node._warn_throttled(
            "lift_pre_transport_tp_init_fail",
            bt_fmt(f"[LiftObj] pre_transport TP init failed: {exc}"),
            period_s=2.0,
        )
        return False

    node.bb["lift_pre_transport_goal_left"] = [float(v) for v in q_left_goal.tolist()]
    node.bb["lift_pre_transport_goal_right"] = [float(v) for v in q_right_goal.tolist()]
    node.bb["lift_pre_transport_tp_ready"] = True
    node._tp_manip_last_exec_monotonic = None
    node.get_logger().info(
        bt_fmt(
            "[LiftObj] pre_transport TP initialized "
            f"(T={traj_time:.2f}s, "
            f"L_goal={np.round(q_left_goal, 3).tolist()}, "
            f"R_goal={np.round(q_right_goal, 3).tolist()})"
        )
    )
    return True


def _log_force_proxy(node, label: str, period_s: float = 1.0):
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
        if node.approach_use_base and used_source == "estimated" and pallet_source in ("config", "env", "gazebo", "world_file"):
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

    now_ros_s = _ros_now_s(node)
    cmd = node._tp_cmd_cache
    can_reuse = (
        cmd is not None
        and node._tp_cmd_cache_time is not None
        and (now_ros_s - node._tp_cmd_cache_time) < max(node.tp_cmd_min_period, 1e-3)
    )
    if not can_reuse:
        tp_dt = _dt_from_ros_time(now_ros_s, node._tp_last_exec_monotonic, default_dt=1.0 / 30.0)
        node._tp_last_exec_monotonic = now_ros_s
        # dt robusto: evita valori troppo piccoli/grandi che destabilizzano il controllo.
        node.tp._delta_t = max(1e-3, min(float(tp_dt), 0.2))
        cmd = node.tp.execute(joint_pos=joint_pos, base_odom=base_odom)
        node._tp_cmd_cache = cmd
        node._tp_cmd_cache_time = now_ros_s
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

    finite_base_dists = [d for d in (left_base_dist, right_base_dist) if math.isfinite(d)]
    arm_gate_dist = max(finite_base_dists) if finite_base_dists else float("inf")
    node._approach_gate_dist = float(arm_gate_dist)
    arm_delay_waiting = bool(node._approach_arm_delay_active and (not node._approach_arm_motion_enabled))
    if arm_delay_waiting:
        if arm_gate_dist <= float(node.approach_arm_enable_dist):
            if node._enable_approach_arm_motion_from_current_state(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            ):
                node._tp_cmd_cache = None
                node._tp_cmd_cache_time = None
                node._tp_last_exec_monotonic = None
                arm_delay_waiting = False
        else:
            node._info_throttled(
                f"{tree_name}_approach_arm_hold",
                bt_fmt(
                    "[ApproachObject] arm pre-phase hold-home active "
                    f"(base_dist={arm_gate_dist:.3f}m > enable_dist={node.approach_arm_enable_dist:.3f}m)"
                ),
                period_s=1.0,
            )

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
    if arm_delay_waiting:
        left_arm_reached = False
        right_arm_reached = False
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
    arm_phase = "tracking" if bool(node._approach_arm_motion_enabled) else ("hold_home" if bool(node._approach_arm_delay_active) else "disabled")

    node._info_throttled(
        f"{tree_name}_approach_cmd",
        bt_fmt(
            "[ApproachObject] cmd "
            f"(side={side or 'both'}) "
            f"base=[{base_cmd_vals[0]:.4f},{base_cmd_vals[1]:.4f},{base_cmd_vals[2]:.4f}], "
            f"arm_max={max(abs(v) for v in arm_cmd_vals):.4f}, "
            f"arm_reached={arm_reached}, base_reached={base_reached}, arm_phase={arm_phase}, "
            f"arm_gate_dist={arm_gate_dist:.3f}, arm_enable_dist={node.approach_arm_enable_dist:.3f}, "
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
        _publish_base_cmd(node, left_cmd=left_base_cmd_vals)
        _publish_arm_cmd(node, left_cmd=left_arm_cmd_vals)
    elif side == "right":
        _publish_base_cmd(node, right_cmd=right_base_cmd_vals)
        _publish_arm_cmd(node, right_cmd=right_arm_cmd_vals)
    else:
        _publish_base_cmd(node, left_cmd=left_base_cmd_vals, right_cmd=right_base_cmd_vals)
        _publish_arm_cmd(node, left_cmd=left_arm_cmd_vals, right_cmd=right_arm_cmd_vals)

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
    - 1) tempo discesa/pick da config: bracci verso pose di pick
    - 2) 0.5 s                   : attach del pacco
    - 3) tempo collect da config: basi + bracci in collect (sollevamento)

    Mappa le fasi:
      * discesa
      * pick (attach)
      * collect
    """
    node = _require_node()
    phase_cfg = node.cfg.phases
    motion = node.cfg.motion_profiles
    man_cfg = node.cfg.manipulation
    descend_and_pick_time = float(phase_cfg.descend_and_pick_time)
    collect_time = float(phase_cfg.collect_time)
    left_arm_pick = _float_vec(motion.left_arm_pick)
    right_arm_pick = _float_vec(motion.right_arm_pick)
    collect_left_base_xy = _float_vec(motion.collect_left_base_xy_vel)
    collect_right_base_xy = _float_vec(motion.collect_right_base_xy_vel)
    left_arm_collect = _float_vec(motion.left_arm_collect)
    right_arm_collect = _float_vec(motion.right_arm_collect)
    use_pkg_hold = bool(man_cfg.enable_object_centric_hold)
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

    if not _phase_pause_gate(node, "LiftObj"):
        return None

    t0 = node.get_action_timer("LiftObj")
    if t0 is None or node.lift_phase is None:
        node.get_logger().info(bt_fmt(f"[LiftObj] start (dur {descend_and_pick_time + 0.5 + collect_time}s)"))
        t0 = node.start_action_timer("LiftObj")
        node.lift_phase = "descend_pick"
        _reset_pkg_hold_runtime(node)
        _reset_pick_waypoint_runtime(node)
        node.bb["pkg_hold_ok"] = False
        node.bb["lift_descend_tp_ready"] = False
        node.bb.pop("lift_descend_goal_left", None)
        node.bb.pop("lift_descend_goal_right", None)
        node.bb["lift_pre_transport_tp_ready"] = False
        node.bb.pop("lift_pre_transport_goal_left", None)
        node.bb.pop("lift_pre_transport_goal_right", None)
        node.bb.pop("lift_collect_tp_initialized", None)
        node.bb.pop("lift_collect_left_target_xy", None)
        node.bb.pop("lift_collect_right_target_xy", None)
        node.bb.pop("lift_collect_arm_goal_left", None)
        node.bb.pop("lift_collect_arm_goal_right", None)

    elapsed = node.get_clock().now().nanoseconds/1e9 - t0

    # 1) Discesa e pick
    if node.lift_phase == "descend_pick":
        live_state = _get_live_tp_state(node)
        if live_state is None:
            node._warn_throttled(
                "lift_descend_wait_data",
                bt_fmt("[LiftObj] waiting for sensor data (joint_states/odom)"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        use_waypoint_pick = bool(getattr(man_cfg, "pick_use_ee_waypoints", True))
        if use_waypoint_pick:
            left_arm_jp, right_arm_jp, left_base, right_base = live_state
            stages = node.bb.get("lift_pick_waypoints", None)
            if stages is None:
                stages = _build_pick_waypoint_stage_plan(
                    node,
                    left_arm_jp=left_arm_jp,
                    right_arm_jp=right_arm_jp,
                    left_base=left_base,
                    right_base=right_base,
                )
                if not stages:
                    node._warn_throttled(
                        "lift_pick_waypoints_init_fail",
                        bt_fmt("[LiftObj] pick waypoint init failed, fallback to joint descend"),
                        period_s=2.0,
                    )
                    use_waypoint_pick = False
                else:
                    node.bb["lift_pick_waypoints"] = stages
                    node.bb["lift_pick_stage_idx"] = 0
                    node.bb["lift_pick_stage_active"] = False
                    node.get_logger().info(
                        bt_fmt(
                            "[LiftObj] pick waypoints initialized "
                            f"(stages={[s['name'] for s in stages]})"
                        )
                    )

            if use_waypoint_pick:
                stages = node.bb.get("lift_pick_waypoints", []) or []
                stage_idx = int(node.bb.get("lift_pick_stage_idx", 0))
                if stage_idx >= len(stages):
                    node.stop_all_movement()
                    _reset_pick_waypoint_runtime(node)
                    node.lift_phase = "attach"
                    node.bb["lift_descend_tp_ready"] = False
                    node.bb.pop("lift_descend_goal_left", None)
                    node.bb.pop("lift_descend_goal_right", None)
                    node.get_logger().info(bt_fmt("[LiftObj] pick waypoints completed, attaching..."))
                    node.start_action_timer("LiftObj")
                    return None

                stage = stages[stage_idx]
                if not bool(node.bb.get("lift_pick_stage_active", False)):
                    ee_live = node._get_live_ee_by_side(
                        left_arm_jp=left_arm_jp,
                        right_arm_jp=right_arm_jp,
                        left_base=left_base,
                        right_base=right_base,
                    )
                    left_ee = ee_live.get("left", None)
                    right_ee = ee_live.get("right", None)
                    if left_ee is None or right_ee is None:
                        node._warn_throttled(
                            "lift_pick_waypoints_no_ee",
                            bt_fmt("[LiftObj] cannot start pick stage: live EE pose unavailable"),
                            period_s=1.0,
                        )
                        rclpy.spin_once(node, timeout_sec=0.01)
                        return None
                    if not _set_tp_ee_traj(
                        node,
                        left_ee_now=left_ee,
                        right_ee_now=right_ee,
                        left_ee_goal=stage["left_goal"],
                        right_ee_goal=stage["right_goal"],
                        traj_time=float(stage.get("traj_time", 2.0)),
                    ):
                        node._warn_throttled(
                            "lift_pick_waypoints_set_traj_fail",
                            bt_fmt("[LiftObj] unable to set TP EE waypoint trajectory, fallback to joint descend"),
                            period_s=2.0,
                        )
                        _reset_pick_waypoint_runtime(node)
                        use_waypoint_pick = False
                    else:
                        node.bb["lift_pick_stage_active"] = True
                        node.bb["lift_pick_stage_start_s"] = _ros_now_s(node)
                        node.get_logger().info(
                            bt_fmt(
                                f"[LiftObj] pick stage start: {stage['name']} "
                                f"(T={float(stage.get('traj_time', 0.0)):.2f}s, "
                                f"L={np.round(np.asarray(stage['left_goal'])[:3], 3).tolist()}, "
                                f"R={np.round(np.asarray(stage['right_goal'])[:3], 3).tolist()})"
                            )
                        )

                if use_waypoint_pick and bool(node.bb.get("lift_pick_stage_active", False)):
                    left_arm_jp, right_arm_jp, left_base, right_base = live_state
                    _execute_tp_arm_control(
                        node,
                        left_arm_jp=left_arm_jp,
                        right_arm_jp=right_arm_jp,
                        left_base=left_base,
                        right_base=right_base,
                        arm_clip_abs=float(getattr(man_cfg, "pick_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
                    )
                    ee_live = node._get_live_ee_by_side(
                        left_arm_jp=left_arm_jp,
                        right_arm_jp=right_arm_jp,
                        left_base=left_base,
                        right_base=right_base,
                    )
                    l_ok, l_pos, l_ori = _ee_goal_reached(
                        node,
                        ee_live.get("left", None),
                        stage["left_goal"],
                        pos_tol=float(getattr(man_cfg, "pick_pos_tol", 0.03)),
                        ori_tol=float(getattr(man_cfg, "pick_ori_tol", 0.20)),
                    )
                    r_ok, r_pos, r_ori = _ee_goal_reached(
                        node,
                        ee_live.get("right", None),
                        stage["right_goal"],
                        pos_tol=float(getattr(man_cfg, "pick_pos_tol", 0.03)),
                        ori_tol=float(getattr(man_cfg, "pick_ori_tol", 0.20)),
                    )
                    t_stage = float(node.bb.get("lift_pick_stage_start_s", _ros_now_s(node)))
                    stage_elapsed = _ros_now_s(node) - t_stage
                    stage_timeout = max(
                        float(getattr(man_cfg, "pick_stage_timeout", 6.0)),
                        float(stage.get("traj_time", 0.0)) + 0.2,
                    )
                    reached = bool(l_ok and r_ok)
                    node._info_throttled(
                        f"lift_pick_waypoint_track_{stage_idx}",
                        bt_fmt(
                            f"[LiftObj] pick stage {stage['name']} "
                            f"reached={reached}, "
                            f"L(pos={l_pos:.3f},ori={l_ori:.3f}), "
                            f"R(pos={r_pos:.3f},ori={r_ori:.3f}), "
                            f"elapsed={stage_elapsed:.2f}/{stage_timeout:.2f}s"
                        ),
                        period_s=1.0,
                    )
                    _log_force_proxy(node, f"pick_{stage['name']}", period_s=1.0)
                    if reached or stage_elapsed >= stage_timeout:
                        node.stop_all_movement()
                        node.bb["lift_pick_stage_idx"] = int(stage_idx) + 1
                        node.bb["lift_pick_stage_active"] = False
                        node.bb.pop("lift_pick_stage_start_s", None)
                        if stage_elapsed >= stage_timeout and (not reached):
                            node.get_logger().warn(
                                bt_fmt(
                                    f"[LiftObj] pick stage timeout: {stage['name']} "
                                    f"(continuing to next stage)"
                                )
                            )
                        else:
                            node.get_logger().info(bt_fmt(f"[LiftObj] pick stage completed: {stage['name']}"))
                    rclpy.spin_once(node, timeout_sec=0.01)
                    return None

        tp_ready = bool(node.bb.get("lift_descend_tp_ready", False))
        if live_state is not None and (not tp_ready):
            left_arm_jp, right_arm_jp, _left_base, _right_base = live_state
            tp_ready = _init_descend_pick_tp(
                node,
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                descend_and_pick_time=descend_and_pick_time,
                left_arm_pick=left_arm_pick,
                right_arm_pick=right_arm_pick,
            )

        descend_reached = False
        if live_state is not None and tp_ready:
            left_arm_jp, right_arm_jp, left_base, right_base = live_state
            _execute_tp_arm_hold(node, left_arm_jp, right_arm_jp, left_base, right_base)
            q_left_goal = node.bb.get("lift_descend_goal_left", None)
            q_right_goal = node.bb.get("lift_descend_goal_right", None)
            l_err = _max_joint_error(node, left_arm_jp, q_left_goal)
            r_err = _max_joint_error(node, right_arm_jp, q_right_goal)
            descend_reached = bool(
                np.isfinite(l_err)
                and np.isfinite(r_err)
                and l_err <= float(node.approach_arm_joint_tol)
                and r_err <= float(node.approach_arm_joint_tol)
            )
            node._info_throttled(
                "lift_descend_tp_track",
                bt_fmt(
                    f"[LiftObj] descending (TP)... "
                    f"l_err={l_err:.4f}, r_err={r_err:.4f}, "
                    f"tol={node.approach_arm_joint_tol:.4f}, reached={descend_reached}"
                ),
                period_s=1.0,
            )
            _log_force_proxy(node, "descend_pick", period_s=1.0)
        elif elapsed < descend_and_pick_time:
            node._warn_throttled(
                "lift_descend_tp_required",
                bt_fmt("[LiftObj] TP required: descend stage waiting TP init (open-loop disabled)"),
                period_s=2.0,
            )

        if elapsed < descend_and_pick_time and not descend_reached:
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        # passa alla fase attach (per tempo o convergenza)
        node.stop_all_movement()
        _reset_pick_waypoint_runtime(node)
        node.lift_phase = "attach"
        node.bb["lift_descend_tp_ready"] = False
        node.bb.pop("lift_descend_goal_left", None)
        node.bb.pop("lift_descend_goal_right", None)
        node.get_logger().info(bt_fmt("[LiftObj] reached pick pose, attaching..."))
        node.start_action_timer("LiftObj")  # restart timer per attach
        return None

    # 2) Attach (tempo breve)
    if node.lift_phase == "attach":
        if not bool(node.bb.get("package_attached", False)):
            if _attach_package_to_arms(node):
                node.bb["package_attached"] = True

        if bool(node.bb.get("package_attached", False)):
            if not node.bb.get("package_gravity_disabled", False):
                if set_package_gravity(node, False):
                    node.bb["package_gravity_disabled"] = True
            if use_pkg_hold:
                live_state = _get_live_tp_state(node)
                if live_state is not None:
                    left_arm_jp, right_arm_jp, left_base, right_base = live_state
                    _capture_pkg_grasp_offsets(node, left_arm_jp, right_arm_jp, left_base, right_base)
                    if node._pkg_hold_start_z is not None:
                        node._pkg_hold_target_z = float(node._pkg_hold_start_z) + float(man_cfg.collect_lift_delta_z)
                        _replan_pkg_hold_tp(
                            node,
                            left_arm_jp,
                            right_arm_jp,
                            left_base,
                            right_base,
                            pkg_z_target=node._pkg_hold_target_z,
                            force=True,
                        )
            node.stop_all_movement()
            node.lift_phase = "collect"
            node.get_logger().info(bt_fmt("[LiftObj] attached, starting collect"))
            node.start_action_timer("LiftObj")  # restart timer per collect
            return None

        node._warn_throttled(
            "lift_attach_wait",
            bt_fmt(f"[LiftObj] waiting package attach... elapsed={elapsed:.2f}s"),
            period_s=0.8,
        )
        rclpy.spin_once(node, timeout_sec=0.01)
        return None

    # 3) Collect / sollevamento
    if node.lift_phase == "collect":
        if elapsed < collect_time:
            live_state = _get_live_tp_state(node)
            if live_state is None:
                node._warn_throttled(
                    "lift_collect_wait_data",
                    bt_fmt("[LiftObj] collect waiting sensor data (joint_states/odom)"),
                    period_s=1.0,
                )
                rclpy.spin_once(node, timeout_sec=0.01)
                return None

            left_arm_jp, right_arm_jp, left_base, right_base = live_state
            if not bool(node.bb.get("lift_collect_tp_initialized", False)):
                base_scale = 1.0
                if use_pkg_hold and bool(node.bb.get("package_attached", False)):
                    base_scale = float(getattr(man_cfg, "hold_base_vel_scale", 1.0))
                left_target_xy = _predict_world_target_from_body_velocity(
                    left_base,
                    _scaled_xy(collect_left_base_xy, base_scale),
                    collect_time,
                )
                right_target_xy = _predict_world_target_from_body_velocity(
                    right_base,
                    _scaled_xy(collect_right_base_xy, base_scale),
                    collect_time,
                )
                if _init_tp_base_stage(
                    node,
                    left_base_goal_xy=left_target_xy,
                    right_base_goal_xy=right_target_xy,
                    period_s=float(max(collect_time, 0.2)),
                    kp_xy=float(getattr(man_cfg, "transport_retreat_kp_x", 1.0)),
                    kp_yaw=0.0,
                ):
                    node.bb["lift_collect_tp_initialized"] = True
                    node.bb["lift_collect_left_target_xy"] = left_target_xy
                    node.bb["lift_collect_right_target_xy"] = right_target_xy
                else:
                    node._warn_throttled(
                        "lift_collect_base_tp_init_fail",
                        bt_fmt("[LiftObj] collect TP base init failed"),
                        period_s=2.0,
                    )
                    return False

                if (not use_pkg_hold) and bool(node.bb.get("package_attached", False)):
                    q_left_goal = np.asarray(left_arm_jp, dtype=np.float32) + np.asarray(left_arm_collect, dtype=np.float32)
                    q_right_goal = np.asarray(right_arm_jp, dtype=np.float32) + np.asarray(right_arm_collect, dtype=np.float32)
                    q_left_goal, q_right_goal = _clip_descend_pick_joint_goals(node, q_left_goal, q_right_goal)
                    _init_tp_arm_joint_stage(node, q_left_goal, q_right_goal, period_s=float(max(collect_time, 0.2)))
                    node.bb["lift_collect_arm_goal_left"] = [float(v) for v in q_left_goal.tolist()]
                    node.bb["lift_collect_arm_goal_right"] = [float(v) for v in q_right_goal.tolist()]

            _execute_tp_full_control(
                node,
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
                arm_clip_abs=float(getattr(man_cfg, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
                base_xy_abs_max=float(getattr(man_cfg, "transport_retreat_cmd_xy_abs_max", 0.20)),
                base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
            )
            hold_executed = False
            if use_pkg_hold and bool(node.bb.get("package_attached", False)):
                if node._pkg_hold_target_z is None:
                    pkg_xyz = _resolve_pkg_reference_xyz(
                        node,
                        left_arm_jp=left_arm_jp,
                        right_arm_jp=right_arm_jp,
                        left_base=left_base,
                        right_base=right_base,
                    )
                    if pkg_xyz is not None:
                        node._pkg_hold_target_z = float(pkg_xyz[2]) + float(man_cfg.collect_lift_delta_z)
                if _replan_pkg_hold_tp(
                    node,
                    left_arm_jp,
                    right_arm_jp,
                    left_base,
                    right_base,
                    pkg_z_target=node._pkg_hold_target_z,
                    preserve_jtc_base=True,
                ):
                    _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "collect")
                    _log_force_proxy(node, "collect", period_s=1.0)
                    hold_executed = True
            if not hold_executed:
                node._warn_throttled(
                    "lift_collect_tp_required",
                    bt_fmt("[LiftObj] TP required: collect arm hold unavailable (open-loop disabled)"),
                    period_s=2.0,
                )

            node._info_throttled("lift_collect", bt_fmt("[LiftObj] collecting (lifting)..."), period_s=1.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        else:
            node.stop_all_movement()
            node.bb.pop("lift_collect_tp_initialized", None)
            node.bb.pop("lift_collect_left_target_xy", None)
            node.bb.pop("lift_collect_right_target_xy", None)
            node.bb.pop("lift_collect_arm_goal_left", None)
            node.bb.pop("lift_collect_arm_goal_right", None)
            if bool(getattr(man_cfg, "pre_transport_enable", False)):
                node.lift_phase = "pre_transport_pose"
                node.bb["lift_pre_transport_tp_ready"] = False
                node.bb.pop("lift_pre_transport_goal_left", None)
                node.bb.pop("lift_pre_transport_goal_right", None)
                node.bb.pop("lift_pre_transport_base_tp_initialized", None)
                node.bb.pop("lift_pre_transport_base_left_target_xy", None)
                node.bb.pop("lift_pre_transport_base_right_target_xy", None)
                node.start_action_timer("LiftObj")
                node.get_logger().info(bt_fmt("[LiftObj] collect done, entering pre-transport pose"))
                return None
            node.lift_phase = None
            node.clear_action_timer("LiftObj")
            _phase_pause_reset(node, "LiftObj")
            node.get_logger().info(bt_fmt("[LiftObj] completed"))
            return True

    # 4) Pre-transport pose (opzionale): package-centric (default) o joint-space (fallback)
    if node.lift_phase == "pre_transport_pose":
        live_state = _get_live_tp_state(node)
        if live_state is None:
            node._warn_throttled(
                "lift_pre_transport_wait_data",
                bt_fmt("[LiftObj] waiting sensor data for pre-transport pose"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        mode = str(getattr(man_cfg, "pre_transport_mode", "package")).strip().lower()
        if mode not in ("package", "joint"):
            mode = "package"

        if mode == "joint":
            tp_ready = bool(node.bb.get("lift_pre_transport_tp_ready", False))
            if not tp_ready:
                tp_ready = _init_pre_transport_tp(node, left_arm_jp=left_arm_jp, right_arm_jp=right_arm_jp)
                if not tp_ready:
                    node._warn_throttled(
                        "lift_pre_transport_disabled_runtime",
                        bt_fmt("[LiftObj] pre-transport pose skipped (TP init failed)"),
                        period_s=2.0,
                    )
                    node.lift_phase = None
                    node.clear_action_timer("LiftObj")
                    _phase_pause_reset(node, "LiftObj")
                    node.get_logger().info(bt_fmt("[LiftObj] completed"))
                    return True

            _execute_tp_arm_control(
                node,
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
                arm_clip_abs=float(getattr(man_cfg, "pre_transport_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
            )

            q_left_goal = node.bb.get("lift_pre_transport_goal_left", None)
            q_right_goal = node.bb.get("lift_pre_transport_goal_right", None)
            l_err = _max_joint_error(node, left_arm_jp, q_left_goal)
            r_err = _max_joint_error(node, right_arm_jp, q_right_goal)
            tol = float(getattr(man_cfg, "pre_transport_joint_tol", 0.12))
            reached = bool(np.isfinite(l_err) and np.isfinite(r_err) and l_err <= tol and r_err <= tol)
            node._info_throttled(
                "lift_pre_transport_track",
                bt_fmt(
                    f"[LiftObj] pre-transport pose tracking (joint) "
                    f"l_err={l_err:.4f}, r_err={r_err:.4f}, tol={tol:.4f}, reached={reached}"
                ),
                period_s=1.0,
            )
            _log_force_proxy(node, "pre_transport_joint", period_s=1.0)
        else:
            _ensure_pkg_hold_state(node)
            if node._pkg_hold_offsets.get("left", None) is None or node._pkg_hold_offsets.get("right", None) is None:
                _capture_pkg_grasp_offsets(node, left_arm_jp, right_arm_jp, left_base, right_base)

            # In pre-transport usiamo come riferimento primario la posa pacco da Gazebo
            # (non la stima da un singolo EE), per evitare bias/asimmetrie tra i due lati.
            pkg_xyz = _get_live_package_xyz(node)
            if pkg_xyz is None:
                pkg_xyz = _resolve_pkg_reference_xyz(
                    node,
                    left_arm_jp=left_arm_jp,
                    right_arm_jp=right_arm_jp,
                    left_base=left_base,
                    right_base=right_base,
                )
            if pkg_xyz is None:
                node._warn_throttled(
                    "lift_pre_transport_pkg_missing",
                    bt_fmt("[LiftObj] pre-transport package reference unavailable"),
                    period_s=1.0,
                )
                rclpy.spin_once(node, timeout_sec=0.01)
                return None

            left_off = node._pkg_hold_offsets.get("left", None)
            right_off = node._pkg_hold_offsets.get("right", None)
            half_span = None
            if isinstance(left_off, np.ndarray) and isinstance(right_off, np.ndarray) and len(left_off) >= 1 and len(right_off) >= 1:
                half_span = 0.5 * abs(float(right_off[0]) - float(left_off[0]))
            if (half_span is None or half_span <= 0.0) and np.isfinite(getattr(node, "_pkg_hold_nominal_dist", float("nan"))):
                half_span = 0.5 * float(node._pkg_hold_nominal_dist)
            if half_span is None or half_span <= 0.0:
                half_span = 0.5 * abs(float(man_cfg.hold_right_offset_x) - float(man_cfg.hold_left_offset_x))
            half_span += abs(float(getattr(man_cfg, "pre_transport_pkg_half_span_margin_x", 0.02)))

            y_off = float(getattr(man_cfg, "pre_transport_pkg_offset_y", -0.10))
            z_off = float(getattr(man_cfg, "pre_transport_pkg_offset_z", 0.18))
            node._pkg_hold_offsets["left"] = np.asarray([-half_span, y_off, z_off], dtype=np.float32)
            node._pkg_hold_offsets["right"] = np.asarray([half_span, y_off, z_off], dtype=np.float32)
            node._pkg_hold_nominal_dist = float(2.0 * abs(half_span))
            node._pkg_hold_target_z = float(pkg_xyz[2])

            base_align_enable = bool(getattr(man_cfg, "pre_transport_base_align_enable", True))
            hold_ok = False
            if _replan_pkg_hold_tp(
                node,
                left_arm_jp,
                right_arm_jp,
                left_base,
                right_base,
                pkg_z_target=node._pkg_hold_target_z,
                preserve_jtc_base=base_align_enable,
            ):
                hold_ok = bool(node.bb.get("pkg_hold_ok", False))

            base_ok = True
            l_dist = float("nan")
            r_dist = float("nan")
            if base_align_enable:
                l_target = [
                    float(pkg_xyz[0]) + float(getattr(man_cfg, "pre_transport_base_left_offset_x", -0.60)),
                    float(pkg_xyz[1]) + float(getattr(man_cfg, "pre_transport_base_offset_y", -0.70)),
                ]
                r_target = [
                    float(pkg_xyz[0]) + float(getattr(man_cfg, "pre_transport_base_right_offset_x", 0.60)),
                    float(pkg_xyz[1]) + float(getattr(man_cfg, "pre_transport_base_offset_y", -0.70)),
                ]
                if not bool(node.bb.get("lift_pre_transport_base_tp_initialized", False)):
                    if _init_tp_base_stage(
                        node,
                        left_base_goal_xy=l_target,
                        right_base_goal_xy=r_target,
                        period_s=float(max(getattr(man_cfg, "pre_transport_traj_time", 2.5), 0.2)),
                        kp_xy=float(getattr(man_cfg, "pre_transport_base_kp_x", 0.9)),
                        kp_yaw=0.0,
                    ):
                        node.bb["lift_pre_transport_base_tp_initialized"] = True
                        node.bb["lift_pre_transport_base_left_target_xy"] = l_target
                        node.bb["lift_pre_transport_base_right_target_xy"] = r_target
                l_ref = node.bb.get("lift_pre_transport_base_left_target_xy", l_target)
                r_ref = node.bb.get("lift_pre_transport_base_right_target_xy", r_target)
                l_dist = float(math.hypot(float(l_ref[0]) - float(left_base[0]), float(l_ref[1]) - float(left_base[1])))
                r_dist = float(math.hypot(float(r_ref[0]) - float(right_base[0]), float(r_ref[1]) - float(right_base[1])))
                base_tol = float(getattr(man_cfg, "pre_transport_base_goal_tol", 0.10))
                base_ok = bool(l_dist <= base_tol and r_dist <= base_tol)

            if base_align_enable:
                _execute_tp_full_control(
                    node,
                    left_arm_jp=left_arm_jp,
                    right_arm_jp=right_arm_jp,
                    left_base=left_base,
                    right_base=right_base,
                    arm_clip_abs=float(getattr(man_cfg, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
                    base_xy_abs_max=float(getattr(man_cfg, "pre_transport_base_cmd_xy_abs_max", 0.12)),
                    base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
                )
            else:
                _execute_tp_arm_hold(node, left_arm_jp, right_arm_jp, left_base, right_base)

            _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "pre_transport")
            _log_force_proxy(node, "pre_transport_pkg", period_s=1.0)
            if base_align_enable:
                node._info_throttled(
                    "lift_pre_transport_base_align",
                    bt_fmt(
                        f"[LiftObj] pre-transport base align (TP) "
                        f"L_dist={l_dist:.3f}, R_dist={r_dist:.3f}, ok={base_ok}"
                    ),
                    period_s=1.0,
                )

            reached = bool(hold_ok and base_ok)
            node._info_throttled(
                "lift_pre_transport_track",
                bt_fmt(
                    f"[LiftObj] pre-transport pose tracking (package) "
                    f"hold_ok={hold_ok}, base_ok={base_ok}, reached={reached}, "
                    f"half_span={half_span:.3f}"
                ),
                period_s=1.0,
            )

        phase_t0 = node.get_action_timer("LiftObj")
        phase_elapsed = (_ros_now_s(node) - float(phase_t0)) if phase_t0 is not None else 0.0
        timeout = max(
            float(getattr(man_cfg, "pre_transport_stage_timeout", 6.0)),
            float(getattr(man_cfg, "pre_transport_traj_time", 2.5)) + 0.2,
        )
        node._info_throttled(
            "lift_pre_transport_elapsed",
            bt_fmt(f"[LiftObj] pre-transport elapsed={phase_elapsed:.2f}/{timeout:.2f}s"),
            period_s=1.0,
        )

        if reached or phase_elapsed >= timeout:
            node.stop_all_movement()
            node.bb["lift_pre_transport_tp_ready"] = False
            node.bb.pop("lift_pre_transport_goal_left", None)
            node.bb.pop("lift_pre_transport_goal_right", None)
            node.bb.pop("lift_pre_transport_base_tp_initialized", None)
            node.bb.pop("lift_pre_transport_base_left_target_xy", None)
            node.bb.pop("lift_pre_transport_base_right_target_xy", None)
            node.lift_phase = None
            node.clear_action_timer("LiftObj")
            _phase_pause_reset(node, "LiftObj")
            if phase_elapsed >= timeout and (not reached):
                node.get_logger().warn(bt_fmt("[LiftObj] pre-transport pose timeout, continuing"))
            node.get_logger().info(bt_fmt("[LiftObj] completed"))
            return True

        rclpy.spin_once(node, timeout_sec=0.01)
        return None

    # fallback
    node.lift_phase = None
    node.clear_action_timer("LiftObj")
    _phase_pause_reset(node, "LiftObj")
    return True


def MoveBase():
    """
    Movimento delle basi (transfer):
    - TP puro in 2 segmenti:
      1) retreat: allontanamento dal pallet
      2) transport: traslazione "a blocco rigido" verso destinazione world
         (default: model `pacco_clone_2` con offset Y=-2.0m).
    """
    node = _require_node()
    phase_cfg = node.cfg.phases
    motion = node.cfg.motion_profiles
    man_cfg = node.cfg.manipulation
    transport_time = float(phase_cfg.transport_time)
    left_transport_xy = _float_vec(motion.left_transport_vel_xy)
    right_transport_xy = _float_vec(motion.right_transport_vel_xy)
    use_pkg_hold = bool(man_cfg.enable_object_centric_hold)
    use_retreat = bool(getattr(man_cfg, "transport_retreat_enable", True))
    lock_arms = bool(getattr(man_cfg, "transport_lock_arm_joints", True))
    movebase_arm_clip_abs = float(
        max(
            1e-4,
            float(getattr(man_cfg, "transport_lock_arm_cmd_abs_max", 0.22))
            if lock_arms
            else float(getattr(man_cfg, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
        )
    )
    lock_arm_kp = float(getattr(man_cfg, "transport_lock_arm_kp", node.approach_jtc_arm_kp))

    def _get_transport_arm_hold_goals(left_arm_jp, right_arm_jp, force_refresh: bool = False):
        gl = node.bb.get("movebase_transport_arm_goal_left", None)
        gr = node.bb.get("movebase_transport_arm_goal_right", None)
        if force_refresh or (not isinstance(gl, (list, tuple)) or len(gl) != 6):
            gl = [float(v) for v in np.asarray(left_arm_jp, dtype=np.float32).tolist()]
            node.bb["movebase_transport_arm_goal_left"] = list(gl)
        if force_refresh or (not isinstance(gr, (list, tuple)) or len(gr) != 6):
            gr = [float(v) for v in np.asarray(right_arm_jp, dtype=np.float32).tolist()]
            node.bb["movebase_transport_arm_goal_right"] = list(gr)
        return gl, gr

    def _init_transport_base_arm_hold(left_target_xy, right_target_xy, left_arm_jp, right_arm_jp, period_s: float, kp_xy: float) -> bool:
        ok_base = _init_tp_base_stage(
            node,
            left_base_goal_xy=left_target_xy,
            right_base_goal_xy=right_target_xy,
            period_s=float(max(period_s, 0.2)),
            kp_xy=kp_xy,
            kp_yaw=0.0,
        )
        arm_goal_l, arm_goal_r = _get_transport_arm_hold_goals(left_arm_jp, right_arm_jp, force_refresh=True)
        ok_arm = _init_tp_arm_joint_stage(
            node,
            left_arm_goal=arm_goal_l,
            right_arm_goal=arm_goal_r,
            period_s=float(max(period_s, 0.2)),
            kp_arm=lock_arm_kp,
        )
        if bool(ok_base and ok_arm) and (node.approach_jtc_task is not None):
            try:
                node.approach_jtc_task.activate()
                node.approach_jtc_task.set_activation("base", True)
                node.approach_jtc_task.set_activation("arm", True)
            except Exception:
                return False
            return True
        return False

    def _init_transport_stage(left_arm_jp, right_arm_jp, left_base_pose, right_base_pose, period_s: float):
        left_target_xy = None
        right_target_xy = None

        dst_pkg_xy, dst_source = _resolve_transport_destination_xy(node)
        if isinstance(dst_pkg_xy, (list, tuple)) and len(dst_pkg_xy) >= 2:
            # Trasporto: usa riferimento pacco reale per evitare bias su un solo EE.
            pkg_ref_xyz = _get_live_package_xyz(node)
            if pkg_ref_xyz is None:
                pkg_ref_xyz = _resolve_pkg_reference_xyz(
                    node,
                    left_arm_jp=left_arm_jp,
                    right_arm_jp=right_arm_jp,
                    left_base=left_base_pose,
                    right_base=right_base_pose,
                )
            if isinstance(pkg_ref_xyz, (list, tuple)) and len(pkg_ref_xyz) >= 2:
                dx = float(dst_pkg_xy[0]) - float(pkg_ref_xyz[0])
                dy = float(dst_pkg_xy[1]) - float(pkg_ref_xyz[1])
                left_target_xy = [float(left_base_pose[0]) + dx, float(left_base_pose[1]) + dy]
                right_target_xy = [float(right_base_pose[0]) + dx, float(right_base_pose[1]) + dy]
                node.bb["movebase_transport_dst_pkg_xy"] = [float(dst_pkg_xy[0]), float(dst_pkg_xy[1])]
            else:
                dst_source = f"{dst_source}|no_pkg_ref"
        else:
            dst_source = f"{dst_source}|no_dst"

        # Fallback legacy: usa profilo body-frame se la destinazione world non e' disponibile.
        if left_target_xy is None or right_target_xy is None:
            left_cmd_xy = (
                _scaled_xy(left_transport_xy, float(getattr(man_cfg, "hold_base_vel_scale", 1.0)))
                if (use_pkg_hold and bool(node.bb.get("package_attached", False)))
                else left_transport_xy
            )
            right_cmd_xy = (
                _scaled_xy(right_transport_xy, float(getattr(man_cfg, "hold_base_vel_scale", 1.0)))
                if (use_pkg_hold and bool(node.bb.get("package_attached", False)))
                else right_transport_xy
            )
            left_target_xy = _predict_world_target_from_body_velocity(left_base_pose, left_cmd_xy, transport_time)
            right_target_xy = _predict_world_target_from_body_velocity(right_base_pose, right_cmd_xy, transport_time)
            dst_source = f"legacy_profile({dst_source})"

        kp_xy = float(getattr(man_cfg, "transport_retreat_kp_x", 1.0))
        ok = (
            _init_transport_base_arm_hold(
                left_target_xy, right_target_xy, left_arm_jp, right_arm_jp, period_s=period_s, kp_xy=kp_xy
            )
            if lock_arms
            else _init_tp_base_stage(
                node,
                left_base_goal_xy=left_target_xy,
                right_base_goal_xy=right_target_xy,
                period_s=float(max(period_s, 0.2)),
                kp_xy=kp_xy,
                kp_yaw=0.0,
            )
        )
        if ok:
            node.bb["movebase_stage"] = "transport"
            node.bb["movebase_transport_left_target_xy"] = left_target_xy
            node.bb["movebase_transport_right_target_xy"] = right_target_xy
            node.bb["movebase_transport_source"] = str(dst_source) + (",arm_lock" if lock_arms else "")
            node.get_logger().info(
                bt_fmt(
                    "[MoveBase] transport target initialized "
                    f"(src={dst_source}) "
                    f"L={np.round(np.asarray(left_target_xy), 3).tolist()}, "
                    f"R={np.round(np.asarray(right_target_xy), 3).tolist()}"
                )
            )
            return True
        return False

    t0 = node.get_action_timer("MoveBase")
    if t0 is None:
        if not _phase_pause_gate(node, "MoveBase"):
            node.stop_all_movement()
            return None
        node.get_logger().info(bt_fmt(f"[MoveBase] start ({transport_time}s)"))
        _reset_movebase_runtime(node)

        if use_retreat:
            live_state = _get_live_tp_state(node)
            if live_state is not None:
                _la, _ra, left_base, right_base = live_state
                pkg_xyz = _resolve_pkg_reference_xyz(
                    node,
                    left_arm_jp=_la,
                    right_arm_jp=_ra,
                    left_base=left_base,
                    right_base=right_base,
                )
                if pkg_xyz is None:
                    pkg_xyz = _get_live_package_xyz(node)
            else:
                pkg_xyz = None
            if live_state is not None and pkg_xyz is not None:
                retreat_y = float(pkg_xyz[1]) - float(getattr(man_cfg, "transport_retreat_pkg_backoff_y", 2.0))
                left_target = [
                    float(left_base[0]) + float(getattr(man_cfg, "transport_retreat_left_offset_x", 0.0)),
                    retreat_y,
                ]
                right_target = [
                    float(right_base[0]) + float(getattr(man_cfg, "transport_retreat_right_offset_x", 0.0)),
                    retreat_y,
                ]
                node.bb["movebase_retreat_left_target_xy"] = left_target
                node.bb["movebase_retreat_right_target_xy"] = right_target
                ok_retreat = (
                    _init_transport_base_arm_hold(
                        left_target,
                        right_target,
                        _la,
                        _ra,
                        period_s=float(max(getattr(man_cfg, "transport_retreat_stage_timeout", 12.0), 0.5)),
                        kp_xy=float(getattr(man_cfg, "transport_retreat_kp_x", 1.0)),
                    )
                    if lock_arms
                    else _init_tp_base_stage(
                        node,
                        left_base_goal_xy=left_target,
                        right_base_goal_xy=right_target,
                        period_s=float(max(getattr(man_cfg, "transport_retreat_stage_timeout", 12.0), 0.5)),
                        kp_xy=float(getattr(man_cfg, "transport_retreat_kp_x", 1.0)),
                        kp_yaw=0.0,
                    )
                )
                if ok_retreat:
                    node.bb["movebase_stage"] = "retreat"
                    node.get_logger().info(
                        bt_fmt(
                            "[MoveBase] retreat stage initialized "
                            f"(L_target={np.round(np.asarray(left_target), 3).tolist()}, "
                            f"R_target={np.round(np.asarray(right_target), 3).tolist()}, "
                            f"pkg_y={float(pkg_xyz[1]):.3f})"
                    )
                    )
                else:
                    if _init_transport_stage(_la, _ra, left_base, right_base, period_s=transport_time):
                        node.get_logger().warn(
                            bt_fmt("[MoveBase] retreat TP init failed, fallback to direct transport stage")
                        )
                    else:
                        node.bb["movebase_stage"] = "transport"
                        node._warn_throttled(
                            "movebase_retreat_init_tp_fail",
                            bt_fmt("[MoveBase] retreat TP init failed and transport init failed"),
                            period_s=2.0,
                        )
            else:
                node.bb["movebase_stage"] = "transport"
                if live_state is not None:
                    _la, _ra, left_base, right_base = live_state
                    _init_transport_stage(_la, _ra, left_base, right_base, period_s=transport_time)
                node._warn_throttled(
                    "movebase_retreat_init_fallback",
                    bt_fmt("[MoveBase] retreat stage skipped (missing live state/package pose), fallback to transport"),
                    period_s=2.0,
                )
        else:
            live_state = _get_live_tp_state(node)
            if live_state is not None:
                _la, _ra, left_base, right_base = live_state
                if not _init_transport_stage(_la, _ra, left_base, right_base, period_s=transport_time):
                    node.bb["movebase_stage"] = "transport"
            else:
                node.bb["movebase_stage"] = "transport"
        t0 = node.start_action_timer("MoveBase")

    stage = str(node.bb.get("movebase_stage", "transport"))
    now_s = _ros_now_s(node)

    if stage == "retreat":
        live_state = _get_live_tp_state(node)
        if live_state is None:
            node._warn_throttled(
                "movebase_retreat_wait_data",
                bt_fmt("[MoveBase] retreat waiting sensor data (joint_states/odom)"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        left_target = node.bb.get("movebase_retreat_left_target_xy", None)
        right_target = node.bb.get("movebase_retreat_right_target_xy", None)
        if (not isinstance(left_target, (list, tuple))) or (not isinstance(right_target, (list, tuple))):
            node._warn_throttled(
                "movebase_retreat_invalid_targets",
                bt_fmt("[MoveBase] retreat targets unavailable, switching to transport"),
                period_s=2.0,
            )
            node.bb["movebase_stage"] = "transport"
            if not _init_transport_stage(left_arm_jp, right_arm_jp, left_base, right_base, period_s=transport_time):
                node._warn_throttled(
                    "movebase_transport_init_after_retreat_invalid",
                    bt_fmt("[MoveBase] transport TP init failed after invalid retreat target"),
                    period_s=2.0,
                )
            node.start_action_timer("MoveBase")
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        _execute_tp_full_control(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
            arm_clip_abs=movebase_arm_clip_abs,
            base_xy_abs_max=float(getattr(man_cfg, "transport_retreat_cmd_xy_abs_max", 0.20)),
            base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
        )

        l_dist = float(math.hypot(float(left_target[0]) - float(left_base[0]), float(left_target[1]) - float(left_base[1])))
        r_dist = float(math.hypot(float(right_target[0]) - float(right_base[0]), float(right_target[1]) - float(right_base[1])))
        tol = float(getattr(man_cfg, "transport_retreat_goal_tol", 0.10))
        l_reached = bool(l_dist <= tol)
        r_reached = bool(r_dist <= tol)

        if use_pkg_hold and bool(node.bb.get("package_attached", False)) and (not lock_arms):
            if _replan_pkg_hold_tp(
                node,
                left_arm_jp,
                right_arm_jp,
                left_base,
                right_base,
                pkg_z_target=node._pkg_hold_target_z,
                preserve_jtc_base=True,
            ):
                _log_force_proxy(node, "retreat", period_s=1.0)
        if use_pkg_hold and bool(node.bb.get("package_attached", False)):
            _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "retreat")

        retreat_elapsed = now_s - float(t0)
        retreat_timeout = float(max(0.5, float(getattr(man_cfg, "transport_retreat_stage_timeout", 12.0))))
        retreat_reached = bool(l_reached and r_reached)
        node._info_throttled(
            "movebase_retreat_track",
            bt_fmt(
                f"[MoveBase] retreat tracking L_dist={l_dist:.3f}, R_dist={r_dist:.3f}, "
                f"reached={retreat_reached}, elapsed={retreat_elapsed:.2f}/{retreat_timeout:.2f}s"
            ),
            period_s=1.0,
        )

        if retreat_reached or retreat_elapsed >= retreat_timeout:
            node.stop_all_movement()
            node.bb["movebase_stage"] = "transport"
            if not _init_transport_stage(left_arm_jp, right_arm_jp, left_base, right_base, period_s=transport_time):
                node._warn_throttled(
                    "movebase_transport_init_tp_fail",
                    bt_fmt("[MoveBase] transport TP init failed"),
                    period_s=2.0,
                )
            node.start_action_timer("MoveBase")
            if retreat_elapsed >= retreat_timeout and (not retreat_reached):
                node.get_logger().warn(bt_fmt("[MoveBase] retreat timeout, continuing with transport segment"))
            node.get_logger().info(bt_fmt("[MoveBase] retreat completed, starting transport segment"))
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        rclpy.spin_once(node, timeout_sec=0.01)
        return None

    # Stage 2: transport point-to-point via TP
    if stage == "transport" and (now_s - float(t0) < max(transport_time, 0.2) + 2.0):
        live_state = _get_live_tp_state(node)
        if live_state is None:
            node._warn_throttled(
                "movebase_transport_wait_data",
                bt_fmt("[MoveBase] transport waiting sensor data (joint_states/odom)"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        left_arm_jp, right_arm_jp, left_base, right_base = live_state

        # Se siamo entrati in transport senza target inizializzati, inizializza ora.
        l_target_ref = node.bb.get("movebase_transport_left_target_xy", None)
        r_target_ref = node.bb.get("movebase_transport_right_target_xy", None)
        if (not isinstance(l_target_ref, (list, tuple))) or (not isinstance(r_target_ref, (list, tuple))):
            if not _init_transport_stage(left_arm_jp, right_arm_jp, left_base, right_base, period_s=transport_time):
                node._warn_throttled(
                    "movebase_transport_lazy_init_fail",
                    bt_fmt("[MoveBase] transport target init failed"),
                    period_s=2.0,
                )
                rclpy.spin_once(node, timeout_sec=0.01)
                return None

        _execute_tp_full_control(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
            arm_clip_abs=movebase_arm_clip_abs,
            base_xy_abs_max=float(getattr(man_cfg, "transport_retreat_cmd_xy_abs_max", 0.20)),
            base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
        )

        left_target = node.bb.get("movebase_transport_left_target_xy", None)
        right_target = node.bb.get("movebase_transport_right_target_xy", None)
        l_dist = float("inf")
        r_dist = float("inf")
        if isinstance(left_target, (list, tuple)) and len(left_target) >= 2:
            l_dist = float(math.hypot(float(left_target[0]) - float(left_base[0]), float(left_target[1]) - float(left_base[1])))
        if isinstance(right_target, (list, tuple)) and len(right_target) >= 2:
            r_dist = float(math.hypot(float(right_target[0]) - float(right_base[0]), float(right_target[1]) - float(right_base[1])))
        tol = float(getattr(man_cfg, "transport_destination_goal_tol", 0.10))
        transport_reached = bool(l_dist <= tol and r_dist <= tol)

        arm_hold_ok = True
        l_arm_err = float("nan")
        r_arm_err = float("nan")
        if lock_arms:
            q_left_goal = node.bb.get("movebase_transport_arm_goal_left", None)
            q_right_goal = node.bb.get("movebase_transport_arm_goal_right", None)
            l_arm_err = _max_joint_error(node, left_arm_jp, q_left_goal)
            r_arm_err = _max_joint_error(node, right_arm_jp, q_right_goal)
            arm_hold_ok = bool(
                np.isfinite(l_arm_err)
                and np.isfinite(r_arm_err)
                and l_arm_err <= float(node.approach_arm_joint_tol)
                and r_arm_err <= float(node.approach_arm_joint_tol)
            )
            transport_reached = bool(transport_reached and arm_hold_ok)

        if use_pkg_hold and bool(node.bb.get("package_attached", False)) and (not lock_arms):
            if _replan_pkg_hold_tp(
                node,
                left_arm_jp,
                right_arm_jp,
                left_base,
                right_base,
                pkg_z_target=node._pkg_hold_target_z,
                preserve_jtc_base=True,
            ):
                _log_force_proxy(node, "transport", period_s=1.0)
        if use_pkg_hold and bool(node.bb.get("package_attached", False)):
            _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "transport")
        node._info_throttled(
            "movebase_transport_track",
            bt_fmt(
                f"[MoveBase] transport tracking src={node.bb.get('movebase_transport_source', 'n/a')}, "
                f"L_dist={l_dist:.3f}, R_dist={r_dist:.3f}, "
                f"arm_lock={lock_arms}, arm_ok={arm_hold_ok}, "
                f"L_arm_err={l_arm_err:.3f}, R_arm_err={r_arm_err:.3f}, reached={transport_reached}"
            ),
            period_s=1.0,
        )
        transport_elapsed = now_s - float(t0)
        transport_timeout = float(max(transport_time, 0.2) + 2.0)
        if transport_reached or (transport_elapsed >= transport_timeout):
            node.stop_all_movement()
            node.clear_action_timer("MoveBase")
            _reset_movebase_runtime(node)
            _phase_pause_reset(node, "MoveBase")
            if transport_elapsed >= transport_timeout and (not transport_reached):
                node.get_logger().warn(bt_fmt("[MoveBase] transport timeout reached, continuing"))
            node.get_logger().info(bt_fmt("[MoveBase] completed"))
            return True
        rclpy.spin_once(node, timeout_sec=0.01)
        return None

    node.stop_all_movement()
    node.clear_action_timer("MoveBase")
    _reset_movebase_runtime(node)
    _phase_pause_reset(node, "MoveBase")
    node.get_logger().info(bt_fmt("[MoveBase] completed"))
    return True


def Drop():
    """
    Calata e posizionamento del pacco in zona place:
    - basi + bracci verso pose di place per il tempo configurato.

    Mappa la fase 'discesa + place'.
    """
    node = _require_node()
    phase_cfg = node.cfg.phases
    motion = node.cfg.motion_profiles
    man_cfg = node.cfg.manipulation
    descend_and_place_time = float(phase_cfg.descend_and_place_time)
    place_left_base_xy = _float_vec(motion.place_left_base_xy_vel)
    place_right_base_xy = _float_vec(motion.place_right_base_xy_vel)
    left_arm_place = _float_vec(motion.left_arm_place)
    right_arm_place = _float_vec(motion.right_arm_place)
    use_pkg_hold = bool(man_cfg.enable_object_centric_hold) and bool(node.bb.get("package_attached", False))

    t0 = node.get_action_timer("Drop")
    if t0 is None:
        if not _phase_pause_gate(node, "Drop"):
            return None
        live_state = _get_live_tp_state(node)
        if live_state is None:
            node._warn_throttled("drop_wait_data_init", bt_fmt("[Drop] waiting sensor data before init"), period_s=1.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        node.get_logger().info(bt_fmt(f"[Drop] start TP ({descend_and_place_time}s)"))

        left_target_xy = _predict_world_target_from_body_velocity(left_base, place_left_base_xy, descend_and_place_time)
        right_target_xy = _predict_world_target_from_body_velocity(right_base, place_right_base_xy, descend_and_place_time)
        node.bb["drop_left_target_xy"] = left_target_xy
        node.bb["drop_right_target_xy"] = right_target_xy
        node.bb["drop_arm_goal_left"] = None
        node.bb["drop_arm_goal_right"] = None

        if not _init_tp_base_stage(
            node,
            left_base_goal_xy=left_target_xy,
            right_base_goal_xy=right_target_xy,
            period_s=float(max(descend_and_place_time, 0.2)),
            kp_xy=float(getattr(man_cfg, "transport_retreat_kp_x", 1.0)),
            kp_yaw=0.0,
        ):
            node.get_logger().warn(bt_fmt("[Drop] unable to initialize TP base stage"))
            return False

        if use_pkg_hold:
            if node._pkg_hold_start_z is not None:
                node._pkg_hold_target_z = (
                    float(node._pkg_hold_start_z)
                    + float(man_cfg.collect_lift_delta_z)
                    - float(man_cfg.drop_delta_z)
                )
        else:
            q_left_goal = np.asarray(left_arm_jp, dtype=np.float32) + np.asarray(left_arm_place, dtype=np.float32)
            q_right_goal = np.asarray(right_arm_jp, dtype=np.float32) + np.asarray(right_arm_place, dtype=np.float32)
            q_left_goal, q_right_goal = _clip_descend_pick_joint_goals(node, q_left_goal, q_right_goal)
            node.bb["drop_arm_goal_left"] = [float(v) for v in q_left_goal.tolist()]
            node.bb["drop_arm_goal_right"] = [float(v) for v in q_right_goal.tolist()]
            _init_tp_arm_joint_stage(node, q_left_goal, q_right_goal, period_s=float(max(descend_and_place_time, 0.2)))

        t0 = node.start_action_timer("Drop")

    live_state = _get_live_tp_state(node)
    if live_state is None:
        node._warn_throttled("drop_wait_data", bt_fmt("[Drop] waiting sensor data (joint_states/odom)"), period_s=1.0)
        rclpy.spin_once(node, timeout_sec=0.01)
        return None

    left_arm_jp, right_arm_jp, left_base, right_base = live_state
    _execute_tp_full_control(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
        arm_clip_abs=float(getattr(man_cfg, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
        base_xy_abs_max=float(getattr(man_cfg, "transport_retreat_cmd_xy_abs_max", 0.20)),
        base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
    )

    if use_pkg_hold:
        if _replan_pkg_hold_tp(
            node,
            left_arm_jp,
            right_arm_jp,
            left_base,
            right_base,
            pkg_z_target=node._pkg_hold_target_z,
            preserve_jtc_base=True,
        ):
            _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "drop")
            _log_force_proxy(node, "drop", period_s=1.0)

    left_target_xy = node.bb.get("drop_left_target_xy", None)
    right_target_xy = node.bb.get("drop_right_target_xy", None)
    l_dist = float("inf")
    r_dist = float("inf")
    if isinstance(left_target_xy, (list, tuple)) and len(left_target_xy) >= 2:
        l_dist = float(math.hypot(float(left_target_xy[0]) - float(left_base[0]), float(left_target_xy[1]) - float(left_base[1])))
    if isinstance(right_target_xy, (list, tuple)) and len(right_target_xy) >= 2:
        r_dist = float(math.hypot(float(right_target_xy[0]) - float(right_base[0]), float(right_target_xy[1]) - float(right_base[1])))
    tol = float(getattr(man_cfg, "transport_retreat_goal_tol", 0.10))
    base_reached = bool(l_dist <= tol and r_dist <= tol)

    arm_reached = True
    if not use_pkg_hold:
        q_left_goal = node.bb.get("drop_arm_goal_left", None)
        q_right_goal = node.bb.get("drop_arm_goal_right", None)
        l_err = _max_joint_error(node, left_arm_jp, q_left_goal)
        r_err = _max_joint_error(node, right_arm_jp, q_right_goal)
        arm_reached = bool(
            np.isfinite(l_err)
            and np.isfinite(r_err)
            and l_err <= float(node.approach_arm_joint_tol)
            and r_err <= float(node.approach_arm_joint_tol)
        )

    elapsed = _ros_now_s(node) - float(t0)
    timeout = float(max(descend_and_place_time, 0.2) + 2.0)
    reached = bool(base_reached and arm_reached)
    node._info_throttled(
        "drop_track",
        bt_fmt(
            f"[Drop] tracking base_reached={base_reached} arm_reached={arm_reached} "
            f"L_dist={l_dist:.3f} R_dist={r_dist:.3f} elapsed={elapsed:.2f}/{timeout:.2f}s"
        ),
        period_s=1.0,
    )

    if reached or elapsed >= timeout:
        node.stop_all_movement()
        node.clear_action_timer("Drop")
        node.bb.pop("drop_left_target_xy", None)
        node.bb.pop("drop_right_target_xy", None)
        node.bb.pop("drop_arm_goal_left", None)
        node.bb.pop("drop_arm_goal_right", None)
        _phase_pause_reset(node, "Drop")
        if elapsed >= timeout and (not reached):
            node.get_logger().warn(bt_fmt("[Drop] timeout reached, continuing"))
        node.get_logger().info(bt_fmt("[Drop] completed"))
        return True

    rclpy.spin_once(node, timeout_sec=0.01)
    return None


def Release():
    """
    Release dell'oggetto:
    - movimento bracci verso pose di release per il tempo configurato
    - detach del pacco (link-attacher)

    Mappa la fase 'release' della demo.
    """
    node = _require_node()
    phase_cfg = node.cfg.phases
    motion = node.cfg.motion_profiles
    release_time = float(phase_cfg.release_time)
    collect_time = float(phase_cfg.collect_time)
    retreat_left_base_xy = _float_vec(motion.collect_left_base_xy_vel)
    retreat_right_base_xy = _float_vec(motion.collect_right_base_xy_vel)
    man_cfg = node.cfg.manipulation

    phase_key = "release_phase"
    phase = str(node.bb.get(phase_key, "open_detach")).strip().lower()
    if phase == "release":
        phase = "open_detach"
        node.bb[phase_key] = phase

    # ----------------------------------------------------------------------
    # Fase 1: apertura palette (TP EE) + detach pacco
    # ----------------------------------------------------------------------
    if phase == "open_detach":
        t0 = node.get_action_timer("Release")
        if t0 is None:
            if not _phase_pause_gate(node, "Release"):
                return None
            live_state = _get_live_tp_state(node)
            if live_state is None:
                node._warn_throttled("release_wait_data_init", bt_fmt("[Release] waiting sensor data before init"), period_s=1.0)
                rclpy.spin_once(node, timeout_sec=0.01)
                return None
            left_arm_jp, right_arm_jp, left_base, right_base = live_state
            ee_live = node._get_live_ee_by_side(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            )
            left_ee = ee_live.get("left", None)
            right_ee = ee_live.get("right", None)
            if left_ee is None or right_ee is None:
                node._warn_throttled("release_no_ee_init", bt_fmt("[Release] live EE unavailable"), period_s=1.0)
                rclpy.spin_once(node, timeout_sec=0.01)
                return None

            pkg_xyz = _get_live_package_xyz(node)
            if pkg_xyz is None:
                pkg_xyz = _resolve_pkg_reference_xyz(node, left_arm_jp, right_arm_jp, left_base, right_base)
            if pkg_xyz is None:
                node._warn_throttled("release_no_pkg_init", bt_fmt("[Release] package pose unavailable"), period_s=1.0)
                rclpy.spin_once(node, timeout_sec=0.01)
                return None

            left_rpy_closed = _default_rpy_for_side(node, "left", left_ee)
            right_rpy_closed = _default_rpy_for_side(node, "right", right_ee)
            axis_idx = _pick_open_axis_idx(getattr(man_cfg, "pick_open_axis", "pitch"))
            left_rpy_open = np.asarray(left_rpy_closed, dtype=np.float32).copy()
            right_rpy_open = np.asarray(right_rpy_closed, dtype=np.float32).copy()
            left_rpy_open[axis_idx] = float(left_rpy_open[axis_idx]) + float(getattr(man_cfg, "pick_open_angle_rad_left", math.pi / 2.0))
            right_rpy_open[axis_idx] = float(right_rpy_open[axis_idx]) + float(getattr(man_cfg, "pick_open_angle_rad_right", math.pi / 2.0))

            left_goal = np.zeros((6,), dtype=np.float32)
            right_goal = np.zeros((6,), dtype=np.float32)
            left_goal[0] = float(pkg_xyz[0]) + float(getattr(man_cfg, "pick_mid_left_offset_x", -0.45))
            left_goal[1] = float(pkg_xyz[1]) + float(getattr(man_cfg, "pick_grasp_offset_y", -0.10))
            left_goal[2] = float(pkg_xyz[2]) + float(getattr(man_cfg, "pick_grasp_offset_z", 0.0)) + 0.02
            left_goal[3:6] = left_rpy_open
            right_goal[0] = float(pkg_xyz[0]) + float(getattr(man_cfg, "pick_mid_right_offset_x", 0.45))
            right_goal[1] = float(pkg_xyz[1]) + float(getattr(man_cfg, "pick_grasp_offset_y", -0.10))
            right_goal[2] = float(pkg_xyz[2]) + float(getattr(man_cfg, "pick_grasp_offset_z", 0.0)) + 0.02
            right_goal[3:6] = right_rpy_open

            if not _set_tp_ee_traj(
                node,
                left_ee_now=left_ee,
                right_ee_now=right_ee,
                left_ee_goal=left_goal,
                right_ee_goal=right_goal,
                traj_time=float(max(release_time, 0.2)),
            ):
                node.get_logger().warn(bt_fmt("[Release] unable to set TP EE trajectory for release-open"))
                return False

            node.bb["release_open_goal_left"] = [float(v) for v in left_goal.tolist()]
            node.bb["release_open_goal_right"] = [float(v) for v in right_goal.tolist()]
            node.get_logger().info(bt_fmt(f"[Release] start TP open+detach ({release_time}s)"))
            t0 = node.start_action_timer("Release")

        live_state = _get_live_tp_state(node)
        if live_state is None:
            node._warn_throttled("release_wait_data", bt_fmt("[Release] waiting sensor data"), period_s=1.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        _execute_tp_arm_control(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
            arm_clip_abs=float(getattr(man_cfg, "pick_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
        )

        ee_live = node._get_live_ee_by_side(
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
        )
        l_goal = node.bb.get("release_open_goal_left", None)
        r_goal = node.bb.get("release_open_goal_right", None)
        l_ok, l_pos, l_ori = _ee_goal_reached(
            node, ee_live.get("left", None), l_goal,
            pos_tol=float(getattr(man_cfg, "pick_pos_tol", 0.03)),
            ori_tol=float(getattr(man_cfg, "pick_ori_tol", 0.20)),
        )
        r_ok, r_pos, r_ori = _ee_goal_reached(
            node, ee_live.get("right", None), r_goal,
            pos_tol=float(getattr(man_cfg, "pick_pos_tol", 0.03)),
            ori_tol=float(getattr(man_cfg, "pick_ori_tol", 0.20)),
        )
        elapsed = _ros_now_s(node) - float(t0)
        timeout = float(max(release_time, 0.2) + 0.8)
        open_reached = bool(l_ok and r_ok)
        node._info_throttled(
            "release_open_track",
            bt_fmt(
                f"[Release] open tracking reached={open_reached}, "
                f"L(pos={l_pos:.3f},ori={l_ori:.3f}), R(pos={r_pos:.3f},ori={r_ori:.3f}), "
                f"elapsed={elapsed:.2f}/{timeout:.2f}s"
            ),
            period_s=1.0,
        )
        if (not open_reached) and elapsed < timeout:
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        node.stop_all_movement()
        _detach_package_from_arms(node)
        node.bb["package_attached"] = False
        if node.bb.pop("package_gravity_disabled", False):
            set_package_gravity(node, True)
        _reset_pkg_hold_runtime(node)

        node.clear_action_timer("Release")
        node.bb.pop("release_open_goal_left", None)
        node.bb.pop("release_open_goal_right", None)
        node.bb[phase_key] = "retreat_home"
        node.get_logger().info(bt_fmt("[Release] detach done, starting TP retreat+home"))
        return None

    # ----------------------------------------------------------------------
    # Fase 2: risalita a home + arretramento basi (TP joint+base)
    # ----------------------------------------------------------------------
    retreat_t0 = node.get_action_timer("ReleaseRetreat")
    if retreat_t0 is None:
        # Passaggio da fase EE (open) a fase JTC (retreat+home):
        # evita conflitti tenendo attivo solo il controller JTC.
        if node.ee_task is not None:
            try:
                if hasattr(node.ee_task, "set_trajectory_plan"):
                    node.ee_task.set_trajectory_plan(None)
            except Exception:
                pass
            try:
                if hasattr(node.ee_task, "deactivate"):
                    node.ee_task.deactivate()
            except Exception:
                pass

        live_state = _get_live_tp_state(node)
        if live_state is None:
            node._warn_throttled("release_retreat_wait_data_init", bt_fmt("[Release] waiting sensor data before retreat init"), period_s=1.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        left_home, _ = node._resolve_side_home_joint_goal("left", left_arm_jp)
        right_home, _ = node._resolve_side_home_joint_goal("right", right_arm_jp)
        left_target_xy = _predict_world_target_from_body_velocity(left_base, retreat_left_base_xy, collect_time)
        right_target_xy = _predict_world_target_from_body_velocity(right_base, retreat_right_base_xy, collect_time)
        node.bb["release_retreat_left_target_xy"] = left_target_xy
        node.bb["release_retreat_right_target_xy"] = right_target_xy
        node.bb["release_home_goal_left"] = [float(v) for v in np.asarray(left_home, dtype=np.float32).tolist()]
        node.bb["release_home_goal_right"] = [float(v) for v in np.asarray(right_home, dtype=np.float32).tolist()]
        ok_base = _init_tp_base_stage(
            node,
            left_base_goal_xy=left_target_xy,
            right_base_goal_xy=right_target_xy,
            period_s=float(max(collect_time, 0.2)),
            kp_xy=float(getattr(man_cfg, "transport_retreat_kp_x", 1.0)),
            kp_yaw=0.0,
        )
        ok_arm = _init_tp_arm_joint_stage(
            node,
            left_home,
            right_home,
            period_s=float(max(collect_time, 0.2)),
        )
        # Release fix: assicurati che base+arm restino attivi insieme nello stesso JTC stage.
        if bool(ok_base and ok_arm) and (node.approach_jtc_task is not None):
            try:
                node.approach_jtc_task.activate()
                node.approach_jtc_task.set_activation("base", True)
                node.approach_jtc_task.set_activation("arm", True)
            except Exception as exc:
                node._warn_throttled(
                    "release_retreat_activation_fix",
                    bt_fmt(f"[Release] unable to enforce base+arm activation: {exc}"),
                    period_s=2.0,
                )
        retreat_t0 = node.start_action_timer("ReleaseRetreat")

    live_state = _get_live_tp_state(node)
    if live_state is None:
        node._warn_throttled("release_retreat_wait_data", bt_fmt("[Release] waiting sensor data in retreat"), period_s=1.0)
        rclpy.spin_once(node, timeout_sec=0.01)
        return None
    left_arm_jp, right_arm_jp, left_base, right_base = live_state
    _execute_tp_full_control(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
        arm_clip_abs=float(getattr(man_cfg, "pre_transport_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
        base_xy_abs_max=float(getattr(man_cfg, "transport_retreat_cmd_xy_abs_max", 0.20)),
        base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
    )

    left_target_xy = node.bb.get("release_retreat_left_target_xy", None)
    right_target_xy = node.bb.get("release_retreat_right_target_xy", None)
    l_dist = float("inf")
    r_dist = float("inf")
    if isinstance(left_target_xy, (list, tuple)) and len(left_target_xy) >= 2:
        l_dist = float(math.hypot(float(left_target_xy[0]) - float(left_base[0]), float(left_target_xy[1]) - float(left_base[1])))
    if isinstance(right_target_xy, (list, tuple)) and len(right_target_xy) >= 2:
        r_dist = float(math.hypot(float(right_target_xy[0]) - float(right_base[0]), float(right_target_xy[1]) - float(right_base[1])))
    tol = float(getattr(man_cfg, "transport_retreat_goal_tol", 0.10))
    base_reached = bool(l_dist <= tol and r_dist <= tol)

    q_left_goal = node.bb.get("release_home_goal_left", None)
    q_right_goal = node.bb.get("release_home_goal_right", None)
    l_err = _max_joint_error(node, left_arm_jp, q_left_goal)
    r_err = _max_joint_error(node, right_arm_jp, q_right_goal)
    arm_reached = bool(
        np.isfinite(l_err)
        and np.isfinite(r_err)
        and l_err <= float(node.approach_arm_joint_tol)
        and r_err <= float(node.approach_arm_joint_tol)
    )

    elapsed = _ros_now_s(node) - float(retreat_t0)
    timeout = float(max(collect_time, 0.2) + 2.0)
    reached = bool(base_reached and arm_reached)
    node._info_throttled(
        "release_retreat_track",
        bt_fmt(
            f"[Release] retreat tracking base={base_reached} arm={arm_reached} "
            f"L_dist={l_dist:.3f} R_dist={r_dist:.3f} "
            f"L_err={l_err:.3f} R_err={r_err:.3f} elapsed={elapsed:.2f}/{timeout:.2f}s"
        ),
        period_s=1.0,
    )
    if reached or elapsed >= timeout:
        node.stop_all_movement()
        node.clear_action_timer("ReleaseRetreat")
        node.bb.pop("release_retreat_left_target_xy", None)
        node.bb.pop("release_retreat_right_target_xy", None)
        node.bb.pop("release_home_goal_left", None)
        node.bb.pop("release_home_goal_right", None)
        node.bb.pop(phase_key, None)
        _phase_pause_reset(node, "Release")
        if elapsed >= timeout and (not reached):
            node.get_logger().warn(bt_fmt("[Release] retreat timeout, finishing anyway"))
        node.get_logger().info(bt_fmt("[Release] completed"))
        return True

    rclpy.spin_once(node, timeout_sec=0.01)
    return None
