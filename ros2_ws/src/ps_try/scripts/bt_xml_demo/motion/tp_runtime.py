#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""TaskPrioritization runtime helpers for BT motion actions."""

from __future__ import annotations

import math

import numpy as np

from TaskPrioritization.Trajectories.trajectory import Trajectory

from bt_xml_demo.bt_action_context import (
    bt_fmt,
)
from bt_xml_demo.cmd_utils import (
    sanitize_arm_cmd as _sanitize_arm_cmd,
    sanitize_base_cmd as _sanitize_base_cmd,
)
from bt_xml_demo.motion.io_timing import (
    _dt_from_ros_time,
    _publish_arm_cmd,
    _publish_base_cmd,
    _ros_now_s,
)
from bt_xml_demo.motion.pose_targets import (
    _get_live_package_pose6,
    _get_live_package_xyz,
    _normalize_yaw,
    _offset_xy_in_target_frame,
)


def _build_base_cmd_to_xy(
    node,
    base_pose,
    target_xy,
    kp_x: float,
    kp_y: float,
    xy_abs_max: float,
    goal_tol: float | None = None,
):
    """Simple P controller for base velocity to track XY target."""
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


def _execute_tp_arm_control(node, left_arm_jp, right_arm_jp, left_base, right_base, arm_clip_abs: float):
    """Execute TP in arm-only mode: arms move, bases stay stationary."""
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
    _publish_base_cmd(node, left_cmd=[0.0, 0.0, 0.0], right_cmd=[0.0, 0.0, 0.0])
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
    """Execute TP in full-control mode: both bases and arms move together."""
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
    """Shortcut for arm-only TP with hold-phase clipping."""
    hold_clip = float(getattr(node.cfg.manipulation, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max))
    return _execute_tp_arm_control(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
        arm_clip_abs=hold_clip,
    )


def _init_tp_base_stage(
    node,
    left_base_goal_xy,
    right_base_goal_xy,
    period_s: float,
    kp_xy: float = 1.0,
    kp_yaw: float = 0.0,
    arm_active: bool = False,
) -> bool:
    """Initialize JTC for base-only stage with omni targets."""
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
        node.approach_jtc_task.set_activation("arm", bool(arm_active))
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
    """Initialize JTC for joint-space arm trajectory (base inactive)."""
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
    """Configure Cartesian EE task for waypoint trajectories."""
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
    """Map nominal roll/pitch/yaw axis name to RPY index."""
    key = str(axis_name).strip().lower()
    if key == "roll":
        return 0
    if key == "yaw":
        return 2
    return 1


def _ee_goal_reached(node, ee_live, ee_goal, pos_tol: float, ori_tol: float):
    """Test Cartesian convergence (position + orientation) for an end-effector."""
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
    """Select RPY target for side (fixed from config or keep live state)."""
    if str(node.approach_ee_orient_mode).lower() == "fixed":
        rpy_goal = node.approach_ee_left_rpy_goal if side == "left" else node.approach_ee_right_rpy_goal
        if rpy_goal is not None and len(rpy_goal) == 3:
            return np.asarray(rpy_goal, dtype=np.float32)
    if isinstance(ee_live, np.ndarray) and ee_live.shape[0] >= 6:
        return np.asarray(ee_live[3:6], dtype=np.float32)
    return np.zeros((3,), dtype=np.float32)


def _build_pick_waypoint_stage_plan(node, left_arm_jp, right_arm_jp, left_base, right_base):
    """Generate multi-stage pick approach trajectory (open -> descend -> grasp)."""
    man_cfg = node.cfg.manipulation
    pkg_pose6 = _get_live_package_pose6(node)
    if pkg_pose6 is None:
        pkg_xyz = _get_live_package_xyz(node)
        pkg_yaw = 0.0
    else:
        pkg_xyz = [float(pkg_pose6[0]), float(pkg_pose6[1]), float(pkg_pose6[2])]
        pkg_yaw = _normalize_yaw(float(pkg_pose6[5]))
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
        gx, gy = _offset_xy_in_target_frame(
            [float(pkg_xyz[0]), float(pkg_xyz[1])],
            float(x_off),
            float(y_off),
            float(pkg_yaw),
        )
        g[0] = float(gx)
        g[1] = float(gy)
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


def _max_joint_error(node, q_now, q_goal) -> float:
    """Compute maximum joint-space error (largest angle difference)."""
    if q_now is None or q_goal is None:
        return float("inf")
    if len(q_now) != len(q_goal):
        return float("inf")
    errs = [abs(float(node._angle_diff(float(q_now[i]), float(q_goal[i])))) for i in range(len(q_now))]
    return float(max(errs)) if errs else 0.0


def _clip_descend_pick_joint_goals(node, q_left_goal: np.ndarray, q_right_goal: np.ndarray):
    """Clip pick/descent targets within joint limit safety margins."""
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
    """Initialize JTC for descend/pick phase in joint-space."""
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
    """Initialize JTC for pre-transport pose (joint-space arm motion)."""
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


__all__ = [
    "_build_base_cmd_to_xy",
    "_execute_tp_arm_control",
    "_execute_tp_full_control",
    "_execute_tp_arm_hold",
    "_init_tp_base_stage",
    "_init_tp_arm_joint_stage",
    "_set_tp_ee_traj",
    "_pick_open_axis_idx",
    "_ee_goal_reached",
    "_default_rpy_for_side",
    "_build_pick_waypoint_stage_plan",
    "_max_joint_error",
    "_clip_descend_pick_joint_goals",
    "_init_descend_pick_tp",
    "_init_pre_transport_tp",
]
