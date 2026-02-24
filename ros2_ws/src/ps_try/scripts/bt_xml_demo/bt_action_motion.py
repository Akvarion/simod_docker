#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Motion BehaviorTree Actions for Dual-Arm Robotic Manipulation (SIMOD).

This module provides high-level BehaviorTree action implementations for a dual-arm mobile
manipulation system (SRM). The actions coordinate motion across multiple subsystems:
- Dual base platforms (left/right summit robots)
- Dual arms (UR manipulators with grippers)
- Package manipulation with optional object-centric hold control
- Trajectory Planning (TP) integration for coordinated motion

Main Actions:
- ApproachObject: Approach pallet in TP mode (base + arm coordination)
- LiftObj: Pick and lift sequence (descend → pick → attach → collect)
- MoveBase: Transport pallet (retreat → transfer to destination)
- Drop: Descent and placement (base align → descend to release)
- Release: Detach and retreat (open palette → detach → home)

Key Features:
- Object-centric hold control (maintains package pose during transport)
- Waypoint-based EE trajectories (pick approach stages)
- Package-centric base alignment (rigid body assumption)
- Multi-phase state machines with timeout/force-completion handling
"""

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
    wake_package_body,
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

# ============================================================================
# MODULE ORGANIZATION
# ============================================================================
# 1) Helper functions for ROS I/O and timing (real-world clock robustness)
# 2) Helper functions for TP (trajectory planning) initialization and execution
# 3) Helper functions for package-centric hold control (offset capture, quality check, replanning)
# 4) Package pose resolution and target computation (destination, drop site, etc.)
# 5) Waypoint generators for multi-stage trajectories (pick approach, etc.)
# 6) State reset and logging utilities
# 7) Phase gating and timing management (pause between phases)
# 8) Main BehaviorTree action functions:
#    - ApproachObject: Dual-base coordination with TP approach to pallet
#    - LiftObj: Pick sequence with optional object-centric hold
#    - MoveBase: Transport phase with optional retreat segment
#    - Drop: Placement with base alignment and descent control
#    - Release: Detach and retreat-home sequence
#
# Each action manages a state-machine via timers + blackboard (`node.bb`)


# ============================================================================
# SECTION 1: ROS I/O AND TIMING HELPERS
# ============================================================================

def _float_vec(values):
    """Convert numeric sequence to list of floats, ensuring consistent type."""
    return [float(v) for v in values]


def _ros_now_s(node) -> float:
    """Get current ROS time in seconds (float) from node clock."""
    return float(node.get_clock().now().nanoseconds) / 1e9


def _dt_from_ros_time(now_s: float, last_s: float, default_dt: float = 1.0 / 30.0) -> float:
    """Compute robust time delta handling simulated clock resets and jumps.
    
    Returns default_dt if:
    - last_s is None (first call)
    - calculated dt is non-finite or <= 0 (clock reset or non-monotonic)
    Otherwise returns the computed delta time.
    """
    if last_s is None:
        return float(default_dt)
    dt = float(now_s) - float(last_s)
    # Protect against non-monotonic clocks (common in simulation)
    if (not np.isfinite(dt)) or dt <= 0.0:
        return float(default_dt)
    return float(dt)


def _publish_base_cmd(node, left_cmd=None, right_cmd=None):
    """Publish Twist commands to base controllers for left/right platforms.
    
    Args:
        node: ROS2 node with left_base_pub and right_base_pub publishers
        left_cmd: [vx, vy, wz] velocity for left base (or None to skip)
        right_cmd: [vx, vy, wz] velocity for right base (or None to skip)
    """
    if left_cmd is not None:
        tl = Twist()
        tl.linear.x, tl.linear.y, tl.angular.z = left_cmd
        node.left_base_pub.publish(tl)
    if right_cmd is not None:
        tr = Twist()
        tr.linear.x, tr.linear.y, tr.angular.z = right_cmd
        node.right_base_pub.publish(tr)


def _publish_arm_cmd(node, left_cmd=None, right_cmd=None):
    """Publish joint velocity commands to arm controllers for left/right.
    
    Args:
        node: ROS2 node with left_arm_pub and right_arm_pub publishers
        left_cmd: Joint velocity array for left arm (or None to skip)
        right_cmd: Joint velocity array for right arm (or None to skip)
    """
    if left_cmd is not None:
        la = Float64MultiArray()
        la.data = left_cmd
        node.left_arm_pub.publish(la)
    if right_cmd is not None:
        ra = Float64MultiArray()
        ra.data = right_cmd
        node.right_arm_pub.publish(ra)


def _publish_base_xy(node, left_xy=None, right_xy=None):
    """Utility: pubblica solo componente XY base (wz=0)."""
    left_cmd = [left_xy[0], left_xy[1], 0.0] if left_xy is not None else None
    right_cmd = [right_xy[0], right_xy[1], 0.0] if right_xy is not None else None
    _publish_base_cmd(node, left_cmd=left_cmd, right_cmd=right_cmd)


def _scaled_xy(xy, scale: float):
    """Scala vettore XY mantenendo formato lista."""
    if xy is None:
        return None
    s = float(scale)
    return [float(xy[0]) * s, float(xy[1]) * s]


def _predict_world_target_from_body_velocity(base_pose, vel_xy, duration_s: float):
    """Convert body-frame velocity to world-frame target position after duration.
    
    Given a constant velocity in base frame (typically from config profiles),
    predicts the final world position after moving for duration_s.
    Used to convert legacy open-loop velocity profiles to point-to-point TP targets.
    
    Args:
        base_pose: Current [x, y, z, rx, ry, rz] in world frame
        vel_xy: [vx_base, vy_base] velocity in base frame
        duration_s: Time to apply velocity
    
    Returns:
        [final_x, final_y] in world frame
    """
    yaw = float(base_pose[5])
    vx_b = float(vel_xy[0])
    vy_b = float(vel_xy[1])
    dt = float(max(duration_s, 0.0))
    dx_w = (math.cos(yaw) * vx_b - math.sin(yaw) * vy_b) * dt
    dy_w = (math.sin(yaw) * vx_b + math.cos(yaw) * vy_b) * dt
    return [float(base_pose[0]) + dx_w, float(base_pose[1]) + dy_w]


def _split_package_model_link(node):
    """Estrae (model, link) del pacco da config `package.link_name`."""
    raw = str(getattr(node.cfg.package, "link_name", "")).strip()
    if "::" in raw:
        model = raw.split("::", 1)[0].strip()
        return model, raw
    model = str(getattr(node, "_gazebo_pallet_pose_start_model", "") or "").strip()
    if not model:
        model = "pacco_clone_1"
    return model, raw or "pacco_clone_1::link_1"


# ============================================================================
# SECTION 3: PACKAGE-CENTRIC HOLD CONTROL
# ============================================================================
# Object-centric hold maintains a package (pallet) between two end-effectors during
# transport. Key idea: compute EE targets relative to an estimated package pose,
# ensuring the distance/orientation constraints are met throughout motion.

def _ensure_pkg_hold_state(node):
    """Initialize or ensure presence of package hold state variables on node.
    
    Creates
buffer for grasp parameter state:
    - _pkg_hold_offsets: offset from EE to package [L/R]
    - _pkg_hold_rpy: captured RPY for orientation control [L/R]
    - _pkg_hold_nominal_dist: nominal distance between left/right EE
    - _pkg_hold_last_replan: timing for periodic replan
    - _pkg_hold_start_z, _pkg_hold_target_z: height control during lift/drop
    - _tp_manip_last_exec_monotonic: timestamp for dt calculation
    """
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


# ============================================================================
# SECTION 4: PACKAGE POSE RESOLUTION AND TARGET COMPUTATION
# ============================================================================
# Resolves the current or target package position from multiple sources with fallback
# logic. Critical for task planning since all base targets depend on package location.

def _get_live_package_xyz(node):
    """Retrieve live package pose from available sources (Gazebo model or blackboard).
    
    Priority:
    1. Last Gazebo model state (most recent sensor measurement)
    2. Startup Gazebo snapshot (fallback if no updates)
    3. Blackboard persistent storage
    4. Mock pallet pose (simulation fallback)
    
    Returns:
        [x, y, z] package position in world frame, or None if unavailable
    """
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
    """Determine transport destination (world-frame XY coordinate).
    
    Priority:
    1. Gazebo model by candidates (configured model name e.g., pacco_clone_2)
    2. Fallback to current package position (if model not found)
    3. Return None if no valid destination exists
    
    Returns:
        ([dest_x, dest_y], source_description) or (None, "unavailable")
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


def _resolve_drop_target_xyz(node):
    """Determine drop/release target location (world-frame XYZ coordinate).
    
    Priority (with optional live vs startup snapshots):
    1. Configured target model (e.g., drop_target_model parameter)
    2. Gazebo model by candidates
    3. Fallback to current package position
    4. Return None if unresolvable
    
    Returns:
        ([target_x, target_y, target_z], source_description) or (None, "unavailable")
    """
    man_cfg = node.cfg.manipulation
    raw = str(getattr(man_cfg, "drop_target_model", "")).strip()
    if not raw:
        raw = str(getattr(man_cfg, "transport_destination_model", "pacco_clone_2")).strip() or "pacco_clone_2"
    cands = node._split_model_candidates(raw) if hasattr(node, "_split_model_candidates") else [raw]
    if not cands:
        cands = ["pacco_clone_2"]

    dst_xyz = None
    dst_model = None
    if hasattr(node, "_get_model_pose_xyz_by_candidates"):
        try:
            # Per il deposito preferiamo la posa live (last) e solo poi lo startup snapshot.
            dst_xyz, dst_model = node._get_model_pose_xyz_by_candidates(cands, prefer_start=False)
            if dst_xyz is None:
                dst_xyz, dst_model = node._get_model_pose_xyz_by_candidates(cands, prefer_start=True)
        except Exception:
            dst_xyz, dst_model = None, None

    if isinstance(dst_xyz, (list, tuple)) and len(dst_xyz) >= 3:
        return [
            float(dst_xyz[0]) + float(getattr(man_cfg, "drop_target_offset_x", 0.0)),
            float(dst_xyz[1]) + float(getattr(man_cfg, "drop_target_offset_y", 0.0)),
            float(dst_xyz[2]) + float(getattr(man_cfg, "drop_target_offset_z", 0.0)),
        ], f"model:{dst_model or cands[0]}"

    # Non usare il fallback del transport (puo' includere offset -Y di trasferimento).
    # Se il modello target non e' disponibile, fallback solo sulla posa pacco corrente.
    pkg_xyz = _get_live_package_xyz(node)
    if isinstance(pkg_xyz, (list, tuple)) and len(pkg_xyz) >= 3:
        node._warn_throttled(
            "drop_target_fallback_pkg",
            bt_fmt(f"[Drop] target model not found ({cands}), fallback to current package pose"),
            period_s=3.0,
        )
        return [float(pkg_xyz[0]), float(pkg_xyz[1]), float(pkg_xyz[2])], "fallback:pkg"

    return None, "unavailable"


def _get_live_tp_state(node):
    """Retrieve current live system state for TP input.
    
    Returns minimal state needed by trajectory planner:
    - left/right arm joint positions
    - left/right base poses (world frame: [x, y, z, rx, ry, rz])
    
    Returns:
        (left_arm_jp, right_arm_jp, left_base, right_base) or None if any unavailable
    """
    left_arm_jp = node.get_arm_joint_positions("left")
    right_arm_jp = node.get_arm_joint_positions("right")
    left_base = node.get_base_pose("left")
    right_base = node.get_base_pose("right")
    if None in [left_arm_jp, right_arm_jp, left_base, right_base]:
        return None
    return left_arm_jp, right_arm_jp, left_base, right_base


def _default_side_pkg_offset(node, side: str):
    """Offset EE-pacco nominale per lato, letto da parametri manipulation.hold_*."""
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
    """Seleziona riferimento per stima pacco durante hold (package/left_ee/right_ee)."""
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
    """Estimate package pose from end-effector position minus grasp offset.
    
    When live Gazebo package pose is unavailable, estimates package location
    from one of the end-effectors using captured grasp offsets.
    
    Reference mode configurable via hold_reference parameter:
    - 'package': Use Gazebo model pose (preferred)
    - 'left_ee': Estimate from left EE - offset
    - 'right_ee': Estimate from right EE - offset
    - 'auto': Choose based on attachment mode
    
    Returns:
        [x, y, z] estimated package position, or None
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


def _pkg_xyz_for_alignment(node, left_arm_jp, right_arm_jp, left_base, right_base):
    """Select preferred package pose for base alignment (drop/placement).
    
    Prefers live Gazebo pose (center of physical package), with fallback to
    estimated pose from hold-reference EE. Used during drop and pre-transport
    phases to compute base target positions relative to the package.
    
    Returns:
        [x, y, z] package position for alignment, or None
    """
    pkg_live = _get_live_package_xyz(node)
    pkg_ref = _resolve_pkg_reference_xyz(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    return pkg_live if pkg_live is not None else pkg_ref


def _build_base_cmd_to_xy(
    node,
    base_pose,
    target_xy,
    kp_x: float,
    kp_y: float,
    xy_abs_max: float,
    goal_tol: float | None = None,
):
    """Simple P controller for base velocity to track XY target.
    
    Converts world-frame difference to base-frame error, applies proportional gain.
    Useful for closing-loop base positioning in open-loop profiles or before TP.
    
    Args:
        base_pose: [x, y, z, rx, ry, rz] in world frame
        target_xy: [target_x, target_y] goal in world frame
        kp_x, kp_y: Proportional gains for x and y
        xy_abs_max: Saturation on output velocity magnitude
        goal_tol: Distance threshold for "reached" (defaults to config)
    
    Returns:
        (cmd=[vx, vy, 0], reached_bool, distance_to_target)
    """
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
    """Capture and store EE-to-package grasp offsets at moment of attachment.
    
    Called after attaching the package to freeze the relative position of each
    end-effector with respect to the package. This offset remains constant during
    hold operations, ensuring consistent grasp geometry.
    
    Computes:
    - left/right offsets: [offset_x, offset_y, offset_z] from EE to package
    - nominal_dist: distance between left and right EE (rigid-body constraint)
    - left/right RPY: orientation angles for orientation hold (optional)
    
    Storage mode configurable via hold_use_captured_offsets parameter.
    """
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
    """Construct EE goal pose [xyz+rpy] for object-centric hold.
    
    Computes the target end-effector pose from:
    - Package reference position (pkg_xyz)
    - Captured grasp offset (left/right specific)
    - Orientation mode (fixed or captured)
    - Z height (for lift/drop height control)
    
    Args:
        side: 'left' or 'right'
        pkg_xyz: Current/target package [x, y, z]
        pkg_z_target: Override Z position (for lift/drop), or None to use pkg_xyz[2]
    
    Returns:
        Goal EE pose [x, y, z, rx, ry, rz] as numpy array
    """
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
    """Replan object-centric hold trajectories for package transport.
    
    This is the core object-centric control mechanism:
    1) Estimates current package pose (from Gazebo, EE sensors, or reference)
    2) Computes left/right EE goals relative to package frame + grasp offsets
    3) Updates EE trajectories to maintain the package between both end-effectors
    
    The constraint "package stays between the two EEs" is enforced at the EE level;
    TP then solves inverse kinematics for joint velocities.
    
    Args:
        node: ROS2 node with package state and ee_task
        left/right_arm_jp, left/right_base: Current system state
        pkg_xy_target: Override package xy target (for transport/drop stages)
        pkg_z_target: Target z position for package (for lift/drop height control)
        force: Force replan even if period hasn't elapsed
        preserve_jtc_base: Don't deactivate base during ee_task update
    
    Returns:
        True if hold trajectory was successfully replanned
    """
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


def _execute_tp_arm_control(node, left_arm_jp, right_arm_jp, left_base, right_base, arm_clip_abs: float):
    """Execute TP in arm-only mode: arms move, bases stay stationary.
    
    This mode is used for hold phases where the package is attached to both arms
    and precise positioning doesn't require base motion. Base commands are zeroed
    to prevent drift between consecutive phases.
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
    clip = float(max(1e-4, arm_clip_abs))
    left_arm_cmd = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "left"), clip)
    right_arm_cmd = _sanitize_arm_cmd(node._get_arm_cmd_values(cmd, "right"), clip)
    # In modalita' arm-only, azzera sempre la base per evitare drift residui tra fasi.
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
    """
    Execute TP in full-control mode: both bases and arms move together.
    
    This is the standard "pure TP" mode used in cooperative phases (lift/transport/drop).
    The BehaviorTree logic determines the phase and target, and TP generates
    continuous velocity commands for simultaneous base+arm coordination.
    
    Args:
        node: ROS2 node with TP instance and command publishers
        left/right_arm_jp: Current joint positions for each arm
        left/right_base: Current [x, y, z, rx, ry, rz] poses in world frame
        arm_clip_abs: Maximum joint velocity magnitude
        base_xy_abs_max: Maximum base xy velocity (defaults to node.tp_base_cmd_xy_abs_max)
        base_wz_abs_max: Maximum base angular velocity (defaults to node.tp_base_cmd_wz_abs_max)
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
    """Shortcut for arm-only TP with hold-phase clipping.
    
    Wrapper around _execute_tp_arm_control using hold-specific joint velocity
    saturation (typically lower than approach phase for stability).
    """
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
    """Initialize JTC (trajectory planner) for base-only stage with omni targets.
    
    Sets up Cartesian [x,y,yaw] trajectory targets for left/right bases in world frame.
    After initialization, the BN action will call _execute_tp_full_control repeatedly
    to track these targets. Used for transport, retreat, and base alignment phases.
    
    Args:
        node: ROS2 node with approach_jtc_task
        left/right_base_goal_xy: Target [x, y] in world frame
        period_s: Trajectory duration (time to reach target)
        kp_xy: Proportional gain for xy motion (typically 1.0)
        kp_yaw: Proportional gain for yaw (typically 0.0 to avoid rotation)
        arm_active: If True, also activate arm tracking (for concurrent arm motion)
    
    Returns:
        True if JTC was successfully initialized
    """
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
    """Initialize JTC for joint-space arm trajectory (base inactive).
    
    Sets up joint-space goals for both arms. Used for lock phases, returns to home,
    and precise joint positioning. The base remains stationary during arm-only phases.
    
    Args:
        node: ROS2 node with approach_jtc_task
        left_arm_goal, right_arm_goal: Target joint position vectors
        period_s: Trajectory duration (time to reach goal)
        kp_arm: Arm proportional gain (defaults to node.approach_jtc_arm_kp)
    
    Returns:
        True if JTC was successfully initialized
    """
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
    """Configure Cartesian EE task for waypoint trajectories.
    
    Generates smooth (poly5) EE trajectories from current to goal poses and activates
    the ArmCartesianControl task. TP solves the inverse kinematics and generates joint
    velocities to track these Cartesian trajectories. Used for precise pick approach
    stages and release opening maneuvers.
    
    Args:
        node: ROS2 node with ee_task (ArmCartesianControl)
        left/right_ee_now: Current EE poses [x, y, z, rx, ry, rz]
        left/right_ee_goal: Target EE poses [x, y, z, rx, ry, rz]
        traj_time: Duration of Cartesian trajectory in seconds
    
    Returns:
        True if trajectory was successfully set on ee_task
    """
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
    """Mappa asse nominale roll/pitch/yaw su indice RPY."""
    key = str(axis_name).strip().lower()
    if key == "roll":
        return 0
    if key == "yaw":
        return 2
    return 1


# ============================================================================
# SECTION 5: CONVERGENCE CHECKING AND ERROR METRICS
# ============================================================================
# Utilities for monitoring task completion via position/orientation tolerances.

def _ee_goal_reached(node, ee_live, ee_goal, pos_tol: float, ori_tol: float):
    """Test Cartesian convergence (position + orientation) for an end-effector.
    
    Checks whether EE has reached goal within specified tolerances:
    - pos_err: Euclidean distance in 3D space
    - ori_err: maximum angle error across [roll, pitch, yaw]
    
    Args:
        node: ROS2 node (for angle_diff helper)
        ee_live: Current EE pose [x, y, z, rx, ry, rz] or None
        ee_goal: Goal EE pose [x, y, z, rx, ry, rz] or None
        pos_tol: Position tolerance in meters
        ori_tol: Orientation tolerance in radians
    
    Returns:
        (reached_bool, pos_error, ori_error)
    """
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
    """Select RPY target for side (fixed from config or keep live state).
    
    Returns the target orientation for an end-effector based on:
    - Configuration: approach.ee_orient_mode
    - Fixed mode: use configured approach_ee_left/right_rpy_goal
    - Live mode: use current EE orientation
    
    Args:
        side: 'left' or 'right'
        ee_live: Current EE pose with RPY [x, y, z, rx, ry, rz] or None
    
    Returns:
        [rx, ry, rz] target orientation
    """
    if str(node.approach_ee_orient_mode).lower() == "fixed":
        rpy_goal = node.approach_ee_left_rpy_goal if side == "left" else node.approach_ee_right_rpy_goal
        if rpy_goal is not None and len(rpy_goal) == 3:
            return np.asarray(rpy_goal, dtype=np.float32)
    if isinstance(ee_live, np.ndarray) and ee_live.shape[0] >= 6:
        return np.asarray(ee_live[3:6], dtype=np.float32)
    return np.zeros((3,), dtype=np.float32)


# ============================================================================
# SECTION 6: WAYPOINT GENERATORS AND TRAJECTORY BUILDERS
# ============================================================================

def _build_pick_waypoint_stage_plan(node, left_arm_jp, right_arm_jp, left_base, right_base):
    """Generate multi-stage pick approach trajectory (open -> descend -> grasp).
    
    Creates a sequence of EE waypoints for a controlled approach to the package:
    1. start_open: Both arms approach from above with fingers open (configured offset)
    2. mid_open: Transition to lower approach height, still open
    3. grasp_close: Fine positioning at package grasp point with fingers closed
    
    Each stage has a target duration, position offset, and orientation angle.
    Uses current EE kinematics and package Gazebo pose as reference frame.
    
    Returns:
        List of stage dicts with keys: 'name', 'traj_time', 'left_goal', 'right_goal'
        or None if package pose is unavailable
    """
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
    """Clean up state from waypoint-based pick phase (preparation for next cycle)."""
    node.bb.pop("lift_pick_waypoints", None)
    node.bb.pop("lift_pick_stage_idx", None)
    node.bb.pop("lift_pick_stage_active", None)
    node.bb.pop("lift_pick_stage_start_s", None)


def _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, label: str):
    """Log package hold quality metrics: EE distance, z-gap, nominal offset errors.
    
    Diagnostics to monitor hold control effectiveness during transport/drop:
    - pkg_hold_dist_err: deviation from nominal EE-to-EE distance
    - pkg_hold_z_gap: vertical gap between left/right EE (should be small)
    - pkg_hold_ok: boolean flag indicating healthy hold state
    
    These metrics help detect grasp failures or package slipping before detach.
    """
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
    """Risoluzione robusta del nome modello Gazebo per lato left/right."""
    side = str(side).lower()
    try:
        model = (node._gazebo_base_pose_start_model or {}).get(side, None)
    except Exception:
        model = None
    if isinstance(model, str) and model.strip():
        return model.strip()
    return "left_robot" if side == "left" else "right_robot"


def _get_arm_attach_link_name(side: str) -> str:
    """Ritorna il link di attach lato-specifico usato dal link-attacher."""
    return "ur_left_wrist_3_link" if str(side).lower() == "left" else "ur_right_wrist_3_link"


def _attach_package_to_arms(node) -> bool:
    """Request package attachment via link-attacher service.
    
    Sends AttachLink requests to rigidly joint the package with end-effector(s).
    Attachment mode (single_left, single_right, or dual) configured globally.
    Returns immediately after sending requests (async operation).
    
    Returns:
        True if requests were sent (doesn't guarantee success)
    """
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
    """Request package detachment and wait for service confirmation.
    
    Sends DetachLink requests to remove rigid joint(s) between package and EE(s).
    **Important**: Uses synchronous wait (spin_until_future_complete) to ensure
    the joint is removed in Gazebo BEFORE re-enabling gravity. Otherwise, the ODE
    physics body can remain in sleep mode even with gravity enabled.
    
    Returns:
        True if all detach operations succeeded
    """
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
        # Attesa sincrona: il joint deve essere rimosso in Gazebo PRIMA
        # di riabilitare la gravità, altrimenti il body ODE può restare
        # in sleep e la gravità non ha effetto.
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


def _reset_pkg_hold_runtime(node):
    """Fully reset package hold state (after detach or phase completion).
    
    Clears all object-centric hold variables to prepare for the next grasp cycle.
    Called after Release phase or on error conditions.
    """
    _ensure_pkg_hold_state(node)
    node._pkg_hold_offsets = {"left": None, "right": None}
    node._pkg_hold_rpy = {"left": None, "right": None}
    node._pkg_hold_nominal_dist = float("nan")
    node._pkg_hold_last_replan = 0.0
    node._pkg_hold_start_z = None
    node._pkg_hold_target_z = None
    node._tp_manip_last_exec_monotonic = None


def _phase_pause_key(phase_name: str) -> str:
    """Chiave blackboard usata per ricordare conferma pausa-fase."""
    return f"phase_pause_approved::{str(phase_name)}"


# ============================================================================
# SECTION 7: PHASE GATING AND STATE MANAGEMENT
# ============================================================================
# Multi-phase actions use blackboard state machines with optional interactive pausing.

def _phase_pause_gate(node, phase_name: str) -> bool:
    """Optional interactive pause before phase (for debugging/manual control).
    
    If enabled, pauses before the phase and waits for user confirmation via stdin.
    Only active in Supervisor tree (prevents multiple prompts from parallel subtrees).
    Non-interactive contexts (non-TTY stdin) auto-continue.
    
    Args:
        node: ROS2 node
        phase_name: Name of phase (e.g., 'LiftObj') for logging
    
    Returns:
        True: Phase approved, continue
        False: User paused phase (q/quit command)
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
    """Clear pause confirmation flag for phase (e.g., on phase restart)."""
    node.bb.pop(_phase_pause_key(phase_name), None)


def _reset_movebase_runtime(node):
    """Clear all MoveBase state machine variables (used between retries or phase resets)."""
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
    """Compute maximum joint-space error (largest angle difference).
    
    Useful for monitoring arm convergence in joint-space trajectories.
    Uses angle-aware difference (handles angle wrapping at ±π).
    
    Returns:
        Maximum absolute angle error across all joints, or inf if data invalid
    """
    if q_now is None or q_goal is None:
        return float("inf")
    if len(q_now) != len(q_goal):
        return float("inf")
    errs = [abs(float(node._angle_diff(float(q_now[i]), float(q_goal[i])))) for i in range(len(q_now))]
    return float(max(errs)) if errs else 0.0


def _clip_descend_pick_joint_goals(node, q_left_goal: np.ndarray, q_right_goal: np.ndarray):
    """Clip pick/descent targets within joint limit safety margins.
    
    Prevents arm singularities or self-collision by enforcing joint limits.
    Uses configured joint limit ranges from robot model.
    
    Args:
        node: ROS2 node with robot model and arm index configuration
        q_left_goal, q_right_goal: Target joint position vectors
    
    Returns:
        (q_left_clipped, q_right_clipped) within safe ranges
    """
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
    """Initialize JTC for descend/pick phase in joint-space.
    
    Moves arms downward by applying joint deltas (from config) to current position.
    The deltas should typically be negative to lower the arms toward the package.
    
    Args:
        node: ROS2 node with approach_jtc_task
        left/right_arm_jp: Current joint positions
        descend_and_pick_time: Duration of descent trajectory
        left/right_arm_pick: Joint delta vectors (config-derived)
    
    Returns:
        True if JTC was successfully initialized
    """
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
    """Initialize JTC for pre-transport pose (joint-space arm motion).
    
    Moves arms to a configured safe pose before starting transport phase.
    Typically used to move arms closer to body or away from hazards.
    Configurable via manipulation.pre_transport_*_arm_goal parameters.
    
    Args:
        node: ROS2 node with approach_jtc_task and config
        left/right_arm_jp: Current joint positions
    
    Returns:
        True if JTC was successfully initialized
    """
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
    """Log joint efforts as force proxy (RMS and MAX per side).
    
    Since the system may not have direct force/torque sensors, joint efforts
    from /joint_states provide a rough indication of load and contact forces.
    Useful for detecting collisions, contact events, or grasp success.
    
    Stores metrics on blackboard for downstream logic:
    - force_proxy_left_rms, force_proxy_right_rms
    - force_proxy_left_max, force_proxy_right_max
    
    Args:
        node: ROS2 node
        label: Debug label for logging (e.g., "attach", "pick")
        period_s: Throttle logging period in seconds
    """
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


# ============================================================================
# SECTION 2: MAIN BEHAVIOR TREE ACTIONS
# ============================================================================

def ApproachObject():
    """Approach pallet using Task Prioritization with dual-base coordination.
    
    DESCRIPTION:
    Dual-arm mobile robot approaches a pallet for pickup. Left (SRM1) and Right (SRM2)
    robots work in parallel, with each following its own TP trajectory. Coordinates via:
    - Base positioning: approach_base_ctrl (PD or TP-based)
    - Arm motion: TP endpoint tracking toward pallet
    - Gating: SRM2 waits for SRM1 to be near object before its approach
    
    FLOW:
    1. Validate live state (joint positions, odometry) and cross-robot synchronization
    2. Initialize/re-initialize TP with approach target (from pallet pose estimate)
    3. Execute TP loop: compute commands, publish to arm+base controllers
    4. Check convergence (joint+EE+base errors within tolerances)
    5. Set <TREE>_near_object flag and exit
    
    NOTES:
    - SRM1 and SRM2 run separately before synchronization (Supervisor oversees)
    - Base can use legacy open-loop profiles or TP-based approach
    - Optional arm pre-delay gates arm motion until base is close enough
    - Supports reinitializing TP if pallet source upgrades (estimated→real)
    
    RETURNS:
        True: Approach converged, near_object flag set
        None: Still running or waiting for data
        False: Fatal error
    """
    # =================================================================
    # INIT: Get ROS node and identify which robot (left/right) this BT runs on
    # =================================================================
    node = _require_node()
    tree_name = get_current_bt_name()  # Name of BT tree (e.g., "SRM1", "SRM2", "Supervisor")
    side = node.side_from_bt_name(tree_name)  # Extract side from tree name: 'left', 'right', or None
    timer_key = f"{tree_name}_ApproachObject"  # Unique key for action timer on node
    near_key = f"{tree_name}_near_object"  # Blackboard key for convergence flag

    # =================================================================
    # SENSOR DATA ACQUISITION: Read current arm joints and base poses
    # =================================================================
    # Query current joint positions from /left/joint_states and /right/joint_states topics
    left_arm_jp = node.get_arm_joint_positions("left")
    right_arm_jp = node.get_arm_joint_positions("right")

    # Query current base poses in world frame from odometry topics
    # Format: [x, y, z, roll, pitch, yaw] (absolute position + orientation)
    left_base = node.get_base_pose("left")
    right_base = node.get_base_pose("right")
    
    # =================================================================
    # GATE 1: Wait for sensor data to be available
    # =================================================================
    # Cannot proceed with TP if any critical sensor data is missing
    # (joint states or odometry must be valid)
    if None in [left_arm_jp, right_arm_jp, left_base, right_base]:
        node._warn_throttled(
            f"{tree_name}_approach_wait",
            bt_fmt("[ApproachObject] Waiting for sensor data (joint_states/odom)"),
            period_s=1.0
        )
        return None  # RUNNING - keep trying until all data arrives

    # =================================================================
    # GATE 2: SRM2 synchronization (right robot waits for SRM1 readiness)
    # =================================================================
    # In dual-robot systems, SRM2 (right) should not approach until SRM1 (left)
    # has validated the pallet location (simulates vision/detection handshake)
    if side == "right" and not bool(node.bb.get("srm1_data_to_srm2", False)):
        node._info_throttled(
            f"{tree_name}_approach_wait_data",
            bt_fmt("[ApproachObject] waiting pallet info from SRM1"),
            period_s=1.0,
        )
        return None  # RUNNING - wait for SRM1 to signal readiness on blackboard

    # =================================================================
    # PALLET POSE RESOLUTION: Get current pallet location for approach target
    # =================================================================
    # Try to resolve pallet XY (2D) and XYZ (3D) poses from multiple sources:
    # 1. Gazebo model states (real-time simulation)
    # 2. Blackboard (shared cross-robot data)
    # 3. Mock/estimated poses (fallback for TP compatibility)
    pallet_xy = node._ensure_mock_pallet_pose(left_base=left_base, right_base=right_base)
    pallet_xyz = node._ensure_mock_pallet_pose_xyz(left_base=left_base, right_base=right_base)

    # =================================================================
    # PHASE INIT: First tick - set up action timer and blackboard state
    # =================================================================
    t0 = node.get_action_timer(timer_key)  # Get cached timer start time (or None if first tick)
    if t0 is None:
        # First execution: initialize state machine for this action
        t0 = node.start_action_timer(timer_key)  # Start tracking elapsed time
        node.bb[near_key] = False  # Initialize convergence flag (not near yet)
        node._approach_slot_assignment = None  # Reset slot (dual-arm positioning slot)
        
        # Track per-side base goal achievement for synchronization
        if side == "left":
            node.bb["SRM1_base_goal_reached"] = False
        elif side == "right":
            node.bb["SRM2_base_goal_reached"] = False
        
        # Log action start with key parameters
        node.get_logger().info(
            bt_fmt(
                f"[ApproachObject] start TP approach "
                f"(side={side or 'both'}) "
                f"(use_base={node.approach_use_base}) "
                f"(dur={node.approach_duration:.1f}s)"
            )
        )

    # =================================================================
    # GATE 3: Wait for pallet pose source upgrade (from estimated to real)
    # =================================================================
    # In motion planning, it's better to wait for a confirmed pose source
    # rather than start with an estimated/guessed one.
    # This gate implements a timeout: wait for real pose, but proceed anyway
    # after approach_estimated_timeout seconds if needed.
    elapsed_from_start = node.get_clock().now().nanoseconds / 1e9 - t0
    pallet_source = str(node.bb.get("pallet_pose_source", "unset")).lower()
    
    # Check if we should wait for source upgrade (real Gazebo/world pose)
    if (
        node.approach_use_base
        and pallet_xy is not None
        and pallet_source == "estimated"  # Currently using estimate (not real)
        and elapsed_from_start < float(node.approach_estimated_timeout)  # Still within grace period
    ):
        # Log waiting state with remaining time
        node._info_throttled(
            f"{tree_name}_approach_wait_pallet_source",
            bt_fmt(
                "[ApproachObject] waiting for pallet pose source upgrade "
                f"(current={pallet_source}, t={elapsed_from_start:.2f}/{node.approach_estimated_timeout:.2f}s)"
            ),
            period_s=0.5,
        )
        return None  # RUNNING - keep waiting for better pose source

    # =================================================================
    # TP TRAJECTORY INIT: Set up TP with approach target (first time or upgrade)
    # =================================================================
    if not node._tp_approach_traj_initialized:
        # First execution: initialize TP trajectory from current live state
        # TP will fill internal trajectory structures for arm+base approach toward pallet
        ok = node._init_tp_approach_trajectory_from_live_state(
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
            pallet_xy=pallet_xy,
            pallet_xyz=pallet_xyz,
        )
        if not ok:
            return None  # RUNNING - TP init not ready yet
    else:
        # TP already initialized - check if pallet pose source improved
        # (e.g., went from estimated to real Gazebo model)
        used_source = str(getattr(node, "_approach_pallet_source_used", "unset")).lower()
        
        # If source is "better" now (real vs estimated), reinitialize to use new pose
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
                return None  # TP reinitialization failed

    # =================================================================
    # CORE TP EXECUTION: Run trajectory planner and handle command caching
    # =================================================================
    # Build input state vectors for TP from current arm/base poses
    joint_pos, base_odom = node._build_tp_inputs_from_side_data(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )

    # Execute TP at safe rate (cache commands to avoid redundant computation)
    # TP can be expensive, so we cache the last command and use it if
    # the time since last execution is less than tp_cmd_min_period
    now_ros_s = _ros_now_s(node)  # Current ROS time in seconds
    cmd = node._tp_cmd_cache  # Get cached command from last execution
    
    # Check if we can reuse cached command (time-based throttling)
    can_reuse = (
        cmd is not None
        and node._tp_cmd_cache_time is not None
        and (now_ros_s - node._tp_cmd_cache_time) < max(node.tp_cmd_min_period, 1e-3)
    )
    
    if not can_reuse:
        # Time to recompute: execute TP trajectory planner
        # 1. Compute robust timestep (handles clock jumps/resets)
        tp_dt = _dt_from_ros_time(now_ros_s, node._tp_last_exec_monotonic, default_dt=1.0 / 30.0)
        node._tp_last_exec_monotonic = now_ros_s
        
        # 2. Set TP timestep (clamped to safe range)
        node.tp._delta_t = max(1e-3, min(float(tp_dt), 0.2))
        
        # 3. Run TP: compute velocity commands from current state to approach target
        cmd = node.tp.execute(joint_pos=joint_pos, base_odom=base_odom)
        
        # 4. Cache result for next cycle (if within min_period, reuse this)
        node._tp_cmd_cache = cmd
        node._tp_cmd_cache_time = now_ros_s
    
    # Validate TP output
    if cmd is not None and len(cmd) < 18:
        node._warn_throttled(
            "approach_cmd_short",
            bt_fmt(f"[ApproachObject] TP command vector too short: len={len(cmd)}")
        )

    # =================================================================
    # RAMP FACTOR: Smooth acceleration from rest to avoid jerky motion
    # =================================================================
    # Linear ramp: starts at 0, reaches 1.0 after tp_base_cmd_ramp_time seconds
    # This prevents sudden command jumps that could cause dynamic imbalance
    elapsed = node.get_clock().now().nanoseconds / 1e9 - t0
    ramp = 1.0
    if node.approach_use_base:
        # Apply linear ramp over configured time window
        ramp = min(max(elapsed / max(node.tp_base_cmd_ramp_time, 1e-3), 0.0), 1.0)

    # =================================================================
    # BASE TARGET COMPUTATION: Compute where each base should go (for monitoring)
    # =================================================================
    # These targets are computed based on pallet pose and dual-base formation
    # Used for visualization and convergence checking (separate from TP targets)
    left_base_reached = True
    right_base_reached = True
    left_target_xy = None
    right_target_xy = None
    left_base_cmd_vis = [0.0, 0.0, 0.0]  # Visualization command (for logging)
    right_base_cmd_vis = [0.0, 0.0, 0.0]
    
    if pallet_xy is not None:
        # Compute approach target for each base relative to pallet
        # (determines formation geometry: left/right spacing)
        left_target_xy = node._compute_side_base_target_xy("left", left_base, right_base, pallet_xy)
        right_target_xy = node._compute_side_base_target_xy("right", left_base, right_base, pallet_xy)
        
        # Get visualization commands (for logging approach progress)
        left_base_cmd_vis, left_base_reached = node._compute_base_cmd_from_pallet(
            "left", left_base, left_target_xy, ramp=ramp
        )
        right_base_cmd_vis, right_base_reached = node._compute_base_cmd_from_pallet(
            "right", right_base, right_target_xy, ramp=ramp
        )
    else:
        # No pallet pose: targets undefined, bases cannot converge
        left_base_reached = False
        right_base_reached = False

    # =================================================================
    # COMMAND EXTRACTION: Parse TP output command vector into base/arm velocities
    # =================================================================
    # TP returns a single vector [base_cmds_left, arm_cmds_left, base_cmds_right, arm_cmds_right]
    # We extract and sanitize these commands per-side
    
    if node.approach_use_base:
        # FULL-TP MODE: Both base and arms commanded by TP
        # Extract base velocity commands from TP vector
        left_base_cmd_vals = _sanitize_base_cmd(
            node._get_base_cmd_values(cmd, "left"),  # Extract left base portion
            xy_abs_max=node.tp_base_cmd_xy_abs_max,  # Clip XY velocity
            wz_abs_max=node.tp_base_cmd_wz_abs_max,  # Clip angular velocity
            ramp=ramp,  # Apply acceleration ramp
        )
        right_base_cmd_vals = _sanitize_base_cmd(
            node._get_base_cmd_values(cmd, "right"),  # Extract right base portion
            xy_abs_max=node.tp_base_cmd_xy_abs_max,
            wz_abs_max=node.tp_base_cmd_wz_abs_max,
            ramp=ramp,
        )
    else:
        # LEGACY MODE: Bases use manual control, not TP
        left_base_cmd_vals = [0.0, 0.0, 0.0]
        right_base_cmd_vals = [0.0, 0.0, 0.0]
        left_base_reached = True  # Manual control not time-based
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
    # Handle optional arm pre-delay (arm motion gated until base is close)
    # Used when arms shouldn't move until bases are in safe approach formation
    if arm_delay_waiting:
        if arm_gate_dist <= float(node.approach_arm_enable_dist):
            # Base is close enough: enable arm motion
            # Re-initialize arm state to avoid jumps
            if node._enable_approach_arm_motion_from_current_state(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            ):
                # Clear cached TP command to force recomputation with arm enabled
                node._tp_cmd_cache = None
                node._tp_cmd_cache_time = None
                node._tp_last_exec_monotonic = None
                arm_delay_waiting = False  # Arms now active
        else:
            # Still too far: keep arms in hold-home position
            node._info_throttled(
                f"{tree_name}_approach_arm_hold",
                bt_fmt(
                    "[ApproachObject] arm pre-phase hold-home active "
                    f"(base_dist={arm_gate_dist:.3f}m > enable_dist={node.approach_arm_enable_dist:.3f}m)"
                ),
                period_s=1.0,
            )

    # Extract arm commands from TP and apply clipping
    left_arm_cmd_vals = _sanitize_arm_cmd(
        node._get_arm_cmd_values(cmd, "left"),  # Extract left arm velocities
        node.tp_arm_cmd_abs_max  # Clip to max joint velocity
    )
    right_arm_cmd_vals = _sanitize_arm_cmd(
        node._get_arm_cmd_values(cmd, "right"),  # Extract right arm velocities
        node.tp_arm_cmd_abs_max
    )
    
    # =================================================================
    # CONVERGENCE CHECKING: Monitor arm and base approach accuracy
    # =================================================================
    # Get live end-effector poses (for arm convergence check)
    ee_live = node._get_live_ee_by_side(
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    
    # Check if each arm has reached its approach target
    left_arm_reached, left_j_err, left_ee_err = node._is_side_arm_converged(
        "left", left_arm_jp, ee_live.get("left")
    )
    right_arm_reached, right_j_err, right_ee_err = node._is_side_arm_converged(
        "right", right_arm_jp, ee_live.get("right")
    )
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

    # =================================================================
    # CONVERGENCE CRITERION: Check if approach task is complete
    # =================================================================
    # Approach converged when:
    # 1. Arms reached target EE pose and joint positions
    # 2. Bases reached target XY positions
    # 3. Minimum execution time elapsed (prevents instant exit)
    approach_converged = bool(
        arm_reached and base_reached and (elapsed >= float(node.approach_min_exec_time))
    )
    
    # Determine current arm motion state (for diagnostics)
    arm_phase = (
        "tracking" if bool(node._approach_arm_motion_enabled)
        else ("hold_home" if bool(node._approach_arm_delay_active) else "disabled")
    )

    # =================================================================
    # LOGGING: Detailed diagnostics for approach progress tracking
    # =================================================================
    # Log comprehensive state including:
    # - Current velocity commands (base XY/wz, arm max magnitude)
    # - Convergence status (arm/base reached flags)
    # - Error metrics (joint errors, EE position/orientation errors, distances)
    # - Configuration state (clipping values, ramp factor, control mode)
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
            f"pallet_src={node.bb.get('pallet_pose_source', 'unset')}, "
            f"side_order={node.bb.get('approach_side_order', 'n/a')}, "
            f"slot_assign={node.bb.get('approach_slot_assignment', 'n/a')}, "
            f"dt={node.tp._delta_t:.4f}s, ramp={ramp:.2f}"
        ),
        period_s=1.0,
    )

    # =================================================================
    # COMMAND PUBLICATION: Send velocity commands to robot controllers
    # =================================================================
    # Publish only commands for the sides handled by this BT instance
    # (Left BT tree controls left robot, Right tree controls right robot)
    if side == "left":
        # Left robot only: send base+arm commands for left side
        _publish_base_cmd(node, left_cmd=left_base_cmd_vals)
        _publish_arm_cmd(node, left_cmd=left_arm_cmd_vals)
    elif side == "right":
        # Right robot only: send base+arm commands for right side
        _publish_base_cmd(node, right_cmd=right_base_cmd_vals)
        _publish_arm_cmd(node, right_cmd=right_arm_cmd_vals)
    else:
        # Both robots (Supervisor tree): send both sides
        _publish_base_cmd(node, left_cmd=left_base_cmd_vals, right_cmd=right_base_cmd_vals)
        _publish_arm_cmd(node, left_cmd=left_arm_cmd_vals, right_cmd=right_arm_cmd_vals)

    # =================================================================
    # SUCCESS: Approach converged - prepare for next phase
    # =================================================================
    if approach_converged:
        # Stop movement to avoid overshoot
        if side in ("left", "right"):
            node.stop_side_movement(side)  # Stop just this side
        else:
            node.stop_all_movement()  # Stop all robots
        
        # Clear action state
        node.clear_action_timer(timer_key)  # Remove timer to signal action completion
        node.bb[near_key] = True  # Signal to downstream actions that approach is complete
        
        # Set per-side goal-reached flags for synchronization
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
        return True  # SUCCESS - action complete

    # =================================================================
    # TIMEOUT CHECK: Log if duration exceeded without convergence
    # =================================================================
    # Non-fatal: just warn but keep running (TP may still be tracking)
    if elapsed >= node.approach_duration:
        node._warn_throttled(
            f"{tree_name}_approach_not_converged",
            bt_fmt(
                "[ApproachObject] elapsed nominal duration without convergence "
                f"(elapsed={elapsed:.2f}s, arm_reached={arm_reached}, base_reached={base_reached})"
            ),
            period_s=2.0,
        )

    return None  # RUNNING - keep executing until convergence or timeout


def LiftObj():
    """Pick and lift phase with optional object-centric hold control.
    
    DESCRIPTION:
    Dual arms coordinate to grasp and lift a pallet. Complex state machine with
    optional waypoint-based approach and object-centric hold during collect phase.
    
    STATE MACHINE:
    1. descend_pick: Waypoint EE approach or direct joint descent to grasp position
       - Waypoints: start_open → mid_open → grasp_close (if enabled)
       - Or: direct joint delta + TP joint-space control (fallback)
    2. attach: Brief attachment phase
       - Send link-attach service to package
       - Disable package gravity
       - Capture grasp offsets for hold control
    3. collect: Lift phase with base+arm coordinated motion
       - If hold active: maintain package between EEs while lifting
       - If hold inactive: joint-space lift trajectory + base retreat
    4. pre_transport_pose (optional): Prepare arms for transport
       - Package-centric mode: position arms relative to package
       - Or: joint-space return to configured pre-transport pose
    
    RETURNS:
        True: Lift completed
        None: Still running or waiting for data
        False: Fatal error
    """
    # =================================================================
    # CONFIGURATION LOAD: Get timing, motion profiles, and manipulation mode
    # =================================================================
    # Read all config from node: pick/collect timings, base/arm velocities,
    # object-centric hold flags, and tree context (Supervisor vs single-arm)
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

    # =================================================================
    # SUPERVISOR SYNC GATE: Wait for both SRM1 & SRM2 to finish approach
    # =================================================================
    # In Supervisor tree: both robots must reach near_object before lifting
    # In single-arm tree: proceed immediately to lift
    if tree_name == "Supervisor":
        srm1_ready = node.bb.get("SRM1_near_object", False)
        srm2_ready = node.bb.get("SRM2_near_object", False)
        # If either robot hasn't finished approach, wait and return RUNNING
        if not (srm1_ready and srm2_ready):
            wait_key = "supervisor_lift_wait_logged"
            # Log once to avoid spam, then poll silently
            if not node.bb.get(wait_key):
                node.bb[wait_key] = True
                node.get_logger().info(bt_fmt("[LiftObj] waiting for SRM1/SRM2 approach completion"))
            return None  # RUNNING - keep waiting
        else:
            # Both ready: clear wait flag
            node.bb.pop("supervisor_lift_wait_logged", None)

    # =================================================================
    # PHASE PAUSE GATE: Don't proceed if system is in pause state
    # =================================================================
    if not _phase_pause_gate(node, "LiftObj"):
        return None  # RUNNING - on pause, keep trying

    # =================================================================
    # INIT: First-run setup for LiftObj action
    # =================================================================
    # Timer t0=None means first-run: set up phase machine, reset all state flags
    # and blackboard keys related to descend/attach/collect/pre-transport
    t0 = node.get_action_timer("LiftObj")
    if t0 is None or node.lift_phase is None:
        # First-run: initialize all timers, state, and blackboard
        node.get_logger().info(bt_fmt(f"[LiftObj] start (dur {descend_and_pick_time + 0.5 + collect_time}s)"))
        t0 = node.start_action_timer("LiftObj")
        node.lift_phase = "descend_pick"  # First phase: descend and pick
        
        # Reset object-centric hold runtime (EE offsets, Z targets, etc.)
        _reset_pkg_hold_runtime(node)
        # Reset pick waypoint state (stage index, active flags, EE goals)
        _reset_pick_waypoint_runtime(node)
        # Clear converged state and all TP states
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

    # =================================================================
    # ELAPSED TIME: Track action execution duration since start
    # =================================================================
    elapsed = node.get_clock().now().nanoseconds/1e9 - t0

    # =================================================================
    # PHASE 1: DESCEND_PICK (pick waypoint or TP joint descent + grasp)
    # =================================================================
    # This phase executes a gripper approach + grasping sequence either via:
    # - EE waypoint trajectories (if enabled): start_open -> mid_open -> grasp_close
    # - Or joint-space trajectory planner (TP) if waypoints unavailable
    if node.lift_phase == "descend_pick":
        # =================================================================
        # SENSOR DATA ACQUISITION: Get current joint/base poses
        # =================================================================
        # TP requires live feedback to compute arm commands
        live_state = _get_live_tp_state(node)
        if live_state is None:
            # No joint states or odometry yet: wait and retry
            node._warn_throttled(
                "lift_descend_wait_data",
                bt_fmt("[LiftObj] waiting for sensor data (joint_states/odom)"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None  # RUNNING

        # =================================================================
        # PICK MODE SELECTION: Waypoint approach vs TP joint descent
        # =================================================================
        # Check config: use EE waypoints (smooth approach) or TP (joint trajectory)?
        use_waypoint_pick = bool(getattr(man_cfg, "pick_use_ee_waypoints", True))
        if use_waypoint_pick:
            left_arm_jp, right_arm_jp, left_base, right_base = live_state
            
            # =================================================================
            # EE WAYPOINT INIT: Build and store multi-stage EE trajectory
            # =================================================================
            # Waypoints sequence: open -> mid (fingers slightly open) -> close
            # provides smooth approach while gripper opens properly
            stages = node.bb.get("lift_pick_waypoints", None)
            if stages is None:
                # First time: generate waypoint stages from current pose to grasp pose
                stages = _build_pick_waypoint_stage_plan(
                    node,
                    left_arm_jp=left_arm_jp,
                    right_arm_jp=right_arm_jp,
                    left_base=left_base,
                    right_base=right_base,
                )
                if not stages:
                    # Waypoint build failed: fallback to TP joint trajectory
                    node._warn_throttled(
                        "lift_pick_waypoints_init_fail",
                        bt_fmt("[LiftObj] pick waypoint init failed, fallback to joint descend"),
                        period_s=2.0,
                    )
                    use_waypoint_pick = False  # Disable waypoints, use TP instead
                else:
                    # Waypoint plan successful: store stages + initialize tracking
                    node.bb["lift_pick_waypoints"] = stages
                    node.bb["lift_pick_stage_idx"] = 0  # Start at stage 0
                    node.bb["lift_pick_stage_active"] = False  # Not executing yet
                    node.get_logger().info(
                        bt_fmt(
                            "[LiftObj] pick waypoints initialized "
                            f"(stages={[s['name'] for s in stages]})"
                        )
                    )

            # =================================================================
            # EE WAYPOINT EXECUTION: Execute each stage until all completed
            # =================================================================
            if use_waypoint_pick:
                stages = node.bb.get("lift_pick_waypoints", []) or []
                stage_idx = int(node.bb.get("lift_pick_stage_idx", 0))
                
                # Check if all waypoint stages have been completed
                if stage_idx >= len(stages):
                    # All stages done: stop movementand transition to attach phase
                    node.stop_all_movement()
                    _reset_pick_waypoint_runtime(node)  # Clear waypoint state
                    node.lift_phase = "attach"  # Move to next phase
                    node.bb["lift_descend_tp_ready"] = False
                    node.bb.pop("lift_descend_goal_left", None)  # Clear TP goals
                    node.bb.pop("lift_descend_goal_right", None)
                    node.get_logger().info(bt_fmt("[LiftObj] pick waypoints completed, attaching..."))
                    node.start_action_timer("LiftObj")
                    return None  # RUNNING - stay in loop, next iteration enters attach phase

                stage = stages[stage_idx]
                
                # =================================================================
                # STAGE ACTIVATE: If not already running, start this waypoint stage
                # =================================================================
                if not bool(node.bb.get("lift_pick_stage_active", False)):
                    # Get current EE poses to compute trajectory from current to goal
                    ee_live = node._get_live_ee_by_side(
                        left_arm_jp=left_arm_jp,
                        right_arm_jp=right_arm_jp,
                        left_base=left_base,
                        right_base=right_base,
                    )
                    left_ee = ee_live.get("left", None)
                    right_ee = ee_live.get("right", None)
                    
                    # Require live EE poses to set trajectories
                    if left_ee is None or right_ee is None:
                        node._warn_throttled(
                            "lift_pick_waypoints_no_ee",
                            bt_fmt("[LiftObj] cannot start pick stage: live EE pose unavailable"),
                            period_s=1.0,
                        )
                        rclpy.spin_once(node, timeout_sec=0.01)
                        return None  # RUNNING - wait for EE data
                    
                    # Invoke TP to compute trajectory from current EE -> stage goal
                    if not _set_tp_ee_traj(
                        node,
                        left_ee_now=left_ee,
                        right_ee_now=right_ee,
                        left_ee_goal=stage["left_goal"],
                        right_ee_goal=stage["right_goal"],
                        traj_time=float(stage.get("traj_time", 2.0)),
                    ):
                        # TP failed to compute trajectory: fallback to joint TP
                        node._warn_throttled(
                            "lift_pick_waypoints_set_traj_fail",
                            bt_fmt("[LiftObj] unable to set TP EE waypoint trajectory, fallback to joint descend"),
                            period_s=2.0,
                        )
                        _reset_pick_waypoint_runtime(node)
                        use_waypoint_pick = False  # Disable waypoints
                    else:
                        # TP trajectory set successfully: enable stage execution
                        node.bb["lift_pick_stage_active"] = True
                        node.bb["lift_pick_stage_start_s"] = _ros_now_s(node)
                        node.get_logger().info(
                            bt_fmt(
                                f"[LiftObj] pick stage start: {stage['name']} "
                                f"(T={float(stage.get('traj_time', 0.0)):.2f}s, "
                                f"L={np.round(np.asarray(stage['left_goal'])[:3], 3).tolist()}, "
                                f"R={np.round(np.asarray(stage['right_goal'])[:3], 3).tolist()}"
                            )
                        )

                # =================================================================
                # STAGE EXECUTE: TP commands + convergence checking for waypoint
                # =================================================================
                if use_waypoint_pick and bool(node.bb.get("lift_pick_stage_active", False)):
                    left_arm_jp, right_arm_jp, left_base, right_base = live_state
                    
                    # Send TP-generated arm commands to controller
                    _execute_tp_arm_control(
                        node,
                        left_arm_jp=left_arm_jp,
                        right_arm_jp=right_arm_jp,
                        left_base=left_base,
                        right_base=right_base,
                        arm_clip_abs=float(getattr(man_cfg, "pick_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
                    )
                    
                    # Get current EE poses for convergence checking
                    ee_live = node._get_live_ee_by_side(
                        left_arm_jp=left_arm_jp,
                        right_arm_jp=right_arm_jp,
                        left_base=left_base,
                        right_base=right_base,
                    )
                    
                    # Check if EEs have reached their goal poses for this stage
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
                    
                    # Check stage elapsed time and timeout
                    t_stage = float(node.bb.get("lift_pick_stage_start_s", _ros_now_s(node)))
                    stage_elapsed = _ros_now_s(node) - t_stage
                    stage_timeout = max(
                        float(getattr(man_cfg, "pick_stage_timeout", 6.0)),
                        float(stage.get("traj_time", 0.0)) + 0.2,
                    )
                    
                    # Stage converged if both EEs reached their goals
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
                    
                    # Move to next stage if converged or timeout exceeded
                    if reached or stage_elapsed >= stage_timeout:
                        node.stop_all_movement()
                        node.bb["lift_pick_stage_idx"] = int(stage_idx) + 1  # Next stage
                        node.bb["lift_pick_stage_active"] = False
                        node.bb.pop("lift_pick_stage_start_s", None)
                        # Warn if timeout but log if converged
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
                    return None  # RUNNING - stay on current stage

        # =================================================================
        # TP JOINT DESCENT FALLBACK: If waypoints disabled, use TP joint trajectory
        # =================================================================
        # Fallback mode: direct joint-space descent to grasp pose using TP
        tp_ready = bool(node.bb.get("lift_descend_tp_ready", False))
        if live_state is not None and (not tp_ready):
            left_arm_jp, right_arm_jp, _left_base, _right_base = live_state
            # Initialize TP for joint-space descent to grasp position
            tp_ready = _init_descend_pick_tp(
                node,
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                descend_and_pick_time=descend_and_pick_time,
                left_arm_pick=left_arm_pick,
                right_arm_pick=right_arm_pick,
            )

        # =================================================================
        # TP JOINT TRACK: Execute TP commands until descent converges
        # =================================================================
        descend_reached = False
        if live_state is not None and tp_ready:
            left_arm_jp, right_arm_jp, left_base, right_base = live_state
            # Send TP-generated joint commands to arm controller (hold-in-place)
            _execute_tp_arm_hold(node, left_arm_jp, right_arm_jp, left_base, right_base)
            
            # Get TP-computed goal joint positions
            q_left_goal = node.bb.get("lift_descend_goal_left", None)
            q_right_goal = node.bb.get("lift_descend_goal_right", None)
            
            # Compute max joint error for each arm
            l_err = _max_joint_error(node, left_arm_jp, q_left_goal)
            r_err = _max_joint_error(node, right_arm_jp, q_right_goal)
            
            # Convergence: both arms within joint tolerance
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
            # TP not ready - warn that this mode requires TP
            node._warn_throttled(
                "lift_descend_tp_required",
                bt_fmt("[LiftObj] TP required: descend stage waiting TP init (open-loop disabled)"),
                period_s=2.0,
            )

        # =================================================================
        # DESCEND COMPLETION: Move to attach phase when descended or timed out
        # =================================================================
        # Proceed to attach if descended OR if elapsed time exceeds descend duration
        if elapsed < descend_and_pick_time and not descend_reached:
            rclpy.spin_once(node, timeout_sec=0.01)
            return None  # RUNNING - still descending

        # Time or convergence reached: stop and transition to attach
        node.stop_all_movement()
        _reset_pick_waypoint_runtime(node)  # Clear waypoint state if used
        node.lift_phase = "attach"  # Move to next phase
        node.bb["lift_descend_tp_ready"] = False
        node.bb.pop("lift_descend_goal_left", None)
        node.bb.pop("lift_descend_goal_right", None)
        node.get_logger().info(bt_fmt("[LiftObj] reached pick pose, attaching..."))
        node.start_action_timer("LiftObj")  # Reset phase timer
        return None  # RUNNING - stay in loop, next iteration enters attach

    # =================================================================
    # PHASE 2: ATTACH (brief phase - link attacher + gravity disable + grasp offset capture)
    # =================================================================
    # Brief phase: establish rigid attachment between gripper and package,
    # disable gravity for package to prevent falling, capture EE-to-package offsets
    # for later object-centric control during lift/transport
    if node.lift_phase == "attach":
        # =================================================================
        # LINK ATTACH: Send RPC to link-attacher for rigid gripper-package bond
        # =================================================================
        # Attach only once per cycle; skip if already attached
        if not bool(node.bb.get("package_attached", False)):
            # Invoke link-attacher RPC to make gripper fingers stay bonded to package
            if _attach_package_to_arms(node):
                node.bb["package_attached"] = True  # Mark as attached

        # =================================================================
        # GRAVITY DISABLE: Remove gravity from package so arms can hold it
        # =================================================================
        if bool(node.bb.get("package_attached", False)):
            # Gravity disabled once at start of attach phase
            if not node.bb.get("package_gravity_disabled", False):
                if set_package_gravity(node, False):  # Disable gravity
                    node.bb["package_gravity_disabled"] = True
            
            # =================================================================
            # GRASP OFFSET CAPTURE (if hold control enabled)
            # =================================================================
            # Capture current EE-to-package offsets for later hold replanning
            if use_pkg_hold:
                live_state = _get_live_tp_state(node)
                if live_state is not None:
                    left_arm_jp, right_arm_jp, left_base, right_base = live_state
                    # Measure and store left/right EE offsets from package center
                    _capture_pkg_grasp_offsets(node, left_arm_jp, right_arm_jp, left_base, right_base)
                    
                    # Set target Z for lift phase based on config delta
                    if node._pkg_hold_start_z is not None:
                        node._pkg_hold_target_z = float(node._pkg_hold_start_z) + float(man_cfg.collect_lift_delta_z)
                        # Pre-compute trajectory for lift motion via hold control
                        _replan_pkg_hold_tp(
                            node,
                            left_arm_jp,
                            right_arm_jp,
                            left_base,
                            right_base,
                            pkg_z_target=node._pkg_hold_target_z,
                            force=True,  # Force replan on first call
                        )
            
            # =================================================================
            # ATTACH COMPLETION: Stop motion and transition to collect phase
            # =================================================================
            node.stop_all_movement()
            node.lift_phase = "collect"  # Move to next phase
            node.get_logger().info(bt_fmt("[LiftObj] attached, starting collect"))
            node.start_action_timer("LiftObj")  # Reset phase timer for collect
            return None  # RUNNING - stay in loop, next iteration enters collect

        # =================================================================
        # ATTACH WAIT: Package not yet attached, keep waiting
        # =================================================================
        node._warn_throttled(
            "lift_attach_wait",
            bt_fmt(f"[LiftObj] waiting package attach... elapsed={elapsed:.2f}s"),
            period_s=0.8,
        )
        rclpy.spin_once(node, timeout_sec=0.01)
        return None  # RUNNING - keep trying to attach

    # =================================================================
    # PHASE 3: COLLECT (synchronized base retreat + arm lift with optional hold)
    # =================================================================
    # Lift phase: bases retreat from pickup location while arms lift package.
    # If hold enabled: maintain package pose (XY + Z) via replanning.
    # If hold disabled: joint-space arm trajectory + base motion (legacy mode).
    if node.lift_phase == "collect":
        # =================================================================
        # COLLECT TIMING: Execute for full collect_time duration
        # =================================================================
        if elapsed < collect_time:
            # =================================================================
            # SENSOR DATA ACQUISITION: Get current joint/base poses
            # =================================================================
            live_state = _get_live_tp_state(node)
            if live_state is None:
                node._warn_throttled(
                    "lift_collect_wait_data",
                    bt_fmt("[LiftObj] collect waiting sensor data (joint_states/odom)"),
                    period_s=1.0,
                )
                rclpy.spin_once(node, timeout_sec=0.01)
                return None  # RUNNING - wait for sensor data

            left_arm_jp, right_arm_jp, left_base, right_base = live_state
            
            # =================================================================
            # BASE+ARM TP INIT: First iteration - compute target trajectory
            # =================================================================
            if not bool(node.bb.get("lift_collect_tp_initialized", False)):
                # Scale base velocity if object-centric hold is active
                base_scale = 1.0
                if use_pkg_hold and bool(node.bb.get("package_attached", False)):
                    base_scale = float(getattr(man_cfg, "hold_base_vel_scale", 1.0))
                
                # Predict world-frame base targets using body-frame velocities + collect duration
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
                
                # Initialize TP for base stage (retreat motion)
                if _init_tp_base_stage(
                    node,
                    left_base_goal_xy=left_target_xy,
                    right_base_goal_xy=right_target_xy,
                    period_s=float(max(collect_time, 0.2)),
                    kp_xy=float(getattr(man_cfg, "transport_retreat_kp_x", 1.0)),
                    kp_yaw=0.0,
                ):
                    # TP base init successful: store targets and mark initialized
                    node.bb["lift_collect_tp_initialized"] = True
                    node.bb["lift_collect_left_target_xy"] = left_target_xy
                    node.bb["lift_collect_right_target_xy"] = right_target_xy
                else:
                    # TP base init failed: cannot proceed
                    node._warn_throttled(
                        "lift_collect_base_tp_init_fail",
                        bt_fmt("[LiftObj] collect TP base init failed"),
                        period_s=2.0,
                    )
                    return False  # FAILURE

                # =================================================================
                # ARM CONTROL SELECTION: Object-centric hold vs legacy joint trajectory
                # =================================================================
                # If not using object-centric hold: compute and init joint-space arm trajectory
                if (not use_pkg_hold) and bool(node.bb.get("package_attached", False)):
                    # Legacy mode: direct joint increments for lift
                    q_left_goal = np.asarray(left_arm_jp, dtype=np.float32) + np.asarray(left_arm_collect, dtype=np.float32)
                    q_right_goal = np.asarray(right_arm_jp, dtype=np.float32) + np.asarray(right_arm_collect, dtype=np.float32)
                    q_left_goal, q_right_goal = _clip_descend_pick_joint_goals(node, q_left_goal, q_right_goal)
                    _init_tp_arm_joint_stage(node, q_left_goal, q_right_goal, period_s=float(max(collect_time, 0.2)))
                    node.bb["lift_collect_arm_goal_left"] = [float(v) for v in q_left_goal.tolist()]
                    node.bb["lift_collect_arm_goal_right"] = [float(v) for v in q_right_goal.tolist()]

            # =================================================================
            # TP EXECUTION: Send TP commands to controllers
            # =================================================================
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
            
            # =================================================================
            # OBJECT-CENTRIC HOLD REPLAN (if enabled)
            # =================================================================
            hold_executed = False
            if use_pkg_hold and bool(node.bb.get("package_attached", False)):
                # If Z target not yet set, compute from current package pose + lift delta
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
                
                # Replan arm trajectory to maintain package centered between EEs at target Z
                if _replan_pkg_hold_tp(
                    node,
                    left_arm_jp,
                    right_arm_jp,
                    left_base,
                    right_base,
                    pkg_z_target=node._pkg_hold_target_z,
                    preserve_jtc_base=True,  # Keep base TP active
                ):
                    # Hold replan successful
                    _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "collect")
                    _log_force_proxy(node, "collect", period_s=1.0)
                    hold_executed = True
            
            # =================================================================
            # FALLBACK WARNING: TP not available in hold mode
            # =================================================================
            if not hold_executed:
                node._warn_throttled(
                    "lift_collect_tp_required",
                    bt_fmt("[LiftObj] TP required: collect arm hold unavailable (open-loop disabled)"),
                    period_s=2.0,
                )

            node._info_throttled("lift_collect", bt_fmt("[LiftObj] collecting (lifting)..."), period_s=1.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            return None  # RUNNING - continue collecting
        else:
            node.stop_all_movement()
            node.bb.pop("lift_collect_tp_initialized", None)
            node.bb.pop("lift_collect_left_target_xy", None)
            node.bb.pop("lift_collect_right_target_xy", None)
            node.bb.pop("lift_collect_arm_goal_left", None)
            node.bb.pop("lift_collect_arm_goal_right", None)
            
            # =================================================================
            # OPTIONAL PRE-TRANSPORT: If enabled, reposition arms for transport
            # =================================================================
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
            
            # =================================================================
            # LIFTOBJ SUCCESS: Collect complete, action done
            # =================================================================
            node.lift_phase = None
            node.clear_action_timer("LiftObj")
            _phase_pause_reset(node, "LiftObj")
            node.get_logger().info(bt_fmt("[LiftObj] completed"))
            return True

    # =================================================================
    # PHASE 4: PRE_TRANSPORT_POSE - optional repositioning before transport
    # =================================================================
    # Optional phase that adjusts arm/base pose before moving package to delivery site
    # Two modes: "joint" (direct joint goal) or "package" (maintain grasp with EE offset)
    if node.lift_phase == "pre_transport_pose":
        # ================================================================
        # GET CURRENT STATE: Fetch sensor data for control loop
        # ================================================================
        # Sensor data acquisition: joint positions, base poses
        live_state = _get_live_tp_state(node)
        if live_state is None:
            # Still waiting for sensors (encoders, TF)
            node._warn_throttled(
                "lift_pre_transport_wait_data",
                bt_fmt("[LiftObj] waiting sensor data for pre-transport pose"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None  # RUNNING - retry next tick

        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        
        # ================================================================
        # MODE SELECTION: Joint-space vs package-centric control
        # ================================================================
        # Get pre-transport mode from config: "joint" or "package"
        # - "joint": Move arms directly to configured home-like pose
        # - "package": Maintain grasp with adjusted EE offset from package
        mode = str(getattr(man_cfg, "pre_transport_mode", "package")).strip().lower()
        if mode not in ("package", "joint"):
            mode = "package"  # Default to package if invalid mode
        
        # Initialize convergence flag (will be set by specific mode logic below)
        reached = False
        # Variabili condivise dal logging finale (devono essere sempre definite).
        hold_ok = False
        base_ok = True
        base_align_enable = False

        # ================================================================
        # JOINT MODE: Direct joint-space control to pre-transport pose
        # ================================================================
        if mode == "joint":
            # Check if TP trajectory has been initialized for this phase
            tp_ready = bool(node.bb.get("lift_pre_transport_tp_ready", False))
            if not tp_ready:
                # First run: plan TP trajectory from current pose to joint goal
                tp_ready = _init_pre_transport_tp(node, left_arm_jp=left_arm_jp, right_arm_jp=right_arm_jp)
                if not tp_ready:
                    # TP planning failed (unreachable, collision, etc.) - skip phase
                    node._warn_throttled(
                        "lift_pre_transport_disabled_runtime",
                        bt_fmt("[LiftObj] pre-transport pose skipped (TP init failed)"),
                        period_s=2.0,
                    )
                    # Skip this phase and move to completion
                    node.lift_phase = None
                    node.clear_action_timer("LiftObj")
                    _phase_pause_reset(node, "LiftObj")
                    node.get_logger().info(bt_fmt("[LiftObj] completed"))
                    return True  # SUCCESS - action complete

            # ================================================================
            # JOINT MODE EXECUTION: Execute TP joint trajectory
            # ================================================================
            # Execute arm control only (no base movement in joint mode)
            _execute_tp_arm_control(
                node,
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
                arm_clip_abs=float(getattr(man_cfg, "pre_transport_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
            )

            # ================================================================
            # JOINT MODE CONVERGENCE: Check if arms reached pre-transport joint goal
            # ================================================================
            # Get target joint configurations from blackboard (set by _init_pre_transport_tp)
            q_left_goal = node.bb.get("lift_pre_transport_goal_left", None)
            q_right_goal = node.bb.get("lift_pre_transport_goal_right", None)
            # Compute max joint error: max(|q_current - q_goal|) across all joints
            l_err = _max_joint_error(node, left_arm_jp, q_left_goal)
            r_err = _max_joint_error(node, right_arm_jp, q_right_goal)
            # Get convergence tolerance (radians) for joint-space control
            tol = float(getattr(man_cfg, "pre_transport_joint_tol", 0.12))
            # Check if both arms converged to their joint goals (or very close)
            reached = bool(np.isfinite(l_err) and np.isfinite(r_err) and l_err <= tol and r_err <= tol)
            
            # Log tracking info
            node._info_throttled(
                "lift_pre_transport_track",
                bt_fmt(
                    f"[LiftObj] pre-transport pose tracking (joint) "
                    f"l_err={l_err:.4f}, r_err={r_err:.4f}, tol={tol:.4f}, reached={reached}"
                ),
                period_s=1.0,
            )
            # Log force/torque sensor info for debugging
            _log_force_proxy(node, "pre_transport_joint", period_s=1.0)
            hold_ok = bool(reached)
            base_ok = True
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

            # ================================================================
            # ARM HOLD TRAJECTORY PLANNING: Replan arm trajectories for new offsets
            # ================================================================
            # Replan TP trajectory for arm hold: arms maintain package position 
            # while obeying new offset geometry. preserve_jtc_base controls whether
            # we also compute base trajectories in same planning stage.
            base_align_enable = bool(getattr(man_cfg, "pre_transport_base_align_enable", True))
            hold_ok = False
            # Call TP planning with new offsets and Z target
            if _replan_pkg_hold_tp(
                node,
                left_arm_jp,
                right_arm_jp,
                left_base,
                right_base,
                pkg_z_target=node._pkg_hold_target_z,  # Keep current package Z height
                preserve_jtc_base=base_align_enable,  # Coordinate base planning if enabled
            ):
                # TP planning succeeded: check if arm trajectory is feasible
                hold_ok = bool(node.bb.get("pkg_hold_ok", False))

            # ================================================================
            # BASE ALIGNMENT (OPTIONAL): Optionally reposition bases while holding
            # ================================================================
            base_ok = True  # Assume base OK if alignment disabled
            l_dist = float("nan")
            r_dist = float("nan")
            if base_align_enable:
                # Compute new base targets for pre-transport formation
                # Position bases closer to package (for better visibility, stability)
                l_target = [
                    float(pkg_xyz[0]) + float(getattr(man_cfg, "pre_transport_base_left_offset_x", -0.60)),
                    float(pkg_xyz[1]) + float(getattr(man_cfg, "pre_transport_base_offset_y", -0.70)),
                ]
                r_target = [
                    float(pkg_xyz[0]) + float(getattr(man_cfg, "pre_transport_base_right_offset_x", 0.60)),
                    float(pkg_xyz[1]) + float(getattr(man_cfg, "pre_transport_base_offset_y", -0.70)),
                ]
                # Initialize TP base trajectory if not already done
                if not bool(node.bb.get("lift_pre_transport_base_tp_initialized", False)):
                    # Plan base trajectory from current positions to pre-transport targets
                    if _init_tp_base_stage(
                        node,
                        left_base_goal_xy=l_target,
                        right_base_goal_xy=r_target,
                        period_s=float(max(getattr(man_cfg, "pre_transport_traj_time", 2.5), 0.2)),
                        kp_xy=float(getattr(man_cfg, "pre_transport_base_kp_x", 0.9)),  # PD gain for XY control
                        kp_yaw=0.0,  # Don't rotate bases during pre-transport
                    ):
                        # TP planning succeeded: mark base stage initialized
                        node.bb["lift_pre_transport_base_tp_initialized"] = True
                        # Store base targets in blackboard for convergence check
                        node.bb["lift_pre_transport_base_left_target_xy"] = l_target
                        node.bb["lift_pre_transport_base_right_target_xy"] = r_target
                # Retrieve base targets from blackboard (for convergence checking)
                l_ref = node.bb.get("lift_pre_transport_base_left_target_xy", l_target)
                r_ref = node.bb.get("lift_pre_transport_base_right_target_xy", r_target)
                
                # ================================================================
                # BASE CONVERGENCE CHECK: Distance-based convergence for base positions
                # ================================================================
                # Compute Euclidean distance from current left base to target XY
                l_dist = float(math.hypot(float(l_ref[0]) - float(left_base[0]), float(l_ref[1]) - float(left_base[1])))
                # Compute Euclidean distance from current right base to target XY
                r_dist = float(math.hypot(float(r_ref[0]) - float(right_base[0]), float(r_ref[1]) - float(right_base[1])))
                # Get convergence tolerance (meters) for base targets
                base_tol = float(getattr(man_cfg, "pre_transport_base_goal_tol", 0.10))
                # Check if BOTH bases reached pre-transport targets
                base_ok = bool(l_dist <= base_tol and r_dist <= base_tol)

            # ================================================================
            # TP CONTROLLER EXECUTION: Execute arm + optional base control
            # ================================================================
            if base_align_enable:
                # Execute full control: arm hold + base alignment trajectories
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
                # Execute arm-only control: hold package without base movement
                _execute_tp_arm_hold(node, left_arm_jp, right_arm_jp, left_base, right_base)

            # ================================================================
            # LOGGING: Monitor hold quality and convergence status
            # ================================================================
            # Log package hold quality metrics (contact forces, offset errors, etc.)
            _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "pre_transport")
            # Log force/torque sensor readings for debugging
            _log_force_proxy(node, "pre_transport_pkg", period_s=1.0)
            # Log base alignment progress if enabled
            if base_align_enable:
                node._info_throttled(
                    "lift_pre_transport_base_align",
                    bt_fmt(
                        f"[LiftObj] pre-transport base align (TP) "
                        f"L_dist={l_dist:.3f}, R_dist={r_dist:.3f}, ok={base_ok}"
                    ),
                    period_s=1.0,
                )

            # ================================================================
            # CONVERGENCE DETECTION: Check if pre-transport stage complete
            # ================================================================
            # Overall convergence: arm hold is stable AND (no base align OR bases converged)
            reached = bool(hold_ok and base_ok)
            # Log tracking info
            node._info_throttled(
                "lift_pre_transport_track",
                bt_fmt(
                    f"[LiftObj] pre-transport pose tracking (package) "
                    f"hold_ok={hold_ok}, base_ok={base_ok}, reached={reached}, "
                    f"half_span={half_span:.3f}"
                ),
                period_s=1.0,
            )

        # ================================================================
        # TIMEOUT CHECK + PHASE COMPLETION
        # ================================================================
        # Get action timer (tracks elapsed time since LiftObj started)
        phase_t0 = node.get_action_timer("LiftObj")
        # Compute elapsed time since phase started
        phase_elapsed = (_ros_now_s(node) - float(phase_t0)) if phase_t0 is not None else 0.0
        # Compute timeout: max of config timeout and TP trajectory time + margin
        timeout = max(
            float(getattr(man_cfg, "pre_transport_stage_timeout", 6.0)),
            float(getattr(man_cfg, "pre_transport_traj_time", 2.5)) + 0.2,
        )
        # Log elapsed time / timeout
        node._info_throttled(
            "lift_pre_transport_elapsed",
            bt_fmt(f"[LiftObj] pre-transport elapsed={phase_elapsed:.2f}/{timeout:.2f}s"),
            period_s=1.0,
        )
        # Check completion: either normal convergence OR timeout reached
        if reached or phase_elapsed >= timeout:
            # ================================================================
            # PHASE COMPLETE: Stop movement and clean up blackboard
            # ================================================================
            # Stop all controllers (arm + base)
            node.stop_all_movement()
            # Reset TP ready flag for next potential pre-transport
            node.bb["lift_pre_transport_tp_ready"] = False
            # Clean blackboard: remove goals and targets (no longer needed)
            node.bb.pop("lift_pre_transport_goal_left", None)
            node.bb.pop("lift_pre_transport_goal_right", None)
            node.bb.pop("lift_pre_transport_base_tp_initialized", None)
            node.bb.pop("lift_pre_transport_base_left_target_xy", None)
            node.bb.pop("lift_pre_transport_base_right_target_xy", None)
            # Clear phase tracker (triggers complete action)
            node.lift_phase = None
            # Clear action timer (action is complete)
            node.clear_action_timer("LiftObj")
            # Reset pause state for LiftObj action
            _phase_pause_reset(node, "LiftObj")
            # Log completion status
            if phase_elapsed >= timeout and (not reached):
                node.get_logger().warn(bt_fmt("[LiftObj] pre-transport pose timeout, continuing"))
            node.get_logger().info(bt_fmt("[LiftObj] completed"))
            return True  # SUCCESS - LiftObj action complete

        rclpy.spin_once(node, timeout_sec=0.01)
        return None

    # fallback
    node.lift_phase = None
    node.clear_action_timer("LiftObj")
    _phase_pause_reset(node, "LiftObj")
    return True


def MoveBase():
    # ================================================================
    # ACTION: MoveBase - Transport package from pickup to delivery location
    # ================================================================
    # Two-phase movement: retreat from pickup location, then transport to destination
    # Optionally maintains arm joint-space hold to prevent package rotation during transport
    # Supports object-centric hold mode for coordinated dual-arm package manipulation
    
    # Trasporto pacco:
    # - inizializza target base left/right coerenti con il riferimento pacco,
    # - opzionalmente blocca i bracci in joint-space,
    # - usa TP full-control per avanzare in modo sincronizzato.
    # In modalita' hold object-centric aggiorna periodicamente i goal EE da _replan_pkg_hold_tp.
    """
    Movimento delle basi (transfer):
    State-machine runtime (`node.bb['movebase_stage']`):
    - retreat -> transport -> done
    - TP puro in 2 segmenti:
      1) retreat: allontanamento dal pallet
      2) transport: traslazione "a blocco rigido" verso destinazione world
         (default: model `pacco_clone_2` con offset Y=-2.0m).
    """
    # Get node + config parameters
    node = _require_node()
    phase_cfg = node.cfg.phases
    motion = node.cfg.motion_profiles
    man_cfg = node.cfg.manipulation
    # Get expected duration for transport phase
    transport_time = float(phase_cfg.transport_time)
    # Get velocity profiles (body-frame command, used if world destination not available)
    left_transport_xy = _float_vec(motion.left_transport_vel_xy)
    right_transport_xy = _float_vec(motion.right_transport_vel_xy)
    # Control mode flags
    use_pkg_hold = bool(man_cfg.enable_object_centric_hold)  # Maintain grasp during transport?
    use_retreat = bool(getattr(man_cfg, "transport_retreat_enable", True))  # Retreat phase?
    lock_arms = bool(getattr(man_cfg, "transport_lock_arm_joints", True))  # Lock arms in place?
    # Arm command amplitude adjusted based on lock mode
    # (if locking, use small gains to maintain position; if free, use normal hold gains)
    movebase_arm_clip_abs = float(
        max(
            1e-4,
            float(getattr(man_cfg, "transport_lock_arm_cmd_abs_max", 0.22))
            if lock_arms
            else float(getattr(man_cfg, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
        )
    )
    lock_arm_kp = float(getattr(man_cfg, "transport_lock_arm_kp", node.approach_jtc_arm_kp))

    # ================================================================
    # HELPER 1: Get/compute arm joint goals for hold mode
    # ================================================================
    def _get_transport_arm_hold_goals(left_arm_jp, right_arm_jp, force_refresh: bool = False):
        """
        Get target joint positions for arm hold during transport.
        On first call, capture current arm position as hold goal.
        Subsequent calls reuse same goal unless force_refresh=True.
        """
        # Retrieve stored arm hold goals from blackboard (if available)
        gl = node.bb.get("movebase_transport_arm_goal_left", None)
        gr = node.bb.get("movebase_transport_arm_goal_right", None)
        # First run or refresh: capture current left arm position as goal
        if force_refresh or (not isinstance(gl, (list, tuple)) or len(gl) != 6):
            # Convert numpy array to list for storage
            gl = [float(v) for v in np.asarray(left_arm_jp, dtype=np.float32).tolist()]
            node.bb["movebase_transport_arm_goal_left"] = list(gl)
        # First run or refresh: capture current right arm position as goal
        if force_refresh or (not isinstance(gr, (list, tuple)) or len(gr) != 6):
            gr = [float(v) for v in np.asarray(right_arm_jp, dtype=np.float32).tolist()]
            node.bb["movebase_transport_arm_goal_right"] = list(gr)
        return gl, gr

    # ================================================================
    # HELPER 2: Initialize base + arm trajectories for locked-arm hold
    # ================================================================
    def _init_transport_base_arm_hold(left_target_xy, right_target_xy, left_arm_jp, right_arm_jp, period_s: float, kp_xy: float) -> bool:
        """
        Plan TP trajectories for both base (2D XY movement) and arm (fix joint angles).
        Used when lock_arms=True to prevent package rotation during transport.
        Returns True if both base and arm planning succeeded.
        """
        # Initialize base trajectory: move base to target XYs (no yaw rotation)
        ok_base = _init_tp_base_stage(
            node,
            left_base_goal_xy=left_target_xy,
            right_base_goal_xy=right_target_xy,
            period_s=float(max(period_s, 0.2)),
            kp_xy=kp_xy,
            kp_yaw=0.0,  # Don't rotate during transport
        )
        # Get current arm positions as hold goals (lock arms in place)
        arm_goal_l, arm_goal_r = _get_transport_arm_hold_goals(left_arm_jp, right_arm_jp, force_refresh=True)
        # Initialize arm trajectory: hold current joint angles (no movement)
        ok_arm = _init_tp_arm_joint_stage(
            node,
            left_arm_goal=arm_goal_l,
            right_arm_goal=arm_goal_r,
            period_s=float(max(period_s, 0.2)),
            kp_arm=lock_arm_kp,  # Use lock gains (small, stiff control)
        )
        # Ensure both base + arm are active in same JTC stage (avoid deactivation conflicts)
        if bool(ok_base and ok_arm) and (node.approach_jtc_task is not None):
            try:
                node.approach_jtc_task.activate()
                node.approach_jtc_task.set_activation("base", True)
                node.approach_jtc_task.set_activation("arm", True)
            except Exception:
                return False
            return True
        return False

    # ================================================================
    # HELPER 3: Initialize transport stage with destination resolution
    # ================================================================
    def _init_transport_stage(left_arm_jp, right_arm_jp, left_base_pose, right_base_pose, period_s: float):
        """
        Compute transport target (world-frame XY position) using delivery destination.
        Then initialize TP planning for base + optional arm movement.
        Returns True if all planning succeeded.
        """
        left_target_xy = None
        right_target_xy = None

        # Resolve world-frame delivery destination (typically pallet location)
        dst_pkg_xy, dst_source = _resolve_transport_destination_xy(node)
        # If valid destination found, compute base movement needed
        if isinstance(dst_pkg_xy, (list, tuple)) and len(dst_pkg_xy) >= 2:
            # Compute base targets based on package reference (not single EE estimate)
            # to avoid bias from asymmetric grasp.
            pkg_ref_xyz = _get_live_package_xyz(node)
            if pkg_ref_xyz is None:
                # Fallback: estimate package position from grasp geometry
                pkg_ref_xyz = _resolve_pkg_reference_xyz(
                    node,
                    left_arm_jp=left_arm_jp,
                    right_arm_jp=right_arm_jp,
                    left_base=left_base_pose,
                    right_base=right_base_pose,
                )
            if isinstance(pkg_ref_xyz, (list, tuple)) and len(pkg_ref_xyz) >= 2:
                # Compute displacement vector (destination - current package position)
                dx = float(dst_pkg_xy[0]) - float(pkg_ref_xyz[0])
                dy = float(dst_pkg_xy[1]) - float(pkg_ref_xyz[1])
                # Apply displacement to each base's current position (rigid body assumption)
                left_target_xy = [float(left_base_pose[0]) + dx, float(left_base_pose[1]) + dy]
                right_target_xy = [float(right_base_pose[0]) + dx, float(right_base_pose[1]) + dy]
                # Store destination in blackboard (for logging/status)
                node.bb["movebase_transport_dst_pkg_xy"] = [float(dst_pkg_xy[0]), float(dst_pkg_xy[1])]
            else:
                dst_source = f"{dst_source}|no_pkg_ref"
        else:
            dst_source = f"{dst_source}|no_dst"

        # Fallback legacy mode: if world destination unavailable, use body-frame velocity profile
        if left_target_xy is None or right_target_xy is None:
            # Scale velocity if object-centric hold active and package attached
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
            # Compute endpoint of body-frame velocity trajectory over transport_time duration
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
        # ================================================================
        # ACTION INIT: Check pause, reset state, setup retreat/transport phases
        # ================================================================
        # Check if action is paused: don't proceed if system is paused
        if not _phase_pause_gate(node, "MoveBase"):
            node.stop_all_movement()
            return None  # RUNNING - on pause, retry next tick
        
        # Log action started with expected duration
        node.get_logger().info(bt_fmt(f"[MoveBase] start ({transport_time}s)"))
        # Reset runtime state (timers, flags) for fresh action execution
        _reset_movebase_runtime(node)

        # ================================================================
        # RETREAT PHASE (OPTIONAL): Move bases away from pickup location
        # ================================================================
        # If retreat enabled, plan and execute base retreat before transport
        if use_retreat:
            # Get sensor data: joint positions and base poses
            live_state = _get_live_tp_state(node)
            if live_state is not None:
                _la, _ra, left_base, right_base = live_state
                # Resolve package reference position for retreat goal computation
                pkg_xyz = _resolve_pkg_reference_xyz(
                    node,
                    left_arm_jp=_la,
                    right_arm_jp=_ra,
                    left_base=left_base,
                    right_base=right_base,
                )
                if pkg_xyz is None:
                    # Fallback: get Gazebo package position
                    pkg_xyz = _get_live_package_xyz(node)
            else:
                pkg_xyz = None
            
            # If both sensor data and package pose available, setup retreat
            if live_state is not None and pkg_xyz is not None:
                # Compute retreat target: move in -Y direction (away) by configured offset
                retreat_y = float(pkg_xyz[1]) - float(getattr(man_cfg, "transport_retreat_pkg_backoff_y", 2.0))
                # Left base retreat target with optional X offset
                left_target = [
                    float(left_base[0]) + float(getattr(man_cfg, "transport_retreat_left_offset_x", 0.0)),
                    retreat_y,
                ]
                # Right base retreat target with optional X offset (asymmetric)
                right_target = [
                    float(right_base[0]) + float(getattr(man_cfg, "transport_retreat_right_offset_x", 0.0)),
                    retreat_y,
                ]
                # Store retreat targets for tracking during execution
                node.bb["movebase_retreat_left_target_xy"] = left_target
                node.bb["movebase_retreat_right_target_xy"] = right_target
                
                # Initialize TP for retreat: use base+arm hold if arms locked, base-only otherwise
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
                
                # Check if retreat planning succeeded
                if ok_retreat:
                    # Retreat TP initialized successfully: set stage and log
                    node.bb["movebase_stage"] = "retreat"
                    node.get_logger().info(
                        bt_fmt(
                            "[MoveBase] retreat stage initialized "
                            f"(L_target={np.round(np.asarray(left_target), 3).tolist()}, "
                            f"R_target={np.round(np.asarray(right_target), 3).tolist()}, "
                            f"pkg_y={float(pkg_xyz[1]):.3f})")
                    )
                else:
                    # Retreat TP planning failed: fallback to direct transport
                    if _init_transport_stage(_la, _ra, left_base, right_base, period_s=transport_time):
                        # Transport stage initialized successfully as fallback
                        node.get_logger().warn(
                            bt_fmt("[MoveBase] retreat TP init failed, fallback to direct transport stage")
                        )
                    else:
                        # Both retreat and transport failed: set transport stage anyway to retry
                        node.bb["movebase_stage"] = "transport"
                        node._warn_throttled(
                            "movebase_retreat_init_tp_fail",
                            bt_fmt("[MoveBase] retreat TP init failed and transport init failed"),
                            period_s=2.0,
                        )
            else:
                # Sensor data or package position not available: skip retreat, go directly to transport
                node.bb["movebase_stage"] = "transport"
                # If sensor data available, try to initialize transport stage
                if live_state is not None:
                    _la, _ra, left_base, right_base = live_state
                    _init_transport_stage(_la, _ra, left_base, right_base, period_s=transport_time)
                # Log warning: retreat skipped, proceeding directly to transport
                node._warn_throttled(
                    "movebase_retreat_init_fallback",
                    bt_fmt("[MoveBase] retreat stage skipped (missing live state/package pose), fallback to transport"),
                    period_s=2.0,
                )
        else:
            # ================================================================
            # DIRECT TRANSPORT (no retreat): Initialize transport stage immediately
            # ================================================================
            # Get sensor data for transport initialization
            live_state = _get_live_tp_state(node)
            if live_state is not None:
                _la, _ra, left_base, right_base = live_state
                # Try to initialize transport stage; if fails, mark as transport anyway to retry
                if not _init_transport_stage(_la, _ra, left_base, right_base, period_s=transport_time):
                    node.bb["movebase_stage"] = "transport"
            else:
                # No sensor data yet: set to transport stage (will retry)
                node.bb["movebase_stage"] = "transport"
        
        # Start action timer: track elapsed time for phase timeouts
        t0 = node.start_action_timer("MoveBase")

    # Get current stage and current time
    stage = str(node.bb.get("movebase_stage", "transport"))
    now_s = _ros_now_s(node)

    # ================================================================
    # RETREAT EXECUTION: Monitor and execute base retreat movement
    # ================================================================
    if stage == "retreat":
        # Get current sensor state: arm joint positions, base poses
        live_state = _get_live_tp_state(node)
        if live_state is None:
            # Waiting for sensor data (joint encoders, odometry)
            node._warn_throttled(
                "movebase_retreat_wait_data",
                bt_fmt("[MoveBase] retreat waiting sensor data (joint_states/odom)"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None  # RUNNING - retry next tick

        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        # Get retreat targets from blackboard (set during INIT)
        left_target = node.bb.get("movebase_retreat_left_target_xy", None)
        right_target = node.bb.get("movebase_retreat_right_target_xy", None)
        # Validate retreat targets are available
        if (not isinstance(left_target, (list, tuple))) or (not isinstance(right_target, (list, tuple))):
            # Targets invalid/missing: fallback to transport stage
            node._warn_throttled(
                "movebase_retreat_invalid_targets",
                bt_fmt("[MoveBase] retreat targets unavailable, switching to transport"),
                period_s=2.0,
            )
            node.bb["movebase_stage"] = "transport"
            # Try to initialize transport as fallback
            if not _init_transport_stage(left_arm_jp, right_arm_jp, left_base, right_base, period_s=transport_time):
                node._warn_throttled(
                    "movebase_transport_init_after_retreat_invalid",
                    bt_fmt("[MoveBase] transport TP init failed after invalid retreat target"),
                    period_s=2.0,
                )
            node.start_action_timer("MoveBase")
            rclpy.spin_once(node, timeout_sec=0.01)
            return None  # RUNNING - retry with new stage

        # Execute TP control: move bases toward retreat targets
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

        # Check convergence: distance to targets
        l_dist = float(math.hypot(float(left_target[0]) - float(left_base[0]), float(left_target[1]) - float(left_base[1])))
        r_dist = float(math.hypot(float(right_target[0]) - float(right_base[0]), float(right_target[1]) - float(right_base[1])))
        tol = float(getattr(man_cfg, "transport_retreat_goal_tol", 0.10))
        l_reached = bool(l_dist <= tol)
        r_reached = bool(r_dist <= tol)

        # Replan package hold if active and arm not locked (free-arm mode)
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
        # Log package hold quality if active
        if use_pkg_hold and bool(node.bb.get("package_attached", False)):
            _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "retreat")

        # Track elapsed time and check timeout
        retreat_elapsed = now_s - float(t0)
        retreat_timeout = float(max(0.5, float(getattr(man_cfg, "transport_retreat_stage_timeout", 12.0))))
        retreat_reached = bool(l_reached and r_reached)
        # Log progress
        node._info_throttled(
            "movebase_retreat_track",
            bt_fmt(
                f"[MoveBase] retreat tracking L_dist={l_dist:.3f}, R_dist={r_dist:.3f}, "
                f"reached={retreat_reached}, elapsed={retreat_elapsed:.2f}/{retreat_timeout:.2f}s"
            ),
            period_s=1.0,
        )

        # Check completion: both bases reached OR timeout exceeded
        if retreat_reached or retreat_elapsed >= retreat_timeout:
            node.stop_all_movement()
            # Transition to transport stage
            node.bb["movebase_stage"] = "transport"
            # Initialize transport stage
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

    # =================================================================
    # TRANSPORT EXECUTION: Move bases to destination while holding package
    # =================================================================
    # Stage 2: transport point-to-point via TP
    # Moves both bases from pickup location toward delivery destination in rigid-body formation
    # Optionally maintains arm joint-space hold to prevent package rotation
    if stage == "transport" and (now_s - float(t0) < max(transport_time, 0.2) + 2.0):
        # ===== SENSOR DATA ACQUISITION =====
        # Get live robot state: arm joint positions + base poses (XY, theta)
        # Required for TP planning and convergence checking
        live_state = _get_live_tp_state(node)
        if live_state is None:
            # Waiting for sensor initialization (joint_states topic + odometry)
            # Common at action startup before first joint/odometry messages arrive
            node._warn_throttled(
                "movebase_transport_wait_data",
                bt_fmt("[MoveBase] transport waiting sensor data (joint_states/odom)"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None  # RUNNING - retry next tick
        left_arm_jp, right_arm_jp, left_base, right_base = live_state

        # ===== LAZY INITIALIZATION: INITIALIZE TRANSPORT STAGE IF NOT ALREADY DONE =====
        # Check if transport targets already initialized (e.g., from init phase or previous tick)
        # If not available, attempt to initialize now (handles edge case where init was skipped)
        l_target_ref = node.bb.get("movebase_transport_left_target_xy", None)
        r_target_ref = node.bb.get("movebase_transport_right_target_xy", None)
        if (not isinstance(l_target_ref, (list, tuple))) or (not isinstance(r_target_ref, (list, tuple))):
            # Transport targets missing: attempt lazy initialization (resolve destination, plan trajectories)
            if not _init_transport_stage(left_arm_jp, right_arm_jp, left_base, right_base, period_s=transport_time):
                # Initialization failed: could not resolve delivery destination or TP planning failed
                node._warn_throttled(
                    "movebase_transport_lazy_init_fail",
                    bt_fmt("[MoveBase] transport target init failed"),
                    period_s=2.0,
                )
                rclpy.spin_once(node, timeout_sec=0.01)
                return None  # RUNNING - retry next tick

        # ===== TP FULL CONTROL EXECUTION =====
        # Execute trajectory planner commands for base movement (+ optional locked arm hold)
        # Computes and sends velocity commands: base XY motion + optional arm joint locking
        # Velocity limits: arm clamped to movebase_arm_clip_abs, base XY to 0.20 m/s max
        _execute_tp_full_control(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
            arm_clip_abs=movebase_arm_clip_abs,  # Lock or hold gains based on lock_arms flag
            base_xy_abs_max=float(getattr(man_cfg, "transport_retreat_cmd_xy_abs_max", 0.20)),  # Max base XY velocity
            base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),  # Max rotation velocity
        )

        # ===== RETRIEVE TRANSPORT TARGETS & COMPUTE BASE DISTANCES =====
        # Get target destinations for both bases (set during init)
        left_target = node.bb.get("movebase_transport_left_target_xy", None)
        right_target = node.bb.get("movebase_transport_right_target_xy", None)
        
        # Calculate Euclidean distance from each base's current position to target
        # Using math.hypot() for numerically stable distance computation
        l_dist = float("inf")  # Initialize to infinity (will be computed if target valid)
        r_dist = float("inf")
        if isinstance(left_target, (list, tuple)) and len(left_target) >= 2:
            # Left base distance: sqrt((target_x - current_x)² + (target_y - current_y)²)
            l_dist = float(math.hypot(float(left_target[0]) - float(left_base[0]), float(left_target[1]) - float(left_base[1])))
        if isinstance(right_target, (list, tuple)) and len(right_target) >= 2:
            # Right base distance: sqrt((target_x - current_x)² + (target_y - current_y)²)
            r_dist = float(math.hypot(float(right_target[0]) - float(right_base[0]), float(right_target[1]) - float(right_base[1])))
        
        # ===== BASE CONVERGENCE CHECK =====
        # Goal tolerance for base position convergence (typically ±0.10m)
        tol = float(getattr(man_cfg, "transport_destination_goal_tol", 0.10))
        # Both bases must be within tolerance distance of their targets to declare convergence
        transport_reached = bool(l_dist <= tol and r_dist <= tol)

        # ===== ARM CONVERGENCE CHECK (if lock_arms=True) =====
        # Initialize arm hold status (default: OK if lock_arms not enabled)
        arm_hold_ok = True
        l_arm_err = float("nan")  # Left arm maximum joint error (to be computed)
        r_arm_err = float("nan")  # Right arm maximum joint error (to be computed)
        
        if lock_arms:
            # If arms are locked during transport, check they maintain their hold positions
            # Retrieve target joint positions for both arms (set during init)
            q_left_goal = node.bb.get("movebase_transport_arm_goal_left", None)
            q_right_goal = node.bb.get("movebase_transport_arm_goal_right", None)
            
            # Calculate maximum joint error (worst-case joint) for each arm
            # Ensures all joints stay within tolerance, not just average
            l_arm_err = _max_joint_error(node, left_arm_jp, q_left_goal)
            r_arm_err = _max_joint_error(node, right_arm_jp, q_right_goal)
            
            # Arm hold convergence: both arms' max errors within joint tolerance threshold
            # Ensures arms maintain package hold configuration during base transport
            arm_hold_ok = bool(
                np.isfinite(l_arm_err)  # Valid error computed (not errored)
                and np.isfinite(r_arm_err)
                and l_arm_err <= float(node.approach_arm_joint_tol)  # Left arm OK
                and r_arm_err <= float(node.approach_arm_joint_tol)  # Right arm OK
            )
            # Transport only reaches goal when BOTH bases AND arms converge (if locked)
            transport_reached = bool(transport_reached and arm_hold_ok)

        # ===== PACKAGE HOLD REPLANNING (object-centric mode, free-arm config) =====
        # If package hold active AND arms NOT locked: continuously replan arm targets to maintain grasp
        # (Locked arm config doesn't need replanning since arms fixed in joint space)
        if use_pkg_hold and bool(node.bb.get("package_attached", False)) and (not lock_arms):
            # Replan package hold trajectory to adjust for any package movement during transport
            # Maintains centered grasp even as package might shift slightly
            if _replan_pkg_hold_tp(
                node,
                left_arm_jp,
                right_arm_jp,
                left_base,
                right_base,
                pkg_z_target=node._pkg_hold_target_z,  # Maintain package at hold Z height
                preserve_jtc_base=True,  # Don't change base trajectory, only arm goals
            ):
                # Replanning succeeded: log internal proxy forces for diagnostics
                _log_force_proxy(node, "transport", period_s=1.0)
        
        # ===== LOGGING: HOLD QUALITY MONITORING =====
        # Log package hold quality metrics (grasp offsets, symmetry, etc.) every second
        # Helps identify hold degradation if package slipping or gripper issue
        if use_pkg_hold and bool(node.bb.get("package_attached", False)):
            _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "transport")
        # ===== TRACKING LOG: PERIODIC STATUS OUTPUT =====
        # Log transport progress once per second: distances, arm hold status, convergence state
        # Helps operator diagnose slow convergence or stuck movement
        node._info_throttled(
            "movebase_transport_track",
            bt_fmt(
                f"[MoveBase] transport tracking src={node.bb.get('movebase_transport_source', 'n/a')}, "  # Destination source (world/legacy)
                f"L_dist={l_dist:.3f}, R_dist={r_dist:.3f}, "  # Distance to targets (meters)
                f"arm_lock={lock_arms}, arm_ok={arm_hold_ok}, "  # Arm control status
                f"L_arm_err={l_arm_err:.3f}, R_arm_err={r_arm_err:.3f}, reached={transport_reached}"  # Arm errors + convergence
            ),
            period_s=1.0,
        )
        
        # ===== TIMEOUT CHECKING & COMPLETION DECISION =====
        # Calculate action elapsed time (from start of MoveBase, includes retreat if any)
        transport_elapsed = now_s - float(t0)
        # Overall timeout: transport planning time + 2 second margin for communication delays
        transport_timeout = float(max(transport_time, 0.2) + 2.0)
        
        # ===== COMPLETION: CONVERGENCE OR TIMEOUT =====
        if transport_reached or (transport_elapsed >= transport_timeout):
            # ===== STOP ALL MOVEMENT =====
            # Halt TP-controlled base motion (transport destination reached or timeout)
            node.stop_all_movement()
            
            # ===== CLEAR ACTION TIMER & RESET STATE =====
            # Remove MoveBase action timer (marks action finished in runtime)
            node.clear_action_timer("MoveBase")
            # Reset internal state variables (timers, targets, etc.)
            _reset_movebase_runtime(node)
            # Clear pause state (allows next MoveBase if paused during execution)
            _phase_pause_reset(node, "MoveBase")
            
            # ===== TIMEOUT HANDLING =====
            # If timeout reached without convergence, log warning but continue
            # Soft timeout allows action to complete even if bases slightly off-target
            if transport_elapsed >= transport_timeout and (not transport_reached):
                node.get_logger().warn(bt_fmt("[MoveBase] transport timeout reached, continuing"))
            
            # ===== ACTION COMPLETION =====
            node.get_logger().info(bt_fmt("[MoveBase] completed"))
            return True  # Action SUCCESS - proceed to next phase
        
        rclpy.spin_once(node, timeout_sec=0.01)
        return None  # RUNNING - continue transport execution

    node.stop_all_movement()
    node.clear_action_timer("MoveBase")
    _reset_movebase_runtime(node)
    _phase_pause_reset(node, "MoveBase")
    node.get_logger().info(bt_fmt("[MoveBase] completed"))
    return True


def Drop():
    # ================================================================
    # ACTION: Drop - Lower and release package at delivery location
    # ================================================================
    # Two-phase approach: base alignment then arm descent with package hold
    # Maintains package center between grippers while lowering to placement height
    
    """
    Calata e posizionamento del pacco in zona place.
    State-machine runtime (`node.bb['drop_stage']`):
    - base_align -> descend_hold -> done
    Modalita' package-centric (default):
    - stage 1: base_align (TP base + hold) verso formazione frontale target
    - stage 2: descend_hold (base ferma, TP bracci/hold) fino a quota release.

    Fallback legacy (quando hold non attivo):
    - basi + bracci verso target TP classico.
    """
    # Get node + config parameters
    node = _require_node()
    phase_cfg = node.cfg.phases
    motion = node.cfg.motion_profiles
    man_cfg = node.cfg.manipulation
    # Expected time for descent and placement phase
    descend_and_place_time = float(phase_cfg.descend_and_place_time)
    # Velocity profiles for place movement
    place_left_base_xy = _float_vec(motion.place_left_base_xy_vel)
    place_right_base_xy = _float_vec(motion.place_right_base_xy_vel)
    left_arm_place = _float_vec(motion.left_arm_place)
    right_arm_place = _float_vec(motion.right_arm_place)
    # Package hold control for descent: maintain centered, lower Z
    use_pkg_hold = bool(man_cfg.enable_object_centric_hold) and bool(node.bb.get("package_attached", False))

    # ================================================================
    # TIMER CHECK: First run or continuation
    # ================================================================
    t0 = node.get_action_timer("Drop")
    if t0 is None:
        # ================================================================
        # INIT: First-run setup - compute drop target, base formation, TP init
        # ================================================================
        # Check pause state: don't proceed if paused
        if not _phase_pause_gate(node, "Drop"):
            return None  # RUNNING - on pause, retry next tick
        
        # Get sensor data: arm joint positions, base poses
        live_state = _get_live_tp_state(node)
        if live_state is None:
            # Waiting for sensor initialization
            node._warn_throttled("drop_wait_data_init", bt_fmt("[Drop] waiting sensor data before init"), period_s=1.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            return None  # RUNNING - retry next tick
        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        
        # Log action started with expected duration
        node.get_logger().info(bt_fmt(f"[Drop] start TP ({descend_and_place_time}s)"))

        # ================================================================
        # RESOLVE DROP TARGET: Get delivery location (XYZ coordinates)
        # ================================================================
        # Resolve drop target: tries config → Gazebo model → estimated position (cascade)
        # Cascading strategy: first try config file target, then Gazebo model, finally fallback estimate
        # Returns (target_xyz, source_string) tuple identifying both location and resolution method
        drop_target_xyz, drop_target_src = _resolve_drop_target_xyz(node)
        
        # ===== STORE TARGET IN BLACKBOARD FOR PHASE TRACKING =====
        # Store resolved drop target XYZ in blackboard for use by base_align and descend_hold phases
        # Convert to float list if valid 3D coordinates, otherwise store None (invalid/not-yet-available)
        # This serves as the definitive drop location for all subsequent target frame calculations
        node.bb["drop_target_xyz"] = (
            [float(drop_target_xyz[0]), float(drop_target_xyz[1]), float(drop_target_xyz[2])]
            if isinstance(drop_target_xyz, (list, tuple)) and len(drop_target_xyz) >= 3
            else None
        )
        # Record resolution method string: "config", "gazebo_model", "estimated", etc.
        # Useful for diagnostics to understand which target source was successful
        node.bb["drop_target_source"] = str(drop_target_src)

        # ===== VALIDATE DROP TARGET & COMPUTE BASE TARGETS =====
        # Proceed only if drop target successfully resolved (3D coordinate available)
        # If not valid, will be handled in fallback below (wait for target)
        if isinstance(drop_target_xyz, (list, tuple)) and len(drop_target_xyz) >= 3:
            # ===== DETERMINE BASE TARGETING MODE =====
            # Two base positioning modes supported:
            # 1. "rigid_pkg": Maintain rigid body formation relative to package (computed from package center position)
            # 2. "fixed_offsets": Use static XY offsets from drop target (simpler, no package tracking needed)
            base_target_mode = str(getattr(man_cfg, "drop_base_target_mode", "rigid_pkg")).strip().lower()
            
            # ===== ESTIMATE CURRENT PACKAGE POSITION =====
            # Compute package XY position from current end-effector poses and grasp geometry
            # Used to determine base formation relative to where package currently is
            # (not relative to where drop target is - accounts for package displacement from movement)
            pkg_ref_xyz = _pkg_xyz_for_alignment(
                node,
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            )
            
            # ===== RIGID PACKAGE FORMATION MODE =====
            # If configured for rigid_pkg mode AND package position estimatable
            # This mode maintains constant base pair geometry while stepping to drop target
            if (
                base_target_mode == "rigid_pkg"
                and isinstance(pkg_ref_xyz, (list, tuple))
                and len(pkg_ref_xyz) >= 2
            ):
                # ===== COMPUTE OFFSET VECTOR FROM PACKAGE TO DROP TARGET =====
                # Vector from current package center to desired drop location
                # Will be applied to current base pair midpoint to compute target bases
                dx = float(drop_target_xyz[0]) - float(pkg_ref_xyz[0])
                dy = float(drop_target_xyz[1]) - float(pkg_ref_xyz[1])
                
                # ===== CAPTURE BASE PAIR SEPARATION GEOMETRY =====
                # Rigid-body constraint: maintain constant XY separation between left/right bases
                # This vector defines the base formation that must be preserved during base motion
                pair_dx = float(right_base[0]) - float(left_base[0])
                pair_dy = float(right_base[1]) - float(left_base[1])
                # Store pair geometry in blackboard for use in replanning if package moves
                node.bb["drop_base_pair_xy"] = [pair_dx, pair_dy]
                
                # ===== COMPUTE TARGET BASE POSITIONS MAINTAINING RIGID FORMATION =====
                # Strategy: compute where base pair midpoint should be, then offset left/right by half-pair
                # Midpoint offset by (dx, dy) from current midpoint towards drop target
                center_x = 0.5 * (float(left_base[0]) + float(right_base[0])) + dx
                center_y = 0.5 * (float(left_base[1]) + float(right_base[1])) + dy
                # Left target: midpoint minus half the pair separation
                # Right target: midpoint plus half the pair separation
                # This maintains exact pair geometry while positioning between drop target and package
                left_target_xy = [center_x - 0.5 * pair_dx, center_y - 0.5 * pair_dy]
                right_target_xy = [center_x + 0.5 * pair_dx, center_y + 0.5 * pair_dy]
                # For logging: show offsets computed from package to target
                target_mode_msg = f"rigid_pkg(dx={dx:.3f},dy={dy:.3f})"
            # ===== FIXED OFFSET MODE (FALLBACK) =====
            # If rigid_pkg mode disabled or package position not estimatable
            # Use static XY offsets from drop target (default: L at -0.6m X and R at +0.6m X, both -0.7m Y)
            else:
                # ===== LEFT BASE TARGET: STATIC OFFSET LEFT OF DROP POINT =====
                # Position left base at (drop_x + left_offset_x, drop_y + offset_y)
                # Default offset: -0.60m X, -0.70m Y (to left-front of drop target)
                left_target_xy = [
                    float(drop_target_xyz[0]) + float(getattr(man_cfg, "drop_base_left_offset_x", -0.60)),
                    float(drop_target_xyz[1]) + float(getattr(man_cfg, "drop_base_offset_y", -0.70)),
                ]
                # ===== RIGHT BASE TARGET: STATIC OFFSET RIGHT OF DROP POINT =====
                # Position right base at (drop_x + right_offset_x, drop_y + offset_y)
                # Default offset: +0.60m X, -0.70m Y (to right-front of drop target)
                right_target_xy = [
                    float(drop_target_xyz[0]) + float(getattr(man_cfg, "drop_base_right_offset_x", 0.60)),
                    float(drop_target_xyz[1]) + float(getattr(man_cfg, "drop_base_offset_y", -0.70)),
                ]
                # ===== STORE BASE PAIR SEPARATION FOR REFERENCE =====
                # Record the resulting base pair geometry (computed from targets)
                # Will be used if phase needs to adapt targets during execution
                node.bb["drop_base_pair_xy"] = [
                    float(right_target_xy[0]) - float(left_target_xy[0]),
                    float(right_target_xy[1]) - float(left_target_xy[1]),
                ]
                target_mode_msg = "fixed_offsets"
            
            # ===== LOG TARGET ACQUISITION SUCCESS =====
            # Confirm drop target location and mode used for base positioning
            # Helps operator verify target source (config/model/estimate) and positioning strategy
            node.get_logger().info(
                bt_fmt(
                    "[Drop] target acquired "
                    f"(src={drop_target_src}, mode={target_mode_msg}, "
                    f"target={np.round(np.asarray(drop_target_xyz), 3).tolist()})"
                )
            )
        # ===== FALLBACK: NO VALID TARGET YET =====
        # If drop target not successfully resolved, cannot proceed with drop motion
        # Wait for valid drop target pose before attempting base movement or descent
        else:
            node._warn_throttled(
                "drop_target_wait",
                bt_fmt("[Drop] target model unavailable, waiting for valid drop target pose"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        # ================================================================
        # STORE BASE TARGETS & INITIALIZE TRAJECTORY PARAMETERS
        # ================================================================
        # ===== BLACKBOARD STORAGE: BASE TARGETS =====
        # Store computed base target positions for use by base_align phase
        # These are the goal positions that TP will move bases toward
        node.bb["drop_left_target_xy"] = left_target_xy
        node.bb["drop_right_target_xy"] = right_target_xy
        
        # ===== BLACKBOARD STORAGE: PACKAGE REFERENCE POSITION =====
        # Store drop target XY (package center should reach this point)
        # Used by descend_hold phase to validate package positioning during descent
        # Set to None if drop target not fully specified (no XY data)
        node.bb["drop_target_pkg_xy"] = [float(drop_target_xyz[0]), float(drop_target_xyz[1])] if isinstance(drop_target_xyz, (list, tuple)) and len(drop_target_xyz) >= 2 else None
        
        # ===== BLACKBOARD INITIALIZATION: ARM GOALS =====
        # Initialize arm goal slots to None (will be set by phase-specific logic)
        # drop_arm_goal_left/right: used in legacy_joint mode for joint-space targets
        # drop_lock_arm_goal_left/right: used in base_align phase for joint locking
        node.bb["drop_arm_goal_left"] = None
        node.bb["drop_arm_goal_right"] = None
        
        # ===== TRAJECTORY PLANNER: BASE MOTION GAINS =====
        # Control gain for base XY positioning in base_align phase
        # Higher values = stiffer control (faster response, potential oscillations)
        # Defaults to drop_base_kp_xy or fallback to transport_retreat_kp_x (inter-phase consistency)
        drop_base_kp_xy = float(getattr(man_cfg, "drop_base_kp_xy", getattr(man_cfg, "transport_retreat_kp_x", 1.0)))
        
        # ===== TRAJECTORY PLANNER: EXECUTION DURATION =====
        # Time budget for base_align phase to reach targets
        # Strategy: allocate ~25% of total descend_and_place_time for base alignment
        # Ranges: min 0.3s (very close target), max 3.0s (distant targets), typically 1-2s
        # Gives TP sufficient time to plan smooth base trajectories
        drop_base_traj_time = float(
            max(
                0.3,  # Minimum: 0.3 second (must allow some planning/execution time)
                getattr(
                    man_cfg,
                    "drop_base_align_traj_time",
                    # Default: 25% of total drop time, bounded 1.0-3.0 seconds
                    # Ensures base moves quickly but arm descent phase has most of the time
                    min(max(descend_and_place_time * 0.25, 1.0), 3.0),
                ),
            )
        )
        # Store trajectory parameters in blackboard for base_align phase runtime
        node.bb["drop_base_align_kp_xy"] = drop_base_kp_xy
        node.bb["drop_base_align_traj_time"] = drop_base_traj_time

        # ================================================================
        # INITIALIZE TRAJECTORY PLANNER: BASE STAGE SETUP
        # ================================================================
        # ===== TP BASE STAGE INITIALIZATION =====
        # Prepare trajectory planner for base-only motion during base_align phase
        # Configures base goals, execution timing, and control parameters
        # arm_active=True if using package hold (TP will also manage arm locking)
        # arm_active=False if not using hold (TP focuses on base movement only)
        if not _init_tp_base_stage(
            node,
            left_base_goal_xy=left_target_xy,      # Left base target position
            right_base_goal_xy=right_target_xy,    # Right base target position
            period_s=drop_base_traj_time,          # Time budget for execution
            kp_xy=drop_base_kp_xy,                 # XY control stiffness gain
            kp_yaw=0.0,                            # YAW disabled (no rotation to targets)
            arm_active=bool(use_pkg_hold),         # Activate arm control if using hold
        ):
            # ===== TP INITIALIZATION FAILURE =====
            # TP rejected base stage initialization (e.g., invalid targets, internal error)
            # Cannot proceed with drop action; abort and return False
            node.get_logger().warn(bt_fmt("[Drop] unable to initialize TP base stage"))
            return False

        # ================================================================
        # CONFIGURE ARM CONTROL: PACKAGE HOLD OR LEGACY MODE
        # ================================================================
        if use_pkg_hold:
            # ===== PACKAGE-CENTRIC HOLD MODE INITIALIZATION =====
            # Set up object-centric hold for descent: maintain package center between grippers
            # Two-phase sequence: 1) base_align (bases move, arms lock), 2) descend_hold (base still, arms descend)
            
            # ===== CAPTURE GRASP OFFSETS IF NEEDED =====
            # If grasp offset geometry not yet captured, measure it now
            # Offsets define vector from end-effector to grasp point (used in hold control)
            # Only captures if not already recorded (left/right both None)
            if node._pkg_hold_offsets.get("left", None) is None or node._pkg_hold_offsets.get("right", None) is None:
                _capture_pkg_grasp_offsets(node, left_arm_jp, right_arm_jp, left_base, right_base)
            
            # ===== ARM JOINT LOCKING SETUP =====
            # During base_align phase: lock arms in current joint configuration to prevent EE motion
            # Protecting package from twisting/rotating while bases move to alignment position
            # Right after package hold is established, joint angles are frozen here
            q_lock_l = np.asarray(left_arm_jp, dtype=np.float32)
            q_lock_r = np.asarray(right_arm_jp, dtype=np.float32)
            # Store lock goals in blackboard for base_align phase
            node.bb["drop_lock_arm_goal_left"] = [float(v) for v in q_lock_l.tolist()]
            node.bb["drop_lock_arm_goal_right"] = [float(v) for v in q_lock_r.tolist()]
            
            # ===== TP ARM JOINT STAGE: INITIALIZE LOCK CONTROL =====
            # Configure trajectory planner to lock arms at current joint positions
            # Execution time: use full descend_and_place_time (arms must maintain hold through all phases)
            # Control gain: transport_lock_arm_kp or fallback to approach_jtc_arm_kp
            # Ensures arms stay fixed in joint space despite external forces from base motion
            _init_tp_arm_joint_stage(
                node,
                left_arm_goal=q_lock_l,                      # Lock to current left arm position
                right_arm_goal=q_lock_r,                     # Lock to current right arm position
                period_s=float(max(descend_and_place_time, 0.2)),  # Full action duration
                kp_arm=float(getattr(man_cfg, "transport_lock_arm_kp", node.approach_jtc_arm_kp)),
            )
            
            # ===== ACTIVATE JTC MULTI-TASK CONTROL =====
            # Enable joint task controller with both base and arm components activated
            # This allows coordinated base+arm motion during base_align phase
            # (arm locked in joint space, base moving to target)
            if node.approach_jtc_task is not None:
                try:
                    node.approach_jtc_task.activate()          # Activate JTC controller
                    node.approach_jtc_task.set_activation("base", True)  # Enable base motion component
                    node.approach_jtc_task.set_activation("arm", True)   # Enable arm holding component
                except Exception:
                    pass  # Silently continue if JTC not available (may be handled elsewhere)
            
            # ===== INITIALIZE PHASE REPLANNING STATE =====
            # Record timestamp of initialization for replanning period checking
            # base_align phase will use this to decide when to replan targets (e.g., if package moves)
            node.bb["drop_base_align_last_replan"] = _ros_now_s(node)
            
            # ===== COMPUTE DESCENT TARGET Z HEIGHT =====
            # Determines how low arms should lower during descend_hold phase
            # Strategy 1: If drop target Z available (from Gazebo model), place above it by release_z_offset
            # Strategy 2: If no drop target Z, compute from initial package height accounting for lifting
            # release_z_offset default: 0.30m above drop surface (gripper open height)
            if isinstance(drop_target_xyz, (list, tuple)) and len(drop_target_xyz) >= 3:
                # Drop target includes Z height: places at that height + offset for gripper open space
                node._pkg_hold_target_z = float(drop_target_xyz[2]) + float(getattr(man_cfg, "drop_release_z_offset", 0.30))
            elif node._pkg_hold_start_z is not None:
                # No drop target Z: use package height from pickup, apply height deltas from collect/drop phases
                # Formula: start_z + collect_lift_delta_z (lifted height) - drop_delta_z (lower amount)
                node._pkg_hold_target_z = (
                    float(node._pkg_hold_start_z)
                    + float(man_cfg.collect_lift_delta_z)
                    - float(man_cfg.drop_delta_z)
                )
            
            # ===== STATE MACHINE: SET INITIAL PHASE =====
            # Begin with base_align: position bases relative to drop target while arms locked
            # After bases converge, will transition to descend_hold for vertical descent
            node.bb["drop_stage"] = "base_align"
            # Record phase start time for phase-specific elapsed time calculation
            node.bb["drop_stage_t0"] = _ros_now_s(node)
        
        # ===== LEGACY FALLBACK: DIRECT JOINT SERVO MODE =====
        # Used if package hold disabled or attach failed
        # Simple direct descent via joint-space arm targets + base targets
        else:
            # ===== COMPUTE ARM DESCENT TARGETS =====
            # Apply preconfigured descent offset delta to current joint positions
            # left_arm_place, right_arm_place: relative joint movements for descent motions
            # Offset applied as: current_joints + place_delta = joint_targets
            q_left_goal = np.asarray(left_arm_jp, dtype=np.float32) + np.asarray(left_arm_place, dtype=np.float32)
            q_right_goal = np.asarray(right_arm_jp, dtype=np.float32) + np.asarray(right_arm_place, dtype=np.float32)
            
            # ===== CLIP JOINT TARGETS TO LIMITS =====
            # Ensure computed goals respect robot joint limits (safety check)
            # Prevents invalid joint commands that would cause TP/controller errors
            q_left_goal, q_right_goal = _clip_descend_pick_joint_goals(node, q_left_goal, q_right_goal)
            
            # ===== STORE ARM TARGETS IN BLACKBOARD =====
            # Save clipped joint goals for legacy_joint phase execution
            node.bb["drop_arm_goal_left"] = [float(v) for v in q_left_goal.tolist()]
            node.bb["drop_arm_goal_right"] = [float(v) for v in q_right_goal.tolist()]
            
            # ===== TP ARM JOINT STAGE: INITIALIZE DESCENT TARGETS =====
            # Configure trajectory planner to move arms to descent positions
            # Execution time: full descend_and_place_time for smooth motion
            # In legacy mode, no hold control needed; TP manages simple joint-to-target motion
            _init_tp_arm_joint_stage(node, q_left_goal, q_right_goal, period_s=float(max(descend_and_place_time, 0.2)))
            
            # ===== STATE MACHINE: SKIP BASE_ALIGN, USE LEGACY_JOINT =====
            # In legacy mode, go directly to legacy_joint phase
            # This phase executes base + arm motion in parallel (no arm locking, simple targets)
            node.bb["drop_stage"] = "legacy_joint"
            # Record phase start time for elapsed time tracking
            node.bb["drop_stage_t0"] = _ros_now_s(node)

        # ================================================================
        # START ACTION TIMER
        # ================================================================
        # ===== ACTION TIMER START =====
        # Mark the beginning of the Drop action for overall timeout tracking
        # Timer used to enforce action-level timeout (prevents indefinite drop cycles)
        # Different from stage_t0 which tracks individual phase durations
        t0 = node.start_action_timer("Drop")

    live_state = _get_live_tp_state(node)
    if live_state is None:
        node._warn_throttled("drop_wait_data", bt_fmt("[Drop] waiting sensor data (joint_states/odom)"), period_s=1.0)
        rclpy.spin_once(node, timeout_sec=0.01)
        return None

    left_arm_jp, right_arm_jp, left_base, right_base = live_state
    stage = str(node.bb.get("drop_stage", "legacy_joint")).strip().lower()
    stage_t0 = float(node.bb.get("drop_stage_t0", _ros_now_s(node)))

    if use_pkg_hold and stage == "base_align":
        # =================================================================
        # BASE_ALIGN PHASE: Position bases relative to drop target
        # =================================================================
        # Execute TP for base positioning + arm locking
        # Continuously replan targets as package moves to track drop location
        
        # ===== TP EXECUTION FOR BASE/ARM MOVEMENT =====
        # Trigger trajectory planner to compute and execute base+arm control commands
        # Base moves to align with drop target while arms maintain package hold grasp
        # Velocity limits: arm motion clamped to hold_arm_cmd_abs_max, base XY to drop_base_cmd_xy_abs_max
        _execute_tp_full_control(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
            arm_clip_abs=float(getattr(man_cfg, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
            base_xy_abs_max=float(getattr(man_cfg, "drop_base_cmd_xy_abs_max", 0.12)),
            base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
        )
        
        # ===== LOGGING: Monitor package hold quality and force proxy =====
        # Log current hold state (left/right grasp offsets, grasp quality metrics)
        # Force proxy logs internal TP proxy forces to validate hold stability during base motion
        _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "drop_base_align")
        _log_force_proxy(node, "drop_base_align", period_s=1.0)

        # ===== RETRIEVE TARGET POSITIONS FROM BLACKBOARD =====
        # Get planned base targets and package reference XY from init phase
        left_target_xy = node.bb.get("drop_left_target_xy", None)
        right_target_xy = node.bb.get("drop_right_target_xy", None)
        drop_target_pkg_xy = node.bb.get("drop_target_pkg_xy", None)
        
        # ===== COMPUTE PACKAGE POSITION ESTIMATE =====
        # Estimates package XYZ position based on current end-effector poses
        # Uses forward kinematics and hold offset geometry to determine grasp point
        pkg_xyz_eval = _pkg_xyz_for_alignment(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
        )
        
        # ===== CALCULATE BASE DISTANCES TO TARGETS =====
        # L_dist: Euclidean distance from left base current pose to target
        # R_dist: Euclidean distance from right base current pose to target
        # Using math.hypot() for stable XY Euclidean distance computation
        l_dist = float("inf")
        r_dist = float("inf")
        if isinstance(left_target_xy, (list, tuple)) and len(left_target_xy) >= 2:
            l_dist = float(math.hypot(float(left_target_xy[0]) - float(left_base[0]), float(left_target_xy[1]) - float(left_base[1])))
        if isinstance(right_target_xy, (list, tuple)) and len(right_target_xy) >= 2:
            r_dist = float(math.hypot(float(right_target_xy[0]) - float(right_base[0]), float(right_target_xy[1]) - float(right_base[1])))
        
        # ===== CALCULATE PACKAGE XY ERROR =====
        # Package position error: distance from estimated package center to drop target center
        # This validates that manipulated object reaches intended placement location
        # If package reference unavailable (pkg_xyz_eval=None), set error to NaN (ignore in convergence check)
        pkg_xy_err = float("nan")
        if (
            isinstance(drop_target_pkg_xy, (list, tuple))
            and len(drop_target_pkg_xy) >= 2
            and isinstance(pkg_xyz_eval, (list, tuple))
            and len(pkg_xyz_eval) >= 2
        ):
            pkg_xy_err = float(
                math.hypot(
                    float(pkg_xyz_eval[0]) - float(drop_target_pkg_xy[0]),
                    float(pkg_xyz_eval[1]) - float(drop_target_pkg_xy[1]),
                )
            )
        # ===== CONVERGENCE CRITERIA: DISTANCE TOLERANCES =====
        # Base tolerance: acceptable distance from bases to their target positions (typically 0.10m)
        base_tol = float(getattr(man_cfg, "drop_base_goal_tol", getattr(man_cfg, "transport_retreat_goal_tol", 0.10)))
        
        # Base convergence: both left AND right bases within tolerance distance
        base_reached = bool(l_dist <= base_tol and r_dist <= base_tol)
        
        # Package convergence: package center within tolerance of drop target (if estimatable)
        pkg_reached = bool(np.isfinite(pkg_xy_err) and pkg_xy_err <= base_tol)
        
        # Overall alignment convergence: bases must reach targets AND (package at target OR package unestimatable)
        # Prevents premature transition to descend when bases haven't arrived but package is close
        # If package position cannot be estimated, allow transition once bases are aligned
        align_reached = bool(base_reached and (pkg_reached or (not np.isfinite(pkg_xy_err))))
        # ===== TRACK PHASE ELAPSED TIME =====
        # Measure seconds since base_align phase started (from stage_t0 recorded at phase transition)
        elapsed_stage = _ros_now_s(node) - stage_t0
        
        # ===== PHASE TIMEOUT CONFIGURATION =====
        # If configured timeout > 0, use it; otherwise no timeout (inf) allows phase to continue indefinitely
        # Prevents bases from getting stuck in unreachable alignment if drop target positioning fails
        timeout_cfg = float(getattr(man_cfg, "drop_base_stage_timeout", 0.0))
        timeout_stage = timeout_cfg if timeout_cfg > 0.0 else float("inf")
        
        # ===== LOG TRACKING STATUS =====
        # Periodic logging (1Hz) of base alignment progress: distances, package error, elapsed time
        # Identifies convergence issues if phase takes longer than expected
        node._info_throttled(
            "drop_base_align_track",
            bt_fmt(
                f"[Drop] base_align target_reached={align_reached} "
                f"L_dist={l_dist:.3f} R_dist={r_dist:.3f} "
                f"pkg_xy_err={pkg_xy_err:.3f} "
                f"elapsed={elapsed_stage:.2f}/{timeout_stage:.2f}s "
                f"target_src={node.bb.get('drop_target_source', 'n/a')}"
            ),
            period_s=1.0,
        )

        # ===== REPLANNING PERIOD: Update targets if package moves =====
        # Minimum time between successive TP replans (typically 0.5-1.5s) to avoid excessive computation
        # Prevents oscillations while allowing adaptation if package/drop target estimates change
        replan_period = float(max(0.5, getattr(man_cfg, "drop_base_replan_period", 1.5)))
        last_replan = float(node.bb.get("drop_base_align_last_replan", 0.0))
        need_replan = bool((_ros_now_s(node) - last_replan) >= replan_period)
        
        # ===== CONVERGENCE DECISION: TRANSITION TO DESCEND_HOLD =====
        if align_reached:
            # ===== STOP BASE MOVEMENT =====
            # Halt all TP-controlled base motion now that alignment converged
            node.stop_all_movement()
            
            # ===== DEACTIVATE BASE+ARM CONTROL IN JTC TASK =====
            # During base_align we ran both base and arm via TP; now switch to arm-only control for descent
            # Deactivate base control to prevent further base motion (already aligned)
            if node.approach_jtc_task is not None:
                try:
                    node.approach_jtc_task.activate()
                    node.approach_jtc_task.set_activation("base", False)  # Disable base task component
                    node.approach_jtc_task.set_activation("arm", False)   # Temporarily disable arm; will re-enable in descend_hold
                except Exception:
                    pass
            
            # ===== CAPTURE END-EFFECTOR ORIENTATION AT ALIGNMENT =====
            # Record (R, P, Y) of both end-effectors at this configuration to prevent unwanted wrist rotations during descent
            # Blocks EE orientation changes that could twist package or break hold configuration
            ee_live = node._get_live_ee_by_side(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            )
            l_ee = ee_live.get("left", None)
            r_ee = ee_live.get("right", None)
            if isinstance(l_ee, np.ndarray) and l_ee.shape[0] >= 6:
                # Extract roll, pitch, yaw (last 3 elements of 6D EE pose vector)
                node._pkg_hold_rpy["left"] = np.asarray(l_ee[3:6], dtype=np.float32).copy()
            if isinstance(r_ee, np.ndarray) and r_ee.shape[0] >= 6:
                # Extract roll, pitch, yaw for right side
                node._pkg_hold_rpy["right"] = np.asarray(r_ee[3:6], dtype=np.float32).copy()
            
            # ===== PHASE TRANSITION TO DESCEND_HOLD =====
            # Mark that descend phase should force a full TP replan to initialize descent trajectory
            node.bb["drop_descend_force_replan"] = True
            # Switch to descend_hold state machine phase
            node.bb["drop_stage"] = "descend_hold"
            # Record phase start time for descend elapsed time calculation
            node.bb["drop_stage_t0"] = _ros_now_s(node)
            
            node.get_logger().info(bt_fmt("[Drop] base aligned, switching to descend_hold"))
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        # ===== PERIODIC REPLANNING: ADAPT TARGETS IF PACKAGE POSITION CHANGES =====
        if need_replan:
            # Check if rigid-body formation mode enabled: maintain base pair geometry relative to drop target
            base_target_mode = str(getattr(man_cfg, "drop_base_target_mode", "rigid_pkg")).strip().lower()
            if (
                base_target_mode == "rigid_pkg"
                and isinstance(drop_target_pkg_xy, (list, tuple))
                and len(drop_target_pkg_xy) >= 2
                and isinstance(pkg_xyz_eval, (list, tuple))
                and len(pkg_xyz_eval) >= 2
            ):
                # ===== COMPUTE OFFSET FROM PACKAGE TO DROP TARGET =====
                # If package moved away from estimated drop location, recalculate base targets
                # to maintain formation geometry relative to actual package position
                dx = float(drop_target_pkg_xy[0]) - float(pkg_xyz_eval[0])
                dy = float(drop_target_pkg_xy[1]) - float(pkg_xyz_eval[1])
                
                # ===== RETRIEVE BASE PAIR GEOMETRY =====
                # Get the rigid separation vector between left and right bases (from init phase)
                # If not available, compute from current base positions
                pair = node.bb.get("drop_base_pair_xy", None)
                if isinstance(pair, (list, tuple)) and len(pair) >= 2:
                    pair_dx = float(pair[0])
                    pair_dy = float(pair[1])
                else:
                    pair_dx = float(right_base[0]) - float(left_base[0])
                    pair_dy = float(right_base[1]) - float(left_base[1])
                    node.bb["drop_base_pair_xy"] = [pair_dx, pair_dy]
                
                # ===== RECOMPUTE BASE TARGET POSITIONS MAINTAINING RIGID PAIR =====
                # Formation center: update from initial drop target accounting for package movement
                center_x = 0.5 * (float(left_base[0]) + float(right_base[0])) + dx
                center_y = 0.5 * (float(left_base[1]) + float(right_base[1])) + dy
                # Left/right targets: maintain symmetry around updated center (half-pair on each side)
                left_target_xy = [center_x - 0.5 * pair_dx, center_y - 0.5 * pair_dy]
                right_target_xy = [center_x + 0.5 * pair_dx, center_y + 0.5 * pair_dy]
                # Update blackboard with new targets for TP initialization
                node.bb["drop_left_target_xy"] = left_target_xy
                node.bb["drop_right_target_xy"] = right_target_xy

            # ===== RE-INITIALIZE TP TRAJECTORY FOR UPDATED TARGETS =====
            # Plan new base trajectory to reach recomputed targets, maintaining arm hold control
            _init_tp_base_stage(
                node,
                left_base_goal_xy=left_target_xy,
                right_base_goal_xy=right_target_xy,
                period_s=float(max(0.3, node.bb.get("drop_base_align_traj_time", 2.5))),
                kp_xy=float(node.bb.get("drop_base_align_kp_xy", getattr(man_cfg, "drop_base_kp_xy", 1.2))),
                kp_yaw=0.0,
                arm_active=True,  # Keep arm control active for package hold during base motion
            )
            # Record replan timestamp to avoid excessive replanning
            node.bb["drop_base_align_last_replan"] = _ros_now_s(node)

        # ===== TIMEOUT HANDLING: SOFT EXTENSION =====
        # If phase exceeds configured timeout but bases haven't converged, log warning and continue
        # Soft timeout allows recovery if bases need extra time due to package movement or difficult terrain
        if np.isfinite(timeout_stage) and elapsed_stage >= timeout_stage:
            node._warn_throttled(
                "drop_base_align_extend",
                bt_fmt(
                    "[Drop] base_align timeout without convergence, continuing tracking "
                    f"(L_dist={l_dist:.3f}, R_dist={r_dist:.3f}, pkg_xy_err={pkg_xy_err:.3f})"
                ),
                period_s=2.0,
            )

        rclpy.spin_once(node, timeout_sec=0.01)
        return None

    # =================================================================
    # DESCEND_HOLD PHASE: Lower package while maintaining object-centric hold
    # =================================================================
    # Multi-threshold convergence strategy:
    # 1. NOMINAL: tight tolerances (< 8s) - high precision placement
    # 2. SOFT: relaxed tolerances (after 8s) - allows some lateral drift during descent
    # 3. FORCE: timeout fallback (12-18s total) - abandon convergence, complete action
    # Enables smooth descent even if perfect alignment difficult to achieve
    
    if use_pkg_hold and stage == "descend_hold":
        # ===== TP ARM CONTROL FOR DESCENT =====
        # Execute trajectory planner for arm-only descent (base now stationary)
        # Controls both arms jointly to lower package while maintaining center-of-mass between end-effectors
        # Arm motion velocity-limited to hold_arm_cmd_abs_max for stable descent speed
        _execute_tp_arm_control(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
            arm_clip_abs=float(getattr(man_cfg, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
        )
        
        # ===== RETRIEVE DROP TARGET LOCATION =====
        # Get target Z for package descent from blackboard (set during init phase)
        drop_target_xyz = node.bb.get("drop_target_xyz", None)
        # Extract XY target for package (may be used for horizontal hold centering)
        drop_xy_target = (
            [float(drop_target_xyz[0]), float(drop_target_xyz[1])]
            if isinstance(drop_target_xyz, (list, tuple)) and len(drop_target_xyz) >= 2
            else None
        )
        
        # ===== FORCE TP REPLAN IF COMING FROM BASE_ALIGN TRANSITION =====
        # Check if we just transitioned from base_align and need to initialize descent trajectory
        # This ensures arms are properly configured for holding during the descent motion
        force_replan = bool(node.bb.pop("drop_descend_force_replan", False))
        
        # ===== REPLANNING: MAINTAIN PACKAGE HOLD TRAJECTORY =====
        # Continuously update hold trajectory as package descends to target Z height
        # Keeps arms coordinated and prevents hold offset drift during long descent
        # force=True on first descent tick ensures fresh trajectory computation from base_align configuration
        if _replan_pkg_hold_tp(
            node,
            left_arm_jp,
            right_arm_jp,
            left_base,
            right_base,
            pkg_xy_target=drop_xy_target,         # XYZ reference for hold geometry
            pkg_z_target=node._pkg_hold_target_z, # Q-offset to lower toward this Z height
            force=force_replan,  # Force first replan from base_align transition
            preserve_jtc_base=False,  # Arms only; base already still from base_align
        ):
            # ===== LOGGING: Monitor hold quality during descent =====
            _log_pkg_hold_quality(node, left_arm_jp, right_arm_jp, left_base, right_base, "drop_descend")
            _log_force_proxy(node, "drop_descend", period_s=1.0)

        # ===== PACKAGE POSITION ESTIMATION =====
        # Retrieve live package position from Gazebo (if available) or estimate from EE geometry
        pkg_xyz_live = _get_live_package_xyz(node)
        # Fallback: compute estimated package position from forward kinematics + hold offsets
        pkg_xyz_ref = _resolve_pkg_reference_xyz(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
        )
        # Use live if available, otherwise estimate from arm kinematics
        pkg_xyz_eval = pkg_xyz_live if pkg_xyz_live is not None else pkg_xyz_ref
        
        # ===== DESCENT TARGET Z HEIGHT =====
        # Target Z for placing: either configured drop height or estimated from initial package height
        z_target = float(node._pkg_hold_target_z) if node._pkg_hold_target_z is not None else (
            float(drop_target_xyz[2]) if isinstance(drop_target_xyz, (list, tuple)) and len(drop_target_xyz) >= 3 else float("nan")
        )
        
        # ===== CALCULATE Z ERROR (VERTICAL PLACEMENT PRECISION) =====
        # Error in Z: absolute difference between current package Z and target descent height
        # Inf if target unavailable (descent cannot proceed)
        z_err = (
            abs(float(pkg_xyz_eval[2]) - z_target)
            if isinstance(pkg_xyz_eval, (list, tuple)) and len(pkg_xyz_eval) >= 3 and np.isfinite(z_target)
            else float("inf")
        )
        
        # ===== CALCULATE XY ERROR (HORIZONTAL PLACEMENT PRECISION) =====
        # Error in XY: Euclidean distance from package center to drop target center
        # NaN if cannot estimate package XY position (e.g., hold lost)
        xy_err = float("nan")
        if (
            isinstance(pkg_xyz_eval, (list, tuple))
            and len(pkg_xyz_eval) >= 2
            and isinstance(drop_target_xyz, (list, tuple))
            and len(drop_target_xyz) >= 2
        ):
            xy_err = float(
                math.hypot(
                    float(pkg_xyz_eval[0]) - float(drop_target_xyz[0]),
                    float(pkg_xyz_eval[1]) - float(drop_target_xyz[1]),
                )
            )
        # ===== TOLERANCE THRESHOLDS FOR MULTI-LEVEL CONVERGENCE =====
        # NOMINAL TOLERANCES: tight acceptance criteria for precise placement
        z_tol = float(getattr(man_cfg, "hold_z_tol", 0.04))  # Tight Z tolerance: ±4cm vertical precision
        xy_tol = float(getattr(man_cfg, "transport_destination_goal_tol", 0.10))  # Tight XY tolerance: ±10cm horizontal
        
        # SOFT TOLERANCES: relaxed criteria applied after 8+ seconds if nominal not reached
        # Allows descent to complete even if perfect alignment not achievable
        xy_tol_soft = float(max(xy_tol, getattr(man_cfg, "drop_descend_xy_tol_soft", 0.40)))  # Relaxed XY: ±40cm
        z_tol_soft = float(
            max(
                z_tol,
                getattr(man_cfg, "drop_descend_z_tol_soft", max(z_tol + 0.02, 0.07)),
            )
        )  # Relaxed Z: ±7cm
        
        # SOFT THRESHOLD TIME: wait this long before applying relaxed tolerances
        soft_after_s = float(max(0.0, getattr(man_cfg, "drop_descend_soft_after_s", 8.0)))  # Typically 8 seconds
        
        # FORCE TIMEOUT: hard limit to abandon convergence and complete descent
        # Fallback safety mechanism (12-18s typically) to prevent indefinite waiting
        timeout_stage = float(max(descend_and_place_time, 0.2) + 2.0)  # Baseline timeout
        force_after_s = float(
            max(
                timeout_stage + 6.0,
                getattr(man_cfg, "drop_descend_force_after_s", timeout_stage + 12.0),
            )
        )  # Hard force timeout ~12-18s
        
        # ===== TRACK ELAPSED TIME IN DESCEND PHASE =====
        elapsed_stage = _ros_now_s(node) - stage_t0
        
        # ===== CONVERGENCE THRESHOLD 1: NOMINAL (TIGHT) PRECISION =====
        # Achieved when Z within tight tolerance AND XY within tolerance (or unmeasurable)
        # Indicates successful placement with high precision
        reached_nominal = bool(z_err <= z_tol and ((not np.isfinite(xy_err)) or (xy_err <= xy_tol)))
        
        # ===== CONVERGENCE THRESHOLD 2: SOFT (RELAXED) PRECISION =====
        # Applied after soft_after_s seconds: allows relaxed tolerances if descent taking extra time
        # Handles cases where perfect centering difficult (e.g., soft gripper deformation, uneven surfaces)
        reached_soft = bool(
            elapsed_stage >= soft_after_s  # Only check after minimum descent time
            and z_err <= z_tol_soft         # Z within relaxed tolerance
            and np.isfinite(xy_err)         # Package position must be estimatable
            and xy_err <= xy_tol_soft       # XY within relaxed tolerance
        )
        
        # ===== CONVERGENCE THRESHOLD 3: FORCE (TIMEOUT FALLBACK) =====
        # Hard limit: if exceeds force_after_s, abandon convergence and complete
        # Prevents indefinite descent if gripper jam, package slipping, or sensor failure
        reached_force = bool(
            elapsed_stage >= force_after_s            # Must exceed hard timeout
            and np.isfinite(xy_err)                   # Package still estimatable
            and xy_err <= xy_tol_soft                 # At least within soft XY tolerance (not wildly off-target)
        )
        
        # ===== OVERALL CONVERGENCE STATE =====
        # Action succeeds if ANY threshold met (nominal OR soft OR force)
        # Prioritizes precision but allows fallback to soft/force if needed
        reached = bool(reached_nominal or reached_soft or reached_force)
        
        # ===== OPERATIONAL LOGGING =====
        node._info_throttled(
            "drop_descend_track",
            bt_fmt(
                f"[Drop] descend_hold reached={reached} "
                f"z_err={z_err:.3f} (tol={z_tol:.3f}) "
                f"xy_err={xy_err:.3f} (tol={xy_tol:.3f}) "
                f"elapsed={elapsed_stage:.2f}/{timeout_stage:.2f}s"
            ),
            period_s=1.0,
        )
        if reached_soft and (not reached_nominal):
            # ===== SOFT CONVERGENCE LOG =====
            # Descent reached soft tolerance but not nominal tight tolerance
            # Indicates successful placement but with relaxed precision (expected after 8+ seconds)
            node._warn_throttled(
                "drop_descend_soft_reached",
                bt_fmt(
                    "[Drop] descend_hold soft-converged "
                    f"(z_err={z_err:.3f}, z_tol_soft={z_tol_soft:.3f}, "
                    f"xy_err={xy_err:.3f}, xy_tol_soft={xy_tol_soft:.3f})"
                ),
                period_s=2.0,
            )
        if reached_force and (not reached_nominal) and (not reached_soft):
            # ===== FORCE TIMEOUT LOG =====
            # Hard timeout exceeded but XY still within soft tolerance
            # Abandoning convergence waiting; descending position acceptable but imprecise
            node._warn_throttled(
                "drop_descend_force_reached",
                bt_fmt(
                    "[Drop] descend_hold force-timeout reached, continuing "
                    f"(z_err={z_err:.3f}, z_tol={z_tol:.3f}, z_tol_soft={z_tol_soft:.3f}, "
                    f"xy_err={xy_err:.3f}, elapsed={elapsed_stage:.2f}/{force_after_s:.2f}s)"
                ),
                period_s=2.0,
            )
        
        # ===== CONVERGENCE CHECK & COMPLETION =====
        if reached:
            # ===== HALT ALL MOVEMENT =====
            # Stop TP-controlled arm descent since convergence achieved
            node.stop_all_movement()
            
            # ===== CLEAR ACTION TIMER =====
            # Remove Drop action timer (marks action as finished in runtime tracking)
            node.clear_action_timer("Drop")
            
            # ===== CLEANUP BLACKBOARD STATE =====
            # Remove all drop-specific temporary state variables for next action
            node.bb.pop("drop_left_target_xy", None)
            node.bb.pop("drop_right_target_xy", None)
            node.bb.pop("drop_base_pair_xy", None)
            node.bb.pop("drop_arm_goal_left", None)
            node.bb.pop("drop_arm_goal_right", None)
            node.bb.pop("drop_stage", None)
            node.bb.pop("drop_stage_t0", None)
            
            # ===== PHASE PAUSE RESET =====
            # Clear any pause state for Drop action (allows next Drop to run if needed)
            _phase_pause_reset(node, "Drop")
            
            # ===== COMPLETION =====
            node.get_logger().info(bt_fmt("[Drop] completed"))
            return True

        if elapsed_stage >= timeout_stage:
            # ===== TIMEOUT HANDLING: SOFT EXTENSION =====
            # If reached baseline timeout without convergence, allow continued descent
            # Phase will continue beyond timeout until reaching force timeout or convergence
            # This soft extend allows for slightly longer descents when needed
            node._warn_throttled(
                "drop_descend_extend",
                bt_fmt(
                    "[Drop] descend_hold timeout without convergence, extending stage "
                    f"(z_err={z_err:.3f}, xy_err={xy_err:.3f})"
                ),
                period_s=2.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

        rclpy.spin_once(node, timeout_sec=0.01)
        return None

    # =================================================================
    # LEGACY FALLBACK: Non-object-centric hold (direct joint servo descent)
    # =================================================================
    # Used when package_centric hold disabled or attach failed
    # Descends via simple joint-space targets without center-of-mass tracking
    
    # ===== TP FULL CONTROL EXECUTION =====
    # Execute trajectory planner for base + arm movement (legacy mode)
    # Both bases move + both arms move toward configured descent targets
    # ===== TP FULL CONTROL EXECUTION =====
    # Execute trajectory planner for base + arm movement (legacy mode)
    # Both bases move + both arms move toward configured descent targets
    _execute_tp_full_control(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
        arm_clip_abs=float(getattr(man_cfg, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
        base_xy_abs_max=float(getattr(man_cfg, "drop_base_cmd_xy_abs_max", 0.12)),
        base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
    )

    # ===== RETRIEVE BASE & ARM TARGETS =====
    # Get planned base target XY positions and arm joint goals
    left_target_xy = node.bb.get("drop_left_target_xy", None)
    right_target_xy = node.bb.get("drop_right_target_xy", None)
    
    # ===== CALCULATE BASE DISTANCES =====
    # Compute Euclidean distance from each base to its target XY position
    l_dist = float("inf")
    r_dist = float("inf")
    if isinstance(left_target_xy, (list, tuple)) and len(left_target_xy) >= 2:
        l_dist = float(math.hypot(float(left_target_xy[0]) - float(left_base[0]), float(left_target_xy[1]) - float(left_base[1])))
    if isinstance(right_target_xy, (list, tuple)) and len(right_target_xy) >= 2:
        r_dist = float(math.hypot(float(right_target_xy[0]) - float(right_base[0]), float(right_target_xy[1]) - float(right_base[1])))
    
    # ===== BASE CONVERGENCE CHECK =====
    # Both bases must be within tolerance distance of targets
    tol = float(getattr(man_cfg, "drop_base_goal_tol", getattr(man_cfg, "transport_retreat_goal_tol", 0.10)))
    base_reached = bool(l_dist <= tol and r_dist <= tol)

    # ===== ARM CONVERGENCE CHECK (Legacy mode) =====
    # For legacy mode (no package hold), verify arms reached joint space targets
    # Compare current joint positions to target joint positions
    arm_reached = True
    if not use_pkg_hold:
        # Retrieve joint-space targets for both arms
        q_left_goal = node.bb.get("drop_arm_goal_left", None)
        q_right_goal = node.bb.get("drop_arm_goal_right", None)
        # Calculate max joint error for each arm (worst joint)
        l_err = _max_joint_error(node, left_arm_jp, q_left_goal)
        r_err = _max_joint_error(node, right_arm_jp, q_right_goal)
        # Both arm max errors must be within joint tolerance threshold
        arm_reached = bool(
            np.isfinite(l_err)
            and np.isfinite(r_err)
            and l_err <= float(node.approach_arm_joint_tol)
            and r_err <= float(node.approach_arm_joint_tol)
        )

    # ===== OVERALL CONVERGENCE & COMPLETION =====
    # Calculate total action elapsed time
    elapsed = _ros_now_s(node) - float(t0)
    # Overall timeout: descend motion time + 2 second margin
    timeout = float(max(descend_and_place_time, 0.2) + 2.0)
    # Action succeeds when both bases AND arms reach targets
    reached = bool(base_reached and arm_reached)
    
    # ===== TRACKING LOG =====
    # Monitor base distance and arm errors during legacy descent
    node._info_throttled(
        "drop_track",
        bt_fmt(
            f"[Drop] tracking base_reached={base_reached} arm_reached={arm_reached} "
            f"L_dist={l_dist:.3f} R_dist={r_dist:.3f} elapsed={elapsed:.2f}/{timeout:.2f}s"
        ),
        period_s=1.0,
    )

    # ===== FINAL CONVERGENCE CHECK & COMPLETION =====
    if reached or elapsed >= timeout:
        # ===== HALT ALL MOVEMENT =====
        # Stop all TP-controlled base and arm motion
        node.stop_all_movement()
        
        # ===== CLEAR ACTION TIMER =====
        # Remove Drop action timer (marks action as finished)
        node.clear_action_timer("Drop")
        
        # ===== BLACKBOARD CLEANUP: Remove ALL drop-phase state =====
        # Clear all temporary blackboard variables to prevent state pollution for next action
        node.bb.pop("drop_left_target_xy", None)         # Left base target position
        node.bb.pop("drop_right_target_xy", None)        # Right base target position
        node.bb.pop("drop_base_pair_xy", None)           # Base pair geometry (rigid formation)
        node.bb.pop("drop_arm_goal_left", None)          # Left arm joint-space goal
        node.bb.pop("drop_arm_goal_right", None)         # Right arm joint-space goal
        node.bb.pop("drop_lock_arm_goal_left", None)     # Left arm joint lock goal (base_align phase)
        node.bb.pop("drop_lock_arm_goal_right", None)    # Right arm joint lock goal (base_align phase)
        node.bb.pop("drop_base_align_last_replan", None) # Timestamp of last base target replan
        node.bb.pop("drop_stage", None)                  # Current state machine phase (base_align/descend_hold/legacy)
        node.bb.pop("drop_stage_t0", None)               # Phase start time for elapsed time calculation
        
        # ===== PAUSE STATE RESET =====
        # Clear pause state for Drop (allows next Drop to run if paused)
        _phase_pause_reset(node, "Drop")
        
        # ===== TIMEOUT HANDLING & COMPLETION LOGGING =====
        # If timeout reached but convergence not achieved, log warning
        if elapsed >= timeout and (not reached):
            node.get_logger().warn(bt_fmt("[Drop] timeout reached, continuing"))
        
        # ===== COMPLETION =====
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
    Sequenza TP:
    1) EE open (cartesiano) per aprire palette,
    2) detach + ripristino gravita',
    3) retreat+home con JTC base+arm.
    State-machine runtime (`node.bb['release_phase']`):
    - open_detach -> retreat_home -> done
    """
    node = _require_node()
    phase_cfg = node.cfg.phases
    release_time = float(phase_cfg.release_time)
    man_cfg = node.cfg.manipulation
    release_retreat_time = float(max(getattr(man_cfg, "release_retreat_time", 2.5), 0.2))
    release_retreat_arm_clip = float(
        max(1e-4, getattr(man_cfg, "release_retreat_arm_cmd_abs_max", getattr(man_cfg, "pre_transport_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)))
    )

    # =================================================================
    # PHASE STATE TRACKING: Manage which release sub-phase we're in
    # =================================================================
    # Blackboard key to store current phase (open_detach or retreat_home)
    phase_key = "release_phase"
    # Get stored phase, default to open_detach if not yet set
    # Normalize to lowercase for safe string comparison
    phase = str(node.bb.get(phase_key, "open_detach")).strip().lower()
    # Sanitize old phase name "release" -> map to proper "open_detach"
    if phase == "release":
        phase = "open_detach"
        node.bb[phase_key] = phase

    # =================================================================
    # PHASE 1: OPEN_DETACH (EE trajectory to open + detach RPC)
    # =================================================================
    # Open gripper fingers and execute detach link-attacher RPC
    # Multi-threshold convergence: nominal (tight) then force timeout (12s)
    if phase == "open_detach":
        # Check if this is first-run (t0 == None) or continuing from previous tick
        t0 = node.get_action_timer("Release")
        if t0 is None:
            # ================================================================
            # FIRST-RUN INIT: Set up open trajectory target and TP configuration
            # ================================================================
            # Check pause state: don't proceed if system is paused
            if not _phase_pause_gate(node, "Release"):
                return None  # RUNNING - on pause, retry next tick
            
            # Get current sensor state (joint positions, base poses)
            live_state = _get_live_tp_state(node)
            if live_state is None:
                # No sensor data yet - TP needs live robot state to plan trajectories
                node._warn_throttled("release_wait_data_init", bt_fmt("[Release] waiting sensor data before init"), period_s=1.0)
                # Spin once to check for new messages from sensors
                rclpy.spin_once(node, timeout_sec=0.01)
                return None  # RUNNING - wait for sensor data
            
            # Unpack sensor state: joint positions for arms, XY poses for bases
            left_arm_jp, right_arm_jp, left_base, right_base = live_state
            
            # Get current end-effector poses in Cartesian space (X,Y,Z, roll,pitch,yaw)
            # Used as trajectory start points for open motion
            ee_live = node._get_live_ee_by_side(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            )
            # Extract left/right EE poses from result dictionary
            left_ee = ee_live.get("left", None)
            right_ee = ee_live.get("right", None)
            # If either EE pose is unavailable, cannot plan EE trajectory
            if left_ee is None or right_ee is None:
                node._warn_throttled("release_no_ee_init", bt_fmt("[Release] live EE unavailable"), period_s=1.0)
                rclpy.spin_once(node, timeout_sec=0.01)
                return None  # RUNNING - wait for EE data

            # ================================================================
            # OPEN POSITION TARGET: Compute EE goal positions with open fingers
            # ================================================================
            # Use drop target as reference for open gesture offset
            release_ref_xyz = node.bb.get("drop_target_xyz", None)
            # If drop_target not cached, try to resolve from model or package pose
            # Cascade: Try drop_target config -> live package Gazebo pose -> estimated from EE
            if not (isinstance(release_ref_xyz, (list, tuple)) and len(release_ref_xyz) >= 3):
                release_ref_xyz, _ = _resolve_drop_target_xyz(node)
            # Second fallback: get package world pose directly from Gazebo via ROS service
            if release_ref_xyz is None:
                release_ref_xyz = _get_live_package_xyz(node)
            # Third fallback: estimate from EE grasp offset (less reliable, may have bias)
            if release_ref_xyz is None:
                release_ref_xyz = _resolve_pkg_reference_xyz(node, left_arm_jp, right_arm_jp, left_base, right_base)
            # All fallbacks exhausted: cannot proceed without package reference
            if release_ref_xyz is None:
                node._warn_throttled("release_no_pkg_init", bt_fmt("[Release] package pose unavailable"), period_s=1.0)
                rclpy.spin_once(node, timeout_sec=0.01)
                return None  # RUNNING - wait for package pose data

            # ================================================================
            # GRIPPER ORIENTATION CALCULATION: Closed vs open RPY angles
            # ================================================================
            # Get default/nominal orientation for gripper closed position
            # (This is the orientation WITH gripper fingers together)
            left_rpy_closed = _default_rpy_for_side(node, "left", left_ee)
            right_rpy_closed = _default_rpy_for_side(node, "right", right_ee)
            axis_idx = _pick_open_axis_idx(getattr(man_cfg, "pick_open_axis", "pitch"))
            # Copy closed pose and apply rotation on the open axis (typically pitch)
            # This creates "open" orientation by rotating around specified axis
            left_rpy_open = np.asarray(left_rpy_closed, dtype=np.float32).copy()
            right_rpy_open = np.asarray(right_rpy_closed, dtype=np.float32).copy()
            # Add open_angle to the selected axis (e.g., pitch += pi/2 for wide open)
            left_rpy_open[axis_idx] = float(left_rpy_open[axis_idx]) + float(getattr(man_cfg, "pick_open_angle_rad_left", math.pi / 2.0))
            right_rpy_open[axis_idx] = float(right_rpy_open[axis_idx]) + float(getattr(man_cfg, "pick_open_angle_rad_right", math.pi / 2.0))

            # ================================================================
            # OPEN GOAL COMPUTATION: Compute 6D EE goal (XYZ + RPY) for open gesture
            # ================================================================
            # Create 6D pose vectors: [x, y, z, roll, pitch, yaw]
            left_goal = np.zeros((6,), dtype=np.float32)
            right_goal = np.zeros((6,), dtype=np.float32)
            
            # LEFT EE GOAL: Package center + configured offsets (for left gripper)
            # Offsets are in world frame relative to package drop target
            left_goal[0] = float(release_ref_xyz[0]) + float(getattr(man_cfg, "release_open_left_offset_x", -0.37))  # X: offset left
            left_goal[1] = float(release_ref_xyz[1]) + float(getattr(man_cfg, "release_open_offset_y", -0.10))       # Y: behind package
            left_goal[2] = float(release_ref_xyz[2]) + float(getattr(man_cfg, "release_open_offset_z", 0.30))       # Z: above package
            left_goal[3:6] = left_rpy_open  # Orientation: gripper open
            
            # RIGHT EE GOAL: Mirror of left (symmetric grasping)
            right_goal[0] = float(release_ref_xyz[0]) + float(getattr(man_cfg, "release_open_right_offset_x", 0.37))  # X: offset right
            right_goal[1] = float(release_ref_xyz[1]) + float(getattr(man_cfg, "release_open_offset_y", -0.10))        # Y: same as left
            right_goal[2] = float(release_ref_xyz[2]) + float(getattr(man_cfg, "release_open_offset_z", 0.30))        # Z: same as left
            right_goal[3:6] = right_rpy_open  # Orientation: gripper open

            # ================================================================
            # TP TRAJECTORY INIT: Compute continuous path from current -> open goal
            # ================================================================
            # Invoke trajectory planner to compute arm command trajectory
            # traj_time: duration to reach goal (approximately release_time seconds)
            if not _set_tp_ee_traj(
                node,
                left_ee_now=left_ee,           # Current EE pose
                right_ee_now=right_ee,         # Current EE pose
                left_ee_goal=left_goal,        # Target EE pose with open fingers
                right_ee_goal=right_goal,      # Target EE pose with open fingers
                traj_time=float(max(release_time, 0.2)),  # Duration for trajectory
            ):
                # TP failed to compute trajectory: cannot proceed
                node.get_logger().warn(bt_fmt("[Release] unable to set TP EE trajectory for release-open"))
                return False

            node.bb["release_open_goal_left"] = [float(v) for v in left_goal.tolist()]
            node.bb["release_open_goal_right"] = [float(v) for v in right_goal.tolist()]
            node.bb["release_ref_xyz"] = [float(v) for v in np.asarray(release_ref_xyz[:3], dtype=np.float32).tolist()]
            node.get_logger().info(bt_fmt(f"[Release] start TP open+detach ({release_time}s)"))
            t0 = node.start_action_timer("Release")

        live_state = _get_live_tp_state(node)
        if live_state is None:
            node._warn_throttled("release_wait_data", bt_fmt("[Release] waiting sensor data"), period_s=1.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        # ================================================================
        # TRAJECTORY EXECUTION: Send TP-computed commands to arm controller
        # ================================================================
        # Execute TP arm control: reads TP output (arm velocities) and publishes
        # Move current joint positions toward goal, respecting max velocity limits
        _execute_tp_arm_control(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
            arm_clip_abs=float(getattr(man_cfg, "pick_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
        )

        # ================================================================
        # CONVERGENCE CHECK: Verify EE has reached open goal pose
        # ================================================================
        # Get current EE poses to check distance from goal
        ee_live = node._get_live_ee_by_side(
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
        )
        # Load goal poses that were computed during init
        l_goal = node.bb.get("release_open_goal_left", None)
        r_goal = node.bb.get("release_open_goal_right", None)
        
        # Get convergence tolerances: position (meters) and orientation (radians)
        open_pos_tol = float(getattr(man_cfg, "release_open_pos_tol", getattr(man_cfg, "pick_pos_tol", 0.03)))
        open_ori_tol = float(getattr(man_cfg, "release_open_ori_tol", getattr(man_cfg, "pick_ori_tol", 0.20)))
        
        # Force-timeout: if elapsed > force_after_s, accept convergence even if not perfect
        force_after_s = float(max(0.5, getattr(man_cfg, "release_open_force_after_s", 12.0)))
        
        # Determine which grippers must converge based on attach_mode
        # (single_left/single_right/dual affects which EE needs to reach goal)
        attach_mode = str(getattr(man_cfg, "attach_mode", "single_left")).strip().lower()
        need_left = attach_mode in ("single_left", "dual")    # Left gripper must converge?
        need_right = attach_mode in ("single_right", "dual")  # Right gripper must converge?
        
        # Check if each EE reached target: compute position & orientation errors
        # _ee_goal_reached returns (reached_bool, position_error, orientation_error)
        l_ok, l_pos, l_ori = _ee_goal_reached(
            node, ee_live.get("left", None), l_goal,
            pos_tol=open_pos_tol,
            ori_tol=open_ori_tol,
        )
        r_ok, r_pos, r_ori = _ee_goal_reached(
            node, ee_live.get("right", None), r_goal,
            pos_tol=open_pos_tol,
            ori_tol=open_ori_tol,
        )
        
        # Track elapsed time since phase started
        elapsed = _ros_now_s(node) - float(t0)
        timeout = float(max(release_time, 0.2) + 0.8)
        
        # Nominal convergence: left (if needed) AND right (if needed) reached goals
        nominal_reached = bool((l_ok or (not need_left)) and (r_ok or (not need_right)))
        # Force convergence: timeout exceeded (accept even if not perfect)
        force_reached = bool(elapsed >= force_after_s)
        # Overall convergence: nominal OR force
        open_reached = bool(nominal_reached or force_reached)
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

        if not open_reached:
            node._warn_throttled(
                "release_open_extend",
                bt_fmt(
                    "[Release] open stage timeout without convergence, extending stage "
                    f"(L_pos={l_pos:.3f}, R_pos={r_pos:.3f})"
                ),
                period_s=2.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        if force_reached and (not nominal_reached):
            node._warn_throttled(
                "release_open_force_reached",
                bt_fmt(
                    "[Release] open stage force timeout reached, proceeding to detach "
                    f"(elapsed={elapsed:.2f}s)"
                ),
                period_s=2.0,
            )

        node.stop_all_movement()
        # ================================================================
        # DETACH GRIPPER + PACKAGE: Unbind rigid connection in Gazebo
        # ================================================================
        # Package was held via Gazebo LinkAttacher (virtual rigid joint between
        # grippers + package body). Call detach RPC to break this connection.
        # After detach, package becomes free object in simulation.
        _detach_package_from_arms(node)
        # Mark package as no longer attached: next phase (catch/place) will handle
        # free object behavior (gravity, contact forces, etc.)
        node.bb["package_attached"] = False
        
        # ================================================================
        # RESTORE GRAVITY: Re-enable gravity for free object
        # ================================================================
        # During grasp + transport, gravity was disabled on package to ensure
        # held position. Now re-enable gravity so object falls/settles naturally.
        if node.bb.pop("package_gravity_disabled", False):
            # Call Gazebo service to enable gravity on package body
            set_package_gravity(node, True)
            # Il body ODE potrebbe essere in stato disabled (sleep) dopo
            # essere rimasto fermo con gravità disabilitata.  SetEntityState
            # forza la riattivazione del body in Gazebo/ODE.
            # (ODE simulator puts static bodies to sleep; we force wake-up
            # so gravity takes effect immediately in next physics step)
            wake_package_body(node)
        # Clear hold control runtime data: no longer maintaining package position
        _reset_pkg_hold_runtime(node)

        node.clear_action_timer("Release")
        node.bb.pop("release_open_goal_left", None)
        node.bb.pop("release_open_goal_right", None)
        node.bb[phase_key] = "retreat_home"
        node.get_logger().info(bt_fmt("[Release] detach done, starting TP retreat+home"))
        return None  # RUNNING - next iteration enters retreat_home

    # =================================================================
    # PHASE 2: RETREAT_HOME (concurrent base retreat + arm return to home)
    # =================================================================
    # Move bases away from delivery site and return arms to home configuration
    # Deactivate EE task before this phase to avoid controller conflicts
    # ----------------------------------------------------------------------
    retreat_t0 = node.get_action_timer("ReleaseRetreat")
    if retreat_t0 is None:
        # ================================================================
        # FIRST-RUN INIT: Switch controller mode and compute retreat targets
        # ================================================================
        # Passaggio da fase EE (open) a fase JTC (retreat+home):
        # evita conflitti tenendo attivo solo il controller JTC.
        # Disattivate EE task (gripper open/close commands) because now we're
        # moving in joint-space (JTC mode) to return arms to home + bases to safe distance
        if node.ee_task is not None:
            # First, clear any pending trajectory plan in EE controller
            try:
                if hasattr(node.ee_task, "set_trajectory_plan"):
                    node.ee_task.set_trajectory_plan(None)
            except Exception:
                pass
            # Then deactivate EE task entirely (no more EE-space commands)
            try:
                if hasattr(node.ee_task, "deactivate"):
                    node.ee_task.deactivate()
            except Exception:
                pass

        # ================================================================
        # GET CURRENT STATE: Arm joint positions and base poses for retreat planning
        # ================================================================
        # Fetch current joint positions + base poses from sensors/TF
        # (needed to compute targets: home pose for arms, retreat offset for bases)
        live_state = _get_live_tp_state(node)
        if live_state is None:
            # Still waiting for sensor data (joint encoders, TF not ready yet)
            node._warn_throttled("release_retreat_wait_data_init", bt_fmt("[Release] waiting sensor data before retreat init"), period_s=1.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            return None  # RUNNING - retry next tick when sensor data available
        # Unpack live state: [left_arm_jp, right_arm_jp, left_base_pose, right_base_pose]
        left_arm_jp, right_arm_jp, left_base, right_base = live_state
        
        # ================================================================
        # HOME GOAL COMPUTATION: Target joint positions for arm home state
        # ================================================================
        # Query node's home goal configuration for left arm
        # (returns canonical home config + flags if found/fallback)
        left_home, _ = node._resolve_side_home_joint_goal("left", left_arm_jp)
        # Query home goal for right arm
        right_home, _ = node._resolve_side_home_joint_goal("right", right_arm_jp)
        
        # ================================================================
        # RETREAT TARGET COMPUTATION: 2D XY positions for base retreat
        # ================================================================
        # Left base retreat: move backward by Y offset (config param "release_retreat_offset_y")
        # and apply left-specific X offset ("release_retreat_left_offset_x", typically 0)
        left_target_xy = [
            float(left_base[0]) + float(getattr(man_cfg, "release_retreat_left_offset_x", 0.0)),
            float(left_base[1]) + float(getattr(man_cfg, "release_retreat_offset_y", -0.20)),
        ]
        # Right base retreat: same Y offset, but with right-specific X offset for asymmetric retreat
        right_target_xy = [
            float(right_base[0]) + float(getattr(man_cfg, "release_retreat_right_offset_x", 0.0)),
            float(right_base[1]) + float(getattr(man_cfg, "release_retreat_offset_y", -0.20)),
        ]
        # Store retreat targets in blackboard for tracking during execution loop
        node.bb["release_retreat_left_target_xy"] = left_target_xy
        node.bb["release_retreat_right_target_xy"] = right_target_xy
        
        # ================================================================
        # STORE HOME + RETREAT GOALS: Save targets for execution tracking loop
        # ================================================================
        # Convert home joint arrays to lists + store in blackboard for convergence checking
        # (will compare current joint angles to these targets each iteration)
        node.bb["release_home_goal_left"] = [float(v) for v in np.asarray(left_home, dtype=np.float32).tolist()]
        node.bb["release_home_goal_right"] = [float(v) for v in np.asarray(right_home, dtype=np.float32).tolist()]
        
        # ================================================================
        # TP INITIALIZATION: Set up joint-space trajectories for arm + base
        # ================================================================
        # Initialize TP trajectory planning for arms to reach home configuration
        # (returns True if plan succeeded, False if collision/unreachable)
        ok_arm = _init_tp_arm_joint_stage(
            node,
            left_home,  # Target joint config for left arm
            right_home,  # Target joint config for right arm
            period_s=release_retreat_time,  # Time budget for trajectory execution
        )
        
        # Initialize TP trajectory planning for bases to reach retreat position
        # (2D XY targets computed above, maintains zero yaw change "kp_yaw=0.0")
        ok_base = _init_tp_base_stage(
            node,
            left_base_goal_xy=left_target_xy,
            right_base_goal_xy=right_target_xy,
            period_s=release_retreat_time,
            kp_xy=float(getattr(man_cfg, "transport_retreat_kp_x", 1.0)),  # PD gain for XY control
            kp_yaw=0.0,  # Don't rotate bases during retreat
            arm_active=bool(ok_arm),  # Coordinate with arm trajectory if arm plan succeeded
        )
        
        # ================================================================
        # FORCE BASE+ARM ACTIVATION: Ensure both move together in same JTC stage
        # ================================================================
        # Release fix: assicurati che base+arm restino attivi insieme nello stesso JTC stage.
        # If both arm + base planning succeeded, activate them BOTH in the JTC controller
        # (prevents scenario where one deactivates early and causes jitter/collisions)
        if bool(ok_base and ok_arm) and (node.approach_jtc_task is not None):
            try:
                # Activate the JTC task itself
                node.approach_jtc_task.activate()
                # Enable base DOF computation (X, Y, theta) in this JTC stage
                node.approach_jtc_task.set_activation("base", True)
                # Enable arm DOF computation (joint positions) in this JTC stage
                node.approach_jtc_task.set_activation("arm", True)
            except Exception as exc:
                # If activation fails, log warning but continue (may only affect one base/arm)
                node._warn_throttled(
                    "release_retreat_activation_fix",
                    bt_fmt(f"[Release] unable to enforce base+arm activation: {exc}"),
                    period_s=2.0,
                )
        # Start timer to track elapsed time during retreat execution
        retreat_t0 = node.start_action_timer("ReleaseRetreat")

    # ================================================================
    # EXECUTION LOOP: Run TP control until bases + arms reach retreat/home targets
    # ================================================================
    # Get current sensor state: joint angles, base positions
    live_state = _get_live_tp_state(node)
    if live_state is None:
        # Sensor data not ready yet (waiting for joint encoders, TF updates)
        node._warn_throttled("release_retreat_wait_data", bt_fmt("[Release] waiting sensor data in retreat"), period_s=1.0)
        rclpy.spin_once(node, timeout_sec=0.01)
        return None  # RUNNING - retry next tick
    left_arm_jp, right_arm_jp, left_base, right_base = live_state
    
    # ================================================================
    # TP CONTROLLER EXECUTION: Send arm + base commands to move toward goals
    # ================================================================
    # Execute TP trajectory planning output: reads joint/base velocities from TP
    # and publishes them as controller commands (respecting velocity limits)
    _execute_tp_full_control(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
        arm_clip_abs=release_retreat_arm_clip,  # Max joint velocity magnitude
        base_xy_abs_max=float(getattr(man_cfg, "release_retreat_cmd_xy_abs_max", 0.12)),  # Max XY velocity
        base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),  # Max angular velocity
    )

    # ================================================================
    # BASE CONVERGENCE CHECKING: Distance-based convergence for XY positions
    # ================================================================
    # Retrieve 2D retreat targets from blackboard (computed during init)
    left_target_xy = node.bb.get("release_retreat_left_target_xy", None)
    right_target_xy = node.bb.get("release_retreat_right_target_xy", None)
    # Initialize distances to infinity (in case targets not found in blackboard)
    l_dist = float("inf")
    r_dist = float("inf")
    # Compute distance from current left base position to target XY
    if isinstance(left_target_xy, (list, tuple)) and len(left_target_xy) >= 2:
        l_dist = float(math.hypot(float(left_target_xy[0]) - float(left_base[0]), float(left_target_xy[1]) - float(left_base[1])))
    # Compute distance from current right base position to target XY
    if isinstance(right_target_xy, (list, tuple)) and len(right_target_xy) >= 2:
        r_dist = float(math.hypot(float(right_target_xy[0]) - float(right_base[0]), float(right_target_xy[1]) - float(right_base[1])))
    # Get convergence tolerance for base positions (meters)
    tol = float(getattr(man_cfg, "release_retreat_goal_tol", 0.10))
    # Check if BOTH bases reached their retreat targets (XY distance <= tolerance)
    base_reached = bool(l_dist <= tol and r_dist <= tol)

    # ================================================================
    # ARM CONVERGENCE CHECKING: Joint-space error-based convergence for home config
    # ================================================================
    # Retrieve home joint configurations from blackboard (computed during init)
    q_left_goal = node.bb.get("release_home_goal_left", None)
    q_right_goal = node.bb.get("release_home_goal_right", None)
    # Compute max joint error: max(|q_current - q_goal|) for multi-joint arms
    l_err = _max_joint_error(node, left_arm_jp, q_left_goal)
    r_err = _max_joint_error(node, right_arm_jp, q_right_goal)
    # Check if BOTH arms reached home: both errors below joint tolerance
    arm_reached = bool(
        np.isfinite(l_err)
        and np.isfinite(r_err)
        and l_err <= float(node.approach_arm_joint_tol)
        and r_err <= float(node.approach_arm_joint_tol)
    )

    # ================================================================
    # TIMEOUT CHECK + OVERALL CONVERGENCE DETECTION
    # ================================================================
    # Track elapsed time since retreat started
    elapsed = _ros_now_s(node) - float(retreat_t0)
    # Timeout = expected retreat time + 2 second margin
    timeout = float(release_retreat_time + 2.0)
    # Overall convergence: BOTH base AND arm must reach targets (can timeout otherwise)
    reached = bool(base_reached and arm_reached)
    
    # Log progress every 1 second
    node._info_throttled(
        "release_retreat_track",
        bt_fmt(
            f"[Release] retreat tracking base={base_reached} arm={arm_reached} "
            f"L_dist={l_dist:.3f} R_dist={r_dist:.3f} "
            f"L_err={l_err:.3f} R_err={r_err:.3f} elapsed={elapsed:.2f}/{timeout:.2f}s"
        ),
        period_s=1.0,
    )
    
    # Check if convergence achieved
    if reached:
        # ================================================================
        # RETREAT COMPLETE: Stop movement and clean up blackboard
        # ================================================================
        # Stop all controllers (arm + base) since we've reached targets
        node.stop_all_movement()
        # Clear action timer (no longer needed for this action)
        node.clear_action_timer("ReleaseRetreat")
        # Clean blackboard: remove retreat targets and home goals (no longer needed)
        node.bb.pop("release_retreat_left_target_xy", None)
        node.bb.pop("release_retreat_right_target_xy", None)
        node.bb.pop("release_home_goal_left", None)
        node.bb.pop("release_home_goal_right", None)
        # Clean release-specific data (goals for open, drop targets, etc.)
        node.bb.pop("release_ref_xyz", None)
        node.bb.pop("drop_target_xyz", None)
        node.bb.pop("drop_target_source", None)
        # Clear phase tracker so next action uses fresh state
        node.bb.pop(phase_key, None)
        # Reset pause state for Release action
        _phase_pause_reset(node, "Release")
        node.get_logger().info(bt_fmt("[Release] completed"))
        return True

    # ================================================================
    # FORCE TIMEOUT: Hard limit - complete phase even if not converged
    # ================================================================
    # If elapsed time exceeds hard force_timeout (typically 12-18 seconds),
    # give up waiting and complete the action (safety fallback to avoid stuck state)
    force_timeout = float(max(timeout + 6.0, getattr(man_cfg, "release_open_force_after_s", 12.0)))
    if elapsed >= force_timeout:
        # Log force timeout: we're giving up on perfect convergence
        node._warn_throttled(
            "release_retreat_force_complete",
            bt_fmt(
                "[Release] retreat force-timeout reached, completing phase "
                f"(L_err={l_err:.3f}, R_err={r_err:.3f}, L_dist={l_dist:.3f}, R_dist={r_dist:.3f})"
            ),
            period_s=2.0,
        )
        # Stop all movement and clean up action state
        node.stop_all_movement()
        node.clear_action_timer("ReleaseRetreat")
        # Clean blackboard of retreat-specific data
        node.bb.pop("release_retreat_left_target_xy", None)
        node.bb.pop("release_retreat_right_target_xy", None)
        node.bb.pop("release_home_goal_left", None)
        node.bb.pop("release_home_goal_right", None)
        node.bb.pop("release_ref_xyz", None)
        node.bb.pop("drop_target_xyz", None)
        node.bb.pop("drop_target_source", None)
        node.bb.pop(phase_key, None)
        _phase_pause_reset(node, "Release")
        node.get_logger().info(bt_fmt("[Release] completed (force-timeout)"))
        return True  # SUCCESS - action complete (forced)

    # ================================================================
    # NORMAL TIMEOUT: Soft limit - extend phase but keep trying
    # ================================================================
    # If elapsed time exceeds normal timeout but hasn't hit force timeout yet,
    # log warning and keep trying (don't give up completely)
    if elapsed >= timeout:
        node._warn_throttled(
            "release_retreat_extend",
            bt_fmt(
                "[Release] retreat timeout without convergence, extending stage "
                f"(L_err={l_err:.3f}, R_err={r_err:.3f}, L_dist={l_dist:.3f}, R_dist={r_dist:.3f})"
            ),
            period_s=2.0,
        )
        rclpy.spin_once(node, timeout_sec=0.01)
        return None  # RUNNING - keep trying until force_timeout

    # ================================================================
    # CONTINUE EXECUTION: Neither timeout nor convergence - keep looping
    # ================================================================
    rclpy.spin_once(node, timeout_sec=0.01)
    return None  # RUNNING - keep tracking convergence
