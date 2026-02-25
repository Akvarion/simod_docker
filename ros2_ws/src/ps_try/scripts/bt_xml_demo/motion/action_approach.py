#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Approach BT action."""

from __future__ import annotations

import math

import numpy as np

from bt_xml_demo.bt_action_context import (
    bt_fmt,
    get_current_bt_name,
    require_node,
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

# Backward-compatible alias used by extracted action bodies.
_require_node = require_node


def ApproachObject():
    """Approach pallet using Task Prioritization with dual-base coordination.

    DESCRIPTION:
    Dual-arm mobile robot approaches a pallet for pickup. Left (SRM1) and Right (SRM2)
    robots work in parallel, with each following its own TP trajectory. Coordinates via:
    - Base positioning: approach_base_ctrl (PD or TP-based)
    - Arm motion: TP endpoint tracking toward pallet
    - BT contract: SRM2 sync is expected from BT gating (DataReceived)

    FLOW:
    1. Validate live state (joint positions, odometry)
    2. Initialize/re-initialize TP with approach target (from pallet pose estimate)
    3. Execute TP loop: compute commands, publish to arm+base controllers
    4. Check convergence (joint+EE+base errors within tolerances)
    5. Set <TREE>_near_object flag and exit

    NOTES:
    - SRM1 and SRM2 run separately; synchronization belongs to BT gating nodes
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
            period_s=1.0,
        )
        return None  # RUNNING - keep trying until all data arrives

    # =================================================================
    # BT PRECONDITION CONTRACT
    # =================================================================
    # La sincronizzazione SRM1 -> SRM2 (payload/vision handoff) e' delegata ai
    # nodi BT di condition/gating (es. DataReceived in bt_action_basic.py).
    # ApproachObject non blocca l'esecuzione su flag orchestration, ma mantiene
    # un warning diagnostico se il dato non e' ancora disponibile.
    if side == "right" and not bool(node.bb.get("srm1_data_to_srm2", False)):
        node._warn_throttled(
            f"{tree_name}_approach_bt_gate_missing",
            bt_fmt(
                "[ApproachObject] SRM2 executing without srm1_data_to_srm2; "
                "expected BT gate via DataReceived"
            ),
            period_s=2.0,
        )

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
            bt_fmt(f"[ApproachObject] TP command vector too short: len={len(cmd)}"),
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
        left_base_dist = float(
            math.hypot(float(left_target_xy[0]) - float(left_base[0]), float(left_target_xy[1]) - float(left_base[1]))
        )
    if right_target_xy is not None:
        right_base_dist = float(
            math.hypot(float(right_target_xy[0]) - float(right_base[0]), float(right_target_xy[1]) - float(right_base[1]))
        )

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
        node.tp_arm_cmd_abs_max,  # Clip to max joint velocity
    )
    right_arm_cmd_vals = _sanitize_arm_cmd(
        node._get_arm_cmd_values(cmd, "right"),  # Extract right arm velocities
        node.tp_arm_cmd_abs_max,
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

__all__ = ["ApproachObject"]
