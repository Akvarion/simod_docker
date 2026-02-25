#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""MoveBase BT action."""

from __future__ import annotations

import math

import numpy as np
import rclpy

from bt_xml_demo.bt_action_context import (
    bt_fmt,
    require_node,
)
from bt_xml_demo.motion.io_timing import (
    _float_vec,
    _predict_world_target_from_body_velocity,
    _ros_now_s,
    _scaled_xy,
)
from bt_xml_demo.motion.pkg_hold import (
    _log_force_proxy,
    _log_pkg_hold_quality,
    _replan_pkg_hold_tp,
)
from bt_xml_demo.motion.phase_state import (
    _phase_pause_gate,
    _phase_pause_reset,
    _reset_adjust_positioning_bb,
    _reset_movebase_runtime,
)
from bt_xml_demo.motion.pose_targets import (
    _get_live_package_xyz,
    _get_live_tp_state,
    _resolve_pkg_reference_xyz,
    _resolve_transport_destination_xy,
)
from bt_xml_demo.motion.tp_runtime import (
    _execute_tp_full_control,
    _init_tp_arm_joint_stage,
    _init_tp_base_stage,
    _max_joint_error,
)

# Backward-compatible alias used by extracted action bodies.
_require_node = require_node


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
        # Expose BT-level completion flag for downstream checks (e.g. InAPositionToDrop?).
        node.bb["movebase_done"] = False
        # New transport cycle invalidates previous pre-drop idoneity.
        node.bb["drop_prepose_ok"] = False
        _reset_adjust_positioning_bb(node)
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
                            f"pkg_y={float(pkg_xyz[1]):.3f})"
                        )
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
            node.bb["movebase_done"] = True
            node.get_logger().info(bt_fmt("[MoveBase] completed"))
            return True  # Action SUCCESS - proceed to next phase

        rclpy.spin_once(node, timeout_sec=0.01)
        return None  # RUNNING - continue transport execution

    node.stop_all_movement()
    node.clear_action_timer("MoveBase")
    _reset_movebase_runtime(node)
    _phase_pause_reset(node, "MoveBase")
    node.bb["movebase_done"] = True
    node.get_logger().info(bt_fmt("[MoveBase] completed"))
    return True


__all__ = ["MoveBase"]
