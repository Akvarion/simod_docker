#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Drop BT action."""

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
    _ros_now_s,
)
from bt_xml_demo.motion.phase_state import (
    _phase_pause_gate,
    _phase_pause_reset,
    _reset_adjust_positioning_bb,
)
from bt_xml_demo.motion.pkg_hold import (
    _capture_pkg_grasp_offsets,
    _log_force_proxy,
    _log_pkg_hold_quality,
    _replan_pkg_hold_tp,
)
from bt_xml_demo.motion.pose_targets import (
    _compute_rigid_pkg_base_targets,
    _get_live_package_xyz,
    _get_live_tp_state,
    _normalize_yaw,
    _offset_xy_in_target_frame,
    _pkg_xyz_for_alignment,
    _resolve_drop_target_xyz,
    _resolve_pkg_reference_xyz,
)
from bt_xml_demo.motion.tp_runtime import (
    _execute_tp_arm_control,
    _execute_tp_full_control,
    _init_tp_arm_joint_stage,
    _init_tp_base_stage,
    _max_joint_error,
)
# Backward-compatible alias used by extracted action bodies.
_require_node = require_node

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
        # Ensure explicit boolean exists for pre-drop gate consumer.
        node.bb.setdefault("drop_prepose_ok", False)
        
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
        prev_drop_target_xyz = node.bb.get("drop_target_xyz", None)
        new_drop_target_xyz = (
            [float(drop_target_xyz[0]), float(drop_target_xyz[1]), float(drop_target_xyz[2])]
            if isinstance(drop_target_xyz, (list, tuple)) and len(drop_target_xyz) >= 3
            else None
        )
        node.bb["drop_target_xyz"] = new_drop_target_xyz
        # Record resolution method string: "config", "gazebo_model", "estimated", etc.
        # Useful for diagnostics to understand which target source was successful
        node.bb["drop_target_source"] = str(drop_target_src)
        if (
            isinstance(prev_drop_target_xyz, (list, tuple))
            and len(prev_drop_target_xyz) >= 3
            and isinstance(new_drop_target_xyz, (list, tuple))
            and len(new_drop_target_xyz) >= 3
        ):
            delta = float(
                math.sqrt(
                    (float(prev_drop_target_xyz[0]) - float(new_drop_target_xyz[0])) ** 2
                    + (float(prev_drop_target_xyz[1]) - float(new_drop_target_xyz[1])) ** 2
                    + (float(prev_drop_target_xyz[2]) - float(new_drop_target_xyz[2])) ** 2
                )
            )
            if delta > 1e-3:
                node.bb["drop_prepose_ok"] = False
                _reset_adjust_positioning_bb(node)

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
                left_target_xy, right_target_xy, pair_xy, dx, dy = _compute_rigid_pkg_base_targets(
                    node=node,
                    man_cfg=man_cfg,
                    left_base=left_base,
                    right_base=right_base,
                    pkg_xy=[float(pkg_ref_xyz[0]), float(pkg_ref_xyz[1])],
                    drop_target_xy=[float(drop_target_xyz[0]), float(drop_target_xyz[1])],
                    pair_xy=None,
                )
                # Store pair geometry in blackboard for use in replanning if package moves
                node.bb["drop_base_pair_xy"] = pair_xy
                # For logging: show offsets computed from package to target
                target_mode_msg = (
                    f"rigid_pkg(dx={dx:.3f},dy={dy:.3f},"
                    f"yaw_align={bool(getattr(man_cfg, 'drop_rigid_pkg_align_yaw', True))},"
                    f"yaw_blend={float(getattr(man_cfg, 'drop_rigid_pkg_yaw_blend', 1.0)):.2f})"
                )
            # ===== FIXED OFFSET MODE (FALLBACK) =====
            # If rigid_pkg mode disabled or package position not estimatable
            # Use static XY offsets from drop target (default: L at -0.6m X and R at +0.6m X, both -0.7m Y)
            else:
                drop_target_yaw = float(_normalize_yaw(float(node.bb.get("drop_target_yaw", 0.0))))
                # ===== LEFT BASE TARGET: STATIC OFFSET LEFT OF DROP POINT =====
                # Position left base at (drop_x + left_offset_x, drop_y + offset_y)
                # Default offset: -0.60m X, -0.70m Y (to left-front of drop target)
                left_target_xy = _offset_xy_in_target_frame(
                    [float(drop_target_xyz[0]), float(drop_target_xyz[1])],
                    float(getattr(man_cfg, "drop_base_left_offset_x", -0.60)),
                    float(getattr(man_cfg, "drop_base_offset_y", -0.70)),
                    drop_target_yaw,
                )
                # ===== RIGHT BASE TARGET: STATIC OFFSET RIGHT OF DROP POINT =====
                # Position right base at (drop_x + right_offset_x, drop_y + offset_y)
                # Default offset: +0.60m X, -0.70m Y (to right-front of drop target)
                right_target_xy = _offset_xy_in_target_frame(
                    [float(drop_target_xyz[0]), float(drop_target_xyz[1])],
                    float(getattr(man_cfg, "drop_base_right_offset_x", 0.60)),
                    float(getattr(man_cfg, "drop_base_offset_y", -0.70)),
                    drop_target_yaw,
                )
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
        drop_base_traj_time_default = float(
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
        if bool(node.bb.get("drop_prepose_ok", False)):
            drop_base_traj_time = float(
                max(
                    0.3,
                    getattr(
                        man_cfg,
                        "drop_base_align_traj_time_short",
                        getattr(man_cfg, "drop_base_align_traj_time", drop_base_traj_time_default),
                    ),
                )
            )
        else:
            drop_base_traj_time = drop_base_traj_time_default
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
                pair = node.bb.get("drop_base_pair_xy", None)
                left_target_xy, right_target_xy, pair_xy, _, _ = _compute_rigid_pkg_base_targets(
                    node=node,
                    man_cfg=man_cfg,
                    left_base=left_base,
                    right_base=right_base,
                    pkg_xy=[float(pkg_xyz_eval[0]), float(pkg_xyz_eval[1])],
                    drop_target_xy=[float(drop_target_pkg_xy[0]), float(drop_target_pkg_xy[1])],
                    pair_xy=pair,
                )
                node.bb["drop_base_pair_xy"] = pair_xy
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
            _reset_adjust_positioning_bb(node)
            
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
        _reset_adjust_positioning_bb(node)
        
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




__all__ = ["Drop"]
