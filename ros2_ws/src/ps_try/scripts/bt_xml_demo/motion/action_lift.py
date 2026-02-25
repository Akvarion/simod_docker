#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Lift BT action."""

from __future__ import annotations

import math

import numpy as np
import rclpy

from bt_xml_demo.bt_action_context import (
    bt_fmt,
    get_current_bt_name,
    require_node,
)
from bt_xml_demo.core import (
    set_package_gravity,
)
from bt_xml_demo.motion.io_timing import (
    _float_vec,
    _predict_world_target_from_body_velocity,
    _ros_now_s,
    _scaled_xy,
)
from bt_xml_demo.motion.phase_state import (
    _phase_pause_gate,
    _phase_pause_reset,
    _reset_pick_waypoint_runtime,
    _reset_pkg_hold_runtime,
)
from bt_xml_demo.motion.pkg_hold import (
    _attach_package_to_arms,
    _capture_pkg_grasp_offsets,
    _ensure_pkg_hold_state,
    _log_force_proxy,
    _log_pkg_hold_quality,
    _replan_pkg_hold_tp,
)
from bt_xml_demo.motion.pose_targets import (
    _get_live_package_xyz,
    _get_live_tp_state,
    _resolve_pkg_reference_xyz,
)
from bt_xml_demo.motion.tp_runtime import (
    _build_pick_waypoint_stage_plan,
    _clip_descend_pick_joint_goals,
    _ee_goal_reached,
    _execute_tp_arm_control,
    _execute_tp_arm_hold,
    _execute_tp_full_control,
    _init_descend_pick_tp,
    _init_pre_transport_tp,
    _init_tp_arm_joint_stage,
    _init_tp_base_stage,
    _max_joint_error,
    _set_tp_ee_traj,
)

# Backward-compatible alias used by extracted action bodies.
_require_node = require_node

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
    # BT PRECONDITION CONTRACT
    # =================================================================
    # La sincronizzazione tra SRM1/SRM2 (alignment/approach complete) e' gestita
    # dai nodi di gating del BT in bt_action_basic.py (es. CheckAlignment,
    # CorrectBasePos). LiftObj esegue solo la fase motion/TP, assumendo che il BT
    # lo invochi quando le precondizioni sono gia' soddisfatte.

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




__all__ = ["LiftObj"]
