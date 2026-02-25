#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Release BT action."""

from __future__ import annotations

import math

import numpy as np
import rclpy

from bt_xml_demo.bt_action_context import (
    bt_fmt,
    require_node,
)
from bt_xml_demo.core import (
    set_package_gravity,
    wake_package_body,
)
from bt_xml_demo.motion.io_timing import (
    _ros_now_s,
)
from bt_xml_demo.motion.phase_state import (
    _phase_pause_gate,
    _phase_pause_reset,
    _reset_pkg_hold_runtime,
)
from bt_xml_demo.motion.pkg_hold import (
    _detach_package_from_arms,
)
from bt_xml_demo.motion.pose_targets import (
    _get_live_package_pose6,
    _get_live_tp_state,
    _normalize_yaw,
    _offset_xy_in_target_frame,
    _resolve_drop_target_xyz,
    _resolve_pkg_reference_xyz,
)
from bt_xml_demo.motion.tp_runtime import (
    _default_rpy_for_side,
    _ee_goal_reached,
    _execute_tp_arm_control,
    _execute_tp_full_control,
    _init_tp_arm_joint_stage,
    _init_tp_base_stage,
    _max_joint_error,
    _pick_open_axis_idx,
    _set_tp_ee_traj,
)

# Backward-compatible alias used by extracted action bodies.
_require_node = require_node


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
            # Use drop target as reference for open gesture offset.
            # Offsets are interpreted in target frame (yaw-aware).
            release_ref_pose6 = node.bb.get("drop_target_pose6", None)
            if not (isinstance(release_ref_pose6, (list, tuple)) and len(release_ref_pose6) >= 6):
                release_ref_xyz, _ = _resolve_drop_target_xyz(node)
                if isinstance(release_ref_xyz, (list, tuple)) and len(release_ref_xyz) >= 3:
                    release_ref_pose6 = [
                        float(release_ref_xyz[0]),
                        float(release_ref_xyz[1]),
                        float(release_ref_xyz[2]),
                        0.0,
                        0.0,
                        float(_normalize_yaw(float(node.bb.get("drop_target_yaw", 0.0)))),
                    ]
            if not (isinstance(release_ref_pose6, (list, tuple)) and len(release_ref_pose6) >= 6):
                release_ref_pose6 = _get_live_package_pose6(node)
            if not (isinstance(release_ref_pose6, (list, tuple)) and len(release_ref_pose6) >= 6):
                release_ref_xyz = _resolve_pkg_reference_xyz(node, left_arm_jp, right_arm_jp, left_base, right_base)
                if isinstance(release_ref_xyz, (list, tuple)) and len(release_ref_xyz) >= 3:
                    release_ref_pose6 = [
                        float(release_ref_xyz[0]),
                        float(release_ref_xyz[1]),
                        float(release_ref_xyz[2]),
                        0.0,
                        0.0,
                        0.0,
                    ]
            # All fallbacks exhausted: cannot proceed without package reference
            if not (isinstance(release_ref_pose6, (list, tuple)) and len(release_ref_pose6) >= 6):
                node._warn_throttled("release_no_pkg_init", bt_fmt("[Release] package pose unavailable"), period_s=1.0)
                rclpy.spin_once(node, timeout_sec=0.01)
                return None  # RUNNING - wait for package pose data
            release_ref_xyz = [float(release_ref_pose6[0]), float(release_ref_pose6[1]), float(release_ref_pose6[2])]
            release_ref_yaw = float(_normalize_yaw(float(release_ref_pose6[5])))

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

            # LEFT EE GOAL: Package center + configured offsets in target frame.
            left_xy = _offset_xy_in_target_frame(
                [float(release_ref_xyz[0]), float(release_ref_xyz[1])],
                float(getattr(man_cfg, "release_open_left_offset_x", -0.37)),
                float(getattr(man_cfg, "release_open_offset_y", -0.10)),
                release_ref_yaw,
            )
            left_goal[0] = float(left_xy[0])  # X: offset left in target frame
            left_goal[1] = float(left_xy[1])  # Y: frontal/back offset in target frame
            left_goal[2] = float(release_ref_xyz[2]) + float(getattr(man_cfg, "release_open_offset_z", 0.30))       # Z: above package
            left_goal[3:6] = left_rpy_open  # Orientation: gripper open

            # RIGHT EE GOAL: Mirror of left (symmetric grasping)
            right_xy = _offset_xy_in_target_frame(
                [float(release_ref_xyz[0]), float(release_ref_xyz[1])],
                float(getattr(man_cfg, "release_open_right_offset_x", 0.37)),
                float(getattr(man_cfg, "release_open_offset_y", -0.10)),
                release_ref_yaw,
            )
            right_goal[0] = float(right_xy[0])  # X: offset right
            right_goal[1] = float(right_xy[1])  # Y: same frontal/back offset
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
            node.bb["release_ref_pose6"] = [float(v) for v in np.asarray(release_ref_pose6[:6], dtype=np.float32).tolist()]
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


__all__ = ["Release"]
