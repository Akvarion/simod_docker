#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Motion BT actions (approach/lift/transport/drop/release)."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from linkattacher_msgs.srv import AttachLink, DetachLink

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

    now_mono = time.monotonic()
    cmd = node._tp_cmd_cache
    can_reuse = (
        cmd is not None
        and node._tp_cmd_cache_time is not None
        and (now_mono - node._tp_cmd_cache_time) < max(node.tp_cmd_min_period, 1e-3)
    )
    if not can_reuse:
        if node._tp_last_exec_monotonic is None:
            tp_dt = 1.0 / 30.0
        else:
            tp_dt = now_mono - node._tp_last_exec_monotonic
        node._tp_last_exec_monotonic = now_mono
        # dt robusto: evita valori troppo piccoli/grandi che destabilizzano il controllo.
        node.tp._delta_t = max(1e-3, min(float(tp_dt), 0.2))
        cmd = node.tp.execute(joint_pos=joint_pos, base_odom=base_odom)
        node._tp_cmd_cache = cmd
        node._tp_cmd_cache_time = now_mono
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
    descend_and_pick_time = float(phase_cfg.descend_and_pick_time)
    collect_time = float(phase_cfg.collect_time)
    left_arm_pick = _float_vec(motion.left_arm_pick)
    right_arm_pick = _float_vec(motion.right_arm_pick)
    collect_left_base_xy = _float_vec(motion.collect_left_base_xy_vel)
    collect_right_base_xy = _float_vec(motion.collect_right_base_xy_vel)
    left_arm_collect = _float_vec(motion.left_arm_collect)
    right_arm_collect = _float_vec(motion.right_arm_collect)
    if node.approach_only_mode:
        node._info_throttled(
            "approach_only_liftobj",
            bt_fmt(
                f"[LiftObj] paused "
                f"(approach.approach_only='{getattr(node, 'approach_only_raw', '1')}', "
                f"parsed={int(bool(node.approach_only_mode))})"
            ),
            period_s=5.0,
        )
        return None
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

    t0 = node.get_action_timer("LiftObj")
    if t0 is None or node.lift_phase is None:
        node.get_logger().info(bt_fmt(f"[LiftObj] start (dur {descend_and_pick_time + 0.5 + collect_time}s)"))
        t0 = node.start_action_timer("LiftObj")
        node.lift_phase = "descend_pick"

    elapsed = node.get_clock().now().nanoseconds/1e9 - t0

    # 1) Discesa e pick
    if node.lift_phase == "descend_pick":
        if elapsed < descend_and_pick_time:
            _publish_arm_cmd(
                node,
                left_cmd=left_arm_pick,
                right_cmd=right_arm_pick,
            )
            node.get_logger().info(bt_fmt("[LiftObj] descending..."))
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        else:
            # passa alla fase attach
            node.stop_all_movement()
            node.lift_phase = "attach"
            node.get_logger().info(bt_fmt("[LiftObj] reached pick pose, attaching..."))
            node.start_action_timer("LiftObj")  # restart timer per attach
            return None

    # 2) Attach (tempo breve)
    if node.lift_phase == "attach":
        if elapsed < 0.5:
            if node.attach_cli.service_is_ready():
                req = AttachLink.Request()
                setattr(req, "model1_name", "left_robot")
                setattr(req, "link1_name", "ur_left_wrist_3_link")
                setattr(req, "model2_name", "pacco_clone_1")
                setattr(req, "link2_name", "pacco_clone_1::link_1")
                node.attach_cli.call_async(req)
                node.get_logger().info(bt_fmt("[LiftObj] attach request sent"))
                if not node.bb.get("package_gravity_disabled", False):
                    if set_package_gravity(node, False):
                        node.bb["package_gravity_disabled"] = True
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        else:
            node.stop_all_movement()
            node.lift_phase = "collect"
            node.get_logger().info(bt_fmt("[LiftObj] attached"))
            node.start_action_timer("LiftObj")  # restart timer per collect
            return None

    # 3) Collect / sollevamento
    if node.lift_phase == "collect":
        if elapsed < collect_time:
            _publish_base_xy(
                node,
                left_xy=collect_left_base_xy,
                right_xy=collect_right_base_xy,
            )
            _publish_arm_cmd(
                node,
                left_cmd=left_arm_collect,
                right_cmd=right_arm_collect,
            )

            node.get_logger().info(bt_fmt("[LiftObj] collecting (lifting)..."))
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        else:
            node.stop_all_movement()
            node.lift_phase = None
            node.clear_action_timer("LiftObj")
            node.get_logger().info(bt_fmt("[LiftObj] completed"))
            return True

    # fallback
    node.lift_phase = None
    node.clear_action_timer("LiftObj")
    return True


def MoveBase():
    """
    Movimento delle basi (transfer):
    - usato dal supervisore per DST e ReturnToPallet
    - qui implementato come semplice "transport" a tempo da config.
    """
    node = _require_node()
    phase_cfg = node.cfg.phases
    motion = node.cfg.motion_profiles
    transport_time = float(phase_cfg.transport_time)
    left_transport_xy = _float_vec(motion.left_transport_vel_xy)
    right_transport_xy = _float_vec(motion.right_transport_vel_xy)

    t0 = node.get_action_timer("MoveBase")
    if t0 is None:
        node.get_logger().info(bt_fmt(f"[MoveBase] start ({transport_time}s)"))
        t0 = node.start_action_timer("MoveBase")

    if node.get_clock().now().nanoseconds/1e9 - t0 < transport_time:
        _publish_base_xy(
            node,
            left_xy=left_transport_xy,
            right_xy=right_transport_xy,
        )
        rclpy.spin_once(node, timeout_sec=0.01)
        return None
    else:
        node.stop_all_movement()
        node.clear_action_timer("MoveBase")
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
    descend_and_place_time = float(phase_cfg.descend_and_place_time)
    place_left_base_xy = _float_vec(motion.place_left_base_xy_vel)
    place_right_base_xy = _float_vec(motion.place_right_base_xy_vel)
    left_arm_place = _float_vec(motion.left_arm_place)
    right_arm_place = _float_vec(motion.right_arm_place)
    t0 = node.get_action_timer("Drop")
    if t0 is None:
        node.get_logger().info(bt_fmt(f"[Drop] start ({descend_and_place_time}s)"))
        t0 = node.start_action_timer("Drop")

    if node.get_clock().now().nanoseconds/1e9 - t0 < descend_and_place_time:
        _publish_base_xy(
            node,
            left_xy=place_left_base_xy,
            right_xy=place_right_base_xy,
        )
        _publish_arm_cmd(
            node,
            left_cmd=left_arm_place,
            right_cmd=right_arm_place,
        )

        rclpy.spin_once(node, timeout_sec=0.01)
        return None
    else:
        node.stop_all_movement()
        node.clear_action_timer("Drop")
        node.get_logger().info(bt_fmt("[Drop] completed"))
        return True


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
    left_arm_release = _float_vec(motion.left_arm_release)
    right_arm_release = _float_vec(motion.right_arm_release)
    retreat_left_base_xy = _float_vec(motion.collect_left_base_xy_vel)
    retreat_right_base_xy = _float_vec(motion.collect_right_base_xy_vel)
    retreat_left_arm = _float_vec(motion.left_arm_collect)
    retreat_right_arm = _float_vec(motion.right_arm_collect)
    t0 = node.get_action_timer("Release")
    if t0 is None:
        node.get_logger().info(bt_fmt(f"[Release] start ({release_time}s)"))
        t0 = node.start_action_timer("Release")

    if node.get_clock().now().nanoseconds/1e9 - t0 < release_time:
        _publish_arm_cmd(
            node,
            left_cmd=left_arm_release,
            right_cmd=right_arm_release,
        )
        rclpy.spin_once(node, timeout_sec=0.01)
        return None
    else:
        node.stop_all_movement()

        # Detach del pacco (best-effort)
        if node.detach_cli.service_is_ready():
            req = DetachLink.Request()
            setattr(req, "model1_name", "left_robot")
            setattr(req, "link1_name", "ur_left_wrist_3_link")
            setattr(req, "model2_name", "pacco_clone_1")
            setattr(req, "link2_name", "pacco_clone_1::link_1")
            node.detach_cli.call_async(req)
            node.get_logger().info(bt_fmt("[Release] detach request sent"))

        if node.bb.pop("package_gravity_disabled", False):
            set_package_gravity(node, True)

        # Inizio fase di allontanamento (retreat)
        node.clear_action_timer("Release")
        retreat_t0 = node.start_action_timer("ReleaseRetreat")
        node.get_logger().info(bt_fmt("[Release] retreat phase started"))

        while node.get_clock().now().nanoseconds/1e9 - retreat_t0 < collect_time:
            _publish_base_xy(node, left_xy=retreat_left_base_xy, right_xy=retreat_right_base_xy)
            _publish_arm_cmd(node, left_cmd=retreat_left_arm, right_cmd=retreat_right_arm)

            node.get_logger().info(bt_fmt("[Release] retreating (moving away)..."))
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.1)

        node.stop_all_movement()
        node.clear_action_timer("ReleaseRetreat")
        node.get_logger().info(bt_fmt("[Release] completed"))
        return True
