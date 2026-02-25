#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Legacy compatibility facade for BT motion helpers/actions.

This module preserves the historical import surface while delegating all
implementations to the split modules under ``bt_xml_demo.motion``.
"""

from __future__ import annotations

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
    _float_vec,
    _ros_now_s,
    _dt_from_ros_time,
    _publish_base_cmd,
    _publish_arm_cmd,
    _publish_base_xy,
    _scaled_xy,
    _predict_world_target_from_body_velocity,
    _split_package_model_link,
)
from bt_xml_demo.motion.pose_targets import (
    _get_live_package_xyz,
    _normalize_yaw,
    _offset_xy_in_target_frame,
    _align_pair_vector_to_target_yaw,
    _get_live_package_pose6,
    _resolve_transport_destination_xy,
    _resolve_drop_target_xyz,
    _get_live_tp_state,
    _default_side_pkg_offset,
    _resolve_hold_reference_mode,
    _resolve_pkg_reference_xyz,
    _pkg_xyz_for_alignment,
    _compute_rigid_pkg_base_targets,
)
from bt_xml_demo.motion.tp_runtime import (
    _build_base_cmd_to_xy,
    _execute_tp_arm_control,
    _execute_tp_full_control,
    _execute_tp_arm_hold,
    _init_tp_base_stage,
    _init_tp_arm_joint_stage,
    _set_tp_ee_traj,
    _pick_open_axis_idx,
    _ee_goal_reached,
    _default_rpy_for_side,
    _build_pick_waypoint_stage_plan,
    _max_joint_error,
    _clip_descend_pick_joint_goals,
    _init_descend_pick_tp,
    _init_pre_transport_tp,
)
from bt_xml_demo.motion.pkg_hold import (
    _ensure_pkg_hold_state,
    _capture_pkg_grasp_offsets,
    _pkg_hold_goal_pose,
    _replan_pkg_hold_tp,
    _log_pkg_hold_quality,
    _get_robot_model_name_for_side,
    _get_arm_attach_link_name,
    _attach_package_to_arms,
    _detach_package_from_arms,
    _log_force_proxy,
)
from bt_xml_demo.motion.phase_state import (
    _reset_pick_waypoint_runtime,
    _reset_pkg_hold_runtime,
    _phase_pause_key,
    _phase_pause_gate,
    _phase_pause_reset,
    _reset_movebase_runtime,
    _reset_adjust_positioning_bb,
)
from bt_xml_demo.motion.action_approach import ApproachObject
from bt_xml_demo.motion.action_lift import LiftObj
from bt_xml_demo.motion.action_movebase import MoveBase
from bt_xml_demo.motion.action_drop import Drop
from bt_xml_demo.motion.action_release import Release

# Backward-compatible alias used by extracted action bodies.
_require_node = require_node

__all__ = [
    "bt_fmt",
    "get_current_bt_name",
    "require_node",
    "_require_node",
    "_sanitize_arm_cmd",
    "_sanitize_base_cmd",
    "_float_vec",
    "_ros_now_s",
    "_dt_from_ros_time",
    "_publish_base_cmd",
    "_publish_arm_cmd",
    "_publish_base_xy",
    "_scaled_xy",
    "_predict_world_target_from_body_velocity",
    "_split_package_model_link",
    "_get_live_package_xyz",
    "_normalize_yaw",
    "_offset_xy_in_target_frame",
    "_align_pair_vector_to_target_yaw",
    "_get_live_package_pose6",
    "_resolve_transport_destination_xy",
    "_resolve_drop_target_xyz",
    "_get_live_tp_state",
    "_default_side_pkg_offset",
    "_resolve_hold_reference_mode",
    "_resolve_pkg_reference_xyz",
    "_pkg_xyz_for_alignment",
    "_compute_rigid_pkg_base_targets",
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
    "_reset_pick_waypoint_runtime",
    "_reset_pkg_hold_runtime",
    "_phase_pause_key",
    "_phase_pause_gate",
    "_phase_pause_reset",
    "_reset_movebase_runtime",
    "_reset_adjust_positioning_bb",
    "ApproachObject",
    "LiftObj",
    "MoveBase",
    "Drop",
    "Release",
]
