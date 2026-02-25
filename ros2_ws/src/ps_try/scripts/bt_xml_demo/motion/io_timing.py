#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS I/O and timing helpers for BT motion actions."""

from __future__ import annotations

import math

import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def _float_vec(values):
    """Convert numeric sequence to list of floats, ensuring consistent type."""
    return [float(v) for v in values]


def _ros_now_s(node) -> float:
    """Get current ROS time in seconds (float) from node clock."""
    return float(node.get_clock().now().nanoseconds) / 1e9


def _dt_from_ros_time(now_s: float, last_s: float, default_dt: float = 1.0 / 30.0) -> float:
    """Compute robust time delta handling simulated clock resets and jumps."""
    if last_s is None:
        return float(default_dt)
    dt = float(now_s) - float(last_s)
    if (not np.isfinite(dt)) or dt <= 0.0:
        return float(default_dt)
    return float(dt)


def _publish_base_cmd(node, left_cmd=None, right_cmd=None):
    """Publish Twist commands to base controllers for left/right platforms."""
    if left_cmd is not None:
        tl = Twist()
        tl.linear.x, tl.linear.y, tl.angular.z = left_cmd
        node.left_base_pub.publish(tl)
    if right_cmd is not None:
        tr = Twist()
        tr.linear.x, tr.linear.y, tr.angular.z = right_cmd
        node.right_base_pub.publish(tr)


def _publish_arm_cmd(node, left_cmd=None, right_cmd=None):
    """Publish joint velocity commands to arm controllers for left/right."""
    if left_cmd is not None:
        la = Float64MultiArray()
        la.data = left_cmd
        node.left_arm_pub.publish(la)
    if right_cmd is not None:
        ra = Float64MultiArray()
        ra.data = right_cmd
        node.right_arm_pub.publish(ra)


def _publish_base_xy(node, left_xy=None, right_xy=None):
    """Publish only base XY components (wz=0)."""
    left_cmd = [left_xy[0], left_xy[1], 0.0] if left_xy is not None else None
    right_cmd = [right_xy[0], right_xy[1], 0.0] if right_xy is not None else None
    _publish_base_cmd(node, left_cmd=left_cmd, right_cmd=right_cmd)


def _scaled_xy(xy, scale: float):
    """Scale XY vector while preserving list format."""
    if xy is None:
        return None
    s = float(scale)
    return [float(xy[0]) * s, float(xy[1]) * s]


def _predict_world_target_from_body_velocity(base_pose, vel_xy, duration_s: float):
    """Convert body-frame velocity to world-frame target position after duration."""
    yaw = float(base_pose[5])
    vx_b = float(vel_xy[0])
    vy_b = float(vel_xy[1])
    dt = float(max(duration_s, 0.0))
    dx_w = (math.cos(yaw) * vx_b - math.sin(yaw) * vy_b) * dt
    dy_w = (math.sin(yaw) * vx_b + math.cos(yaw) * vy_b) * dt
    return [float(base_pose[0]) + dx_w, float(base_pose[1]) + dy_w]


def _split_package_model_link(node):
    """Extract (model, link) of package from config `package.link_name`."""
    raw = str(getattr(node.cfg.package, "link_name", "")).strip()
    if "::" in raw:
        model = raw.split("::", 1)[0].strip()
        return model, raw
    model = str(getattr(node, "_gazebo_pallet_pose_start_model", "") or "").strip()
    if not model:
        model = "pacco_clone_1"
    return model, raw or "pacco_clone_1::link_1"

__all__ = [
    "_float_vec",
    "_ros_now_s",
    "_dt_from_ros_time",
    "_publish_base_cmd",
    "_publish_arm_cmd",
    "_publish_base_xy",
    "_scaled_xy",
    "_predict_world_target_from_body_velocity",
    "_split_package_model_link",
]
