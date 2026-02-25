#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Phase gating and runtime state helpers for BT motion actions."""

from __future__ import annotations

import sys

from bt_xml_demo.bt_action_context import (
    bt_fmt,
    get_current_bt_name,
)
from bt_xml_demo.motion.hold_state import (
    _ensure_pkg_hold_state,
)


def _reset_pick_waypoint_runtime(node):
    """Clean up state from waypoint-based pick phase."""
    node.bb.pop("lift_pick_waypoints", None)
    node.bb.pop("lift_pick_stage_idx", None)
    node.bb.pop("lift_pick_stage_active", None)
    node.bb.pop("lift_pick_stage_start_s", None)


def _reset_pkg_hold_runtime(node):
    """Fully reset package hold runtime state."""
    _ensure_pkg_hold_state(node)
    node._pkg_hold_offsets = {"left": None, "right": None}
    node._pkg_hold_rpy = {"left": None, "right": None}
    node._pkg_hold_nominal_dist = float("nan")
    node._pkg_hold_last_replan = 0.0
    node._pkg_hold_start_z = None
    node._pkg_hold_target_z = None
    node._tp_manip_last_exec_monotonic = None


def _phase_pause_key(phase_name: str) -> str:
    """Blackboard key used to store phase pause approval."""
    return f"phase_pause_approved::{str(phase_name)}"


def _phase_pause_gate(node, phase_name: str) -> bool:
    """Optional interactive pause before phase (debug/manual control)."""
    if not bool(getattr(node, "pause_between_phases", False)):
        return True
    if get_current_bt_name() != "Supervisor":
        return True

    key = _phase_pause_key(phase_name)
    if bool(node.bb.get(key, False)):
        return True

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
    """Clear pause confirmation flag for phase restart."""
    node.bb.pop(_phase_pause_key(phase_name), None)


def _reset_movebase_runtime(node):
    """Clear MoveBase runtime keys."""
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


def _reset_adjust_positioning_bb(node):
    """Clear BT-side pre-drop micro-correction runtime keys."""
    node.bb.pop("adjust_positioning_stage", None)
    node.bb.pop("adjust_positioning_t0", None)
    node.bb.pop("adjust_positioning_last_err", None)
    node.bb.pop("adjust_positioning_left_target_xy", None)
    node.bb.pop("adjust_positioning_right_target_xy", None)

__all__ = [
    "_reset_pick_waypoint_runtime",
    "_reset_pkg_hold_runtime",
    "_phase_pause_key",
    "_phase_pause_gate",
    "_phase_pause_reset",
    "_reset_movebase_runtime",
    "_reset_adjust_positioning_bb",
]
