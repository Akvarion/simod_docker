#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shared package-hold runtime state helpers."""

from __future__ import annotations


def _ensure_pkg_hold_state(node):
    """Initialize or ensure package-hold runtime attributes on node.

    Creates/ensures:
    - _pkg_hold_offsets: offset from EE to package [left/right]
    - _pkg_hold_rpy: captured RPY for orientation hold [left/right]
    - _pkg_hold_nominal_dist: nominal distance between left/right EE
    - _pkg_hold_last_replan: timestamp for periodic hold replan
    - _pkg_hold_start_z, _pkg_hold_target_z: height control for lift/drop
    - _tp_manip_last_exec_monotonic: TP dt calculation timestamp
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


__all__ = [
    "_ensure_pkg_hold_state",
]
