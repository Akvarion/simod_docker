#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Facade module exporting BT actions and tree patch helpers."""

from bt_xml_demo.bt_action_context import (
    get_bt_node,
    set_bt_node,
    CURRENT_BT_NAME,
    CURRENT_BT_NODE_ATTRS,
    get_current_bt_name,
    get_current_bt_attrs,
    get_bt_attr,
    bt_fmt,
)
from bt_xml_demo.bt_tree_patch import assign_bt_name, apply_tree_patches
from bt_xml_demo.bt_action_basic import (
    Sync,
    FindObj,
    CalculateGoal,
    CheckAlignment,
    Set,
    CloseGripper,
    SaySomething,
    NearObj,
    DataReceived,
    Controller,
    CorrectBasePos,
    AdjustPositioning,
    ReturnToPallet,
)
from bt_xml_demo.bt_action_motion import (
    ApproachObject,
    LiftObj,
    MoveBase,
    Drop,
    Release,
)


apply_tree_patches()


__all__ = [
    "get_bt_node",
    "set_bt_node",
    "CURRENT_BT_NAME",
    "CURRENT_BT_NODE_ATTRS",
    "get_current_bt_name",
    "get_current_bt_attrs",
    "get_bt_attr",
    "bt_fmt",
    "assign_bt_name",
    "Sync",
    "FindObj",
    "CalculateGoal",
    "CheckAlignment",
    "Set",
    "CloseGripper",
    "SaySomething",
    "NearObj",
    "DataReceived",
    "Controller",
    "CorrectBasePos",
    "AdjustPositioning",
    "ReturnToPallet",
    "ApproachObject",
    "LiftObj",
    "MoveBase",
    "Drop",
    "Release",
]
