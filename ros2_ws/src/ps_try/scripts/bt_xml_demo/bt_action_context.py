#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shared BT action execution context (active node + BT metadata)."""

from __future__ import annotations

import contextvars
from typing import Dict, Optional

from bt_xml_demo.core import BTDemoNode


bt_node: BTDemoNode | None = None

CURRENT_BT_NAME = contextvars.ContextVar("CURRENT_BT_NAME", default="<unknown>")
CURRENT_BT_NODE_ATTRS = contextvars.ContextVar("CURRENT_BT_NODE_ATTRS", default={})


def set_bt_node(node: BTDemoNode | None):
    global bt_node
    bt_node = node


def get_bt_node() -> BTDemoNode | None:
    return bt_node


def get_current_bt_name() -> str:
    return CURRENT_BT_NAME.get("<unknown>")


def get_current_bt_attrs() -> Dict[str, str]:
    attrs = CURRENT_BT_NODE_ATTRS.get({})
    return attrs if isinstance(attrs, dict) else {}


def get_bt_attr(name: str, default: Optional[str] = None) -> Optional[str]:
    return get_current_bt_attrs().get(name, default)


def bt_fmt(message: str) -> str:
    return f"[{get_current_bt_name()}] {message}"


def require_node() -> BTDemoNode:
    if bt_node is None:
        raise RuntimeError("bt_node non inizializzato")
    return bt_node
