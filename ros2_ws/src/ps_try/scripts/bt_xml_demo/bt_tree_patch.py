#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tree parser/runner patching for BehaviorTree.CPP-style XML."""

from __future__ import annotations

from xml.etree.ElementTree import parse

from behaviortree.tree import Tree

from bt_xml_demo import bt_action_context as ctx


def assign_bt_name(tree: Tree, name: str):
    setattr(tree, "bt_name", name)
    for child in getattr(tree, "child_", []):
        assign_bt_name(child, name)


def gen_tree_from_btcpp(self, xmlfile, funcs):
    """Custom parser for XML exported from Groot/BehaviorTree.CPP."""
    func_dict = {f.__name__: f for f in funcs}

    tree_xml = parse(xmlfile)
    root_elem = tree_xml.getroot()

    control_nodes = [
        "Sequence", "Fallback", "ParallelAll", "Parallel",
        "RetryUntilSuccessful", "BehaviorTree", "root",
        "TreeNodesModel", "SubTree",
    ]
    metadata_nodes = ["TreeNodesModel"]

    def _recurse(xml_elem, bt_root, parent_node, depth, child_index, is_first):
        current_node = parent_node

        if xml_elem.tag in metadata_nodes:
            return current_node

        if xml_elem.tag in control_nodes and xml_elem.tag not in ("root", "BehaviorTree"):
            if is_first:
                current_node = bt_root._Tree__add_child(xml_elem.tag, depth, child_index)
                is_first = False
            else:
                current_node = parent_node._Tree__add_child(xml_elem.tag, depth, child_index)
        elif xml_elem.tag not in control_nodes:
            func_name = xml_elem.tag
            func_obj = func_dict.get(func_name, None)
            if parent_node is not None and func_obj is not None:
                attrs = dict(xml_elem.attrib) if xml_elem.attrib else {}
                if attrs:
                    def _wrapped_func(_func=func_obj, _attrs=attrs):
                        token_attrs = ctx.CURRENT_BT_NODE_ATTRS.set(_attrs)
                        try:
                            return _func()
                        finally:
                            ctx.CURRENT_BT_NODE_ATTRS.reset(token_attrs)
                    leaf_func = _wrapped_func
                else:
                    leaf_func = func_obj
                parent_node._Tree__add_child("Action", depth, child_index, func_name, leaf_func)
            elif parent_node is not None:
                print(f"[BT] WARNING: funzione '{func_name}' non trovata nel dizionario funzioni.")

        if len(xml_elem) > 0:
            new_depth = depth + 1
            for i, child in enumerate(xml_elem):
                _recurse(child, bt_root, current_node, new_depth, i, is_first)

        return current_node

    _recurse(root_elem, self, None, 1, 0, True)


def bt_run(self):
    """Simplified run() semantics for leaves and control nodes."""
    node_type = getattr(self, "type_", None)

    if getattr(self, "id_", None) is not None and getattr(self, "func_", None) is not None:
        if getattr(self, "_status", "IDLE") == "SUCCESS":
            return True

        tree_name = getattr(self, "bt_name", "<unnamed>")
        token = ctx.CURRENT_BT_NAME.set(tree_name)
        token_attrs = ctx.CURRENT_BT_NODE_ATTRS.set({})
        try:
            result = self.func_()
        finally:
            ctx.CURRENT_BT_NODE_ATTRS.reset(token_attrs)
            ctx.CURRENT_BT_NAME.reset(token)

        if result is True:
            self._status = "SUCCESS"
            return True
        if result is None:
            self._status = "RUNNING"
            return None
        if result is False:
            self._status = "FAIL"
            return False

        self._status = "RUNNING"
        return None

    if not hasattr(self, "_status"):
        self._status = "IDLE"

    if node_type == "Sequence":
        for child in self.child_:
            status = child.run()
            if status is None:
                self._status = "RUNNING"
                return None
            if status is False:
                self._status = "FAIL"
                return False
        self._status = "SUCCESS"
        return True

    if node_type == "Fallback":
        for child in self.child_:
            status = child.run()
            if status is True:
                self._status = "SUCCESS"
                return True
            if status is None:
                self._status = "RUNNING"
                return None
        self._status = "FAIL"
        return False

    if node_type in ("ParallelAll", "Parallel"):
        running_seen = False
        for child in self.child_:
            status = child.run()
            if status is False:
                self._status = "FAIL"
                return False
            if status is None:
                running_seen = True
        if running_seen:
            self._status = "RUNNING"
            return None
        self._status = "SUCCESS"
        return True

    if node_type == "RetryUntilSuccessful":
        if not self.child_:
            self._status = "SUCCESS"
            return True
        child = self.child_[0]
        status = child.run()
        if status is True:
            self._status = "SUCCESS"
            return True
        self._status = "RUNNING"
        return None

    for child in self.child_:
        status = child.run()
        if status is not None:
            return status
    return None


def apply_tree_patches():
    Tree.gen_tree_from_btcpp = gen_tree_from_btcpp
    Tree.run = bt_run
