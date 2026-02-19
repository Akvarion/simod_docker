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


def _parse_int_attr(attrs, keys, default):
    for key in keys:
        if key in attrs:
            raw = str(attrs.get(key, "")).strip()
            if raw == "":
                return default
            try:
                return int(raw)
            except Exception:
                return default
    return default


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
            setattr(current_node, "_bt_attrs", dict(xml_elem.attrib) if xml_elem.attrib else {})
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
                leaf_node = parent_node._Tree__add_child("Action", depth, child_index, func_name, leaf_func)
                setattr(leaf_node, "_bt_attrs", attrs)
            elif parent_node is not None:
                raise ValueError(
                    f"[BT] XML node '{func_name}' non mappato in all_funcs. "
                    "Aggiungi la funzione in runner.py/all_funcs oppure correggi il tag nell'XML."
                )

        if len(xml_elem) > 0:
            new_depth = depth + 1
            for i, child in enumerate(xml_elem):
                _recurse(child, bt_root, current_node, new_depth, i, is_first)

        return current_node

    _recurse(root_elem, self, None, 1, 0, True)


def bt_run(self):
    """Simplified run() semantics for leaves and control nodes."""
    node_type = getattr(self, "type_", None)
    node_attrs = getattr(self, "_bt_attrs", {}) or {}

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
        n_children = len(getattr(self, "child_", []))
        success_count = 0
        failure_count = 0
        running_seen = False
        for child in self.child_:
            status = child.run()
            if status is True:
                success_count += 1
            elif status is False:
                failure_count += 1
            else:
                running_seen = True

        if node_type == "ParallelAll":
            # BT.CPP-style ParallelAll: all children should succeed; max_failures can relax failure trigger.
            max_failures = max(
                0,
                _parse_int_attr(node_attrs, ("max_failures", "maxFailures"), 0),
            )
            if failure_count > max_failures:
                self._status = "FAIL"
                return False
            if (success_count + failure_count) == n_children:
                # all children completed and failures are within tolerated budget
                self._status = "SUCCESS"
                return True
            self._status = "RUNNING"
            return None

        # BT.CPP-style Parallel with thresholds
        success_th = _parse_int_attr(
            node_attrs,
            ("success_threshold", "successThreshold", "success_count", "successCount"),
            n_children,
        )
        failure_th = _parse_int_attr(
            node_attrs,
            ("failure_threshold", "failureThreshold", "failure_count", "failureCount"),
            1,
        )
        success_th = min(max(1, success_th), max(1, n_children))
        failure_th = min(max(1, failure_th), max(1, n_children))

        if success_count >= success_th:
            self._status = "SUCCESS"
            return True
        if failure_count >= failure_th:
            self._status = "FAIL"
            return False
        self._status = "RUNNING"
        return None

    if node_type == "RetryUntilSuccessful":
        if not self.child_:
            self._status = "SUCCESS"
            return True
        max_attempts = _parse_int_attr(
            node_attrs,
            ("num_attempts", "numAttempts"),
            -1,  # -1 => infinite retries (BT.CPP-like)
        )
        if not hasattr(self, "_retry_count"):
            self._retry_count = 0
        child = self.child_[0]
        status = child.run()
        if status is True:
            self._status = "SUCCESS"
            self._retry_count = 0
            return True
        if status is False:
            self._retry_count += 1
            if max_attempts >= 0 and self._retry_count >= max_attempts:
                self._status = "FAIL"
                return False
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
