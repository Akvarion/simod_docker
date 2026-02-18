#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Basic/mock BT actions and conditions."""

from __future__ import annotations

from bt_xml_demo.bt_action_context import (
    bt_fmt,
    get_bt_attr,
    get_current_bt_name,
    require_node,
)

# Backward-compatible alias used by extracted action bodies.
_require_node = require_node
def Sync():
    """
    Nodo di sincronizzazione (SRM1/SRM2/supervisor).
    Qui simulato come attesa di 1 s, poi SUCCESS.
    """
    node = _require_node()
    timer_key = f"{get_current_bt_name()}_Sync"
    t0 = node.get_action_timer(timer_key)
    if t0 is None:
        node.get_logger().info(bt_fmt("[Sync] start"))
        t0 = node.start_action_timer(timer_key)

    if node.get_clock().now().nanoseconds/1e9 - t0 < 1.0:
        return None  # RUNNING
    node.clear_action_timer(timer_key)
    node.get_logger().info(bt_fmt("[Sync] done"))
    return True


def FindObj():
    """
    Identificazione pacco target.
    Mock: attende 1 s e restituisce SUCCESS.
    """
    node = _require_node()
    action_name = get_bt_attr("name", "FindObj")
    timer_key = f"{get_current_bt_name()}_FindObj"
    t0 = node.get_action_timer(timer_key)
    if t0 is None:
        node.get_logger().info(bt_fmt(f"[FindObj] start ({action_name})"))
        t0 = node.start_action_timer(timer_key)

    if node.get_clock().now().nanoseconds/1e9 - t0 < 1.0:
        return None
    node.clear_action_timer(timer_key)
    node.bb["target_found"] = True
    # Simula il riconoscimento pallet appena disponibili odometrie.
    tree_name = get_current_bt_name()
    if tree_name in ("SRM1", "SRM2"):
        _ = node._ensure_mock_pallet_pose()
    # SRM2 deve partire appena SRM1 ha stimato il pallet (prima della Sync).
    if tree_name == "SRM1" and "pallet" in str(action_name).lower():
        node.bb["srm1_data_to_srm2"] = bool(node.bb.get("pallet_info_ready", False))
        if node.bb["srm1_data_to_srm2"]:
            node.get_logger().info(bt_fmt("[FindObj] SRM1 payload published for SRM2"))
    node.get_logger().info(bt_fmt(f"[FindObj] target_found = True ({action_name})"))
    return True


def CalculateGoal():
    """
    Analisi posizione pacco / calcolo goal.
    Mock: SUCCESS immediato con side-effect sulla blackboard.
    """
    node = _require_node()
    goal_name = get_bt_attr("goal", "goal")
    node.bb["goal_computed"] = True
    node.bb["last_goal_name"] = goal_name
    node.get_logger().info(bt_fmt(f"[CalculateGoal] goal_computed = True ({goal_name})"))
    return True


def CheckAlignment():
    """
    Verifica allineamento (basi / EE).
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    # Potresti leggere qualcosa dal mondo qui.
    node.get_logger().info(bt_fmt("[CheckAlignment] (mock)"))
    return True


def Set():
    """
    Settaggio stato (es. cambio modalita controller).
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[Set] (mock)"))
    return True


def CloseGripper():
    """
    Chiusura gripper / prese.
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[CloseGripper] (mock)"))
    return True


def SaySomething():
    """
    Nodo "log vocale".
    Qui solo log su console.
    """
    node = _require_node()
    message = get_bt_attr("message", "(mock)")
    node.get_logger().info(bt_fmt(f"[SaySomething] {message}"))

    # Simula passaggio informazione SRM1 -> SRM2 sulla posizione del pallet.
    if get_current_bt_name() == "SRM1" and "pallet" in str(message).lower():
        if not node.bb.get("pallet_info_ready", False):
            _ = node._ensure_mock_pallet_pose()
        node.bb["srm1_data_to_srm2"] = bool(node.bb.get("pallet_info_ready", False))
    return True


def NearObj():
    """
    Condizione "vicino all'oggetto".
    In questa demo viene legata al completamento di ApproachObject.
    """
    node = _require_node()
    tree_name = get_current_bt_name()
    name_attr = str(get_bt_attr("name", "") or "").lower()
    if "srm1" in name_attr:
        key = "SRM1_near_object"
    elif "srm2" in name_attr:
        key = "SRM2_near_object"
    else:
        key = f"{tree_name}_near_object"
    value = bool(node.bb.get(key, False))
    node.get_logger().info(bt_fmt(f"[NearObj] ({key}) = {value}"))
    return value


def DataReceived():
    """
    Condizione "dati ricevuti" (es. SRM2 che riceve info da SRM1).
    Mock: attesa 1 s dopo un eventuale messaggio, poi SUCCESS.
    """
    node = _require_node()
    timer_key = f"{get_current_bt_name()}_DataReceived"
    t0 = node.get_action_timer(timer_key)
    if t0 is None:
        node.get_logger().info(bt_fmt("[DataReceived] waiting SRM1 payload"))
        t0 = node.start_action_timer(timer_key)

    # Attende esplicitamente il "messaggio" condiviso da SRM1 (mock visione).
    if not bool(node.bb.get("srm1_data_to_srm2", False)):
        return None
    node.clear_action_timer(timer_key)
    node.get_logger().info(bt_fmt("[DataReceived] done (SRM1 payload available)"))
    return True


def Controller():
    """
    Nodo generico di controllo (forza, posizione, angoli, ecc.).
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[Controller] (mock)"))
    return True


def CorrectBasePos():
    """
    Correzione posizione basi per allineamento.
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[CorrectBasePos] (mock)"))
    return True


def AdjustPositioning():
    """
    Aggiustamento posizionamento per drop.
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[AdjustPositioning] (mock)"))
    return True


def ReturnToPallet():
    """
    Ritorno al pallet dopo il rilascio.
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[ReturnToPallet] (mock)"))
    return True

