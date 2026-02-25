#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Basic/mock BT actions and conditions."""

from __future__ import annotations

import math

import numpy as np
import rclpy

from bt_xml_demo.bt_action_context import (
    bt_fmt,
    get_bt_attr,
    get_current_bt_name,
    require_node,
)

# Backward-compatible alias used by extracted action bodies.
_require_node = require_node


def _arm_pair_aligned(node) -> bool:
    """
    Allineamento operativo pre-pick della coppia robot.

    Per evitare avanzamenti prematuri del Supervisor (LiftObj avviato mentre
    ApproachObject e' ancora in tracking), qui richiediamo convergenza completa
    dell'approach su entrambi i lati: i flag `SRM*_near_object` devono essere
    entrambi True.
    """
    return bool(node.bb.get("SRM1_near_object", False) and node.bb.get("SRM2_near_object", False))


def _reset_adjust_positioning_runtime(node) -> None:
    """Clear runtime state for AdjustPositioning micro-correction."""
    node.bb.pop("adjust_positioning_stage", None)
    node.bb.pop("adjust_positioning_t0", None)
    node.bb.pop("adjust_positioning_last_err", None)
    node.bb.pop("adjust_positioning_left_target_xy", None)
    node.bb.pop("adjust_positioning_right_target_xy", None)

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
    Verifica allineamento (basi / EE) con semantica minima non-mock.
    Non comanda il robot: espone solo una condizione coerente al BT.
    """
    node = _require_node()
    name_attr = str(get_bt_attr("name", "CheckAlignment") or "").strip().lower()

    # Caso principale usato nel Supervisor prima del pick:
    # richiede che entrambi i robot abbiano completato l'approach.
    if "arm1-arm2" in name_attr:
        ok = _arm_pair_aligned(node)
        node.get_logger().info(
            bt_fmt(
                "[CheckAlignment] ARM1-ARM2 "
                f"ok={ok} "
                f"(SRM1_base_goal_reached={bool(node.bb.get('SRM1_base_goal_reached', False))}, "
                f"SRM2_base_goal_reached={bool(node.bb.get('SRM2_base_goal_reached', False))}, "
                f"SRM1_near_object={bool(node.bb.get('SRM1_near_object', False))}, "
                f"SRM2_near_object={bool(node.bb.get('SRM2_near_object', False))})"
            )
        )
        return ok

    # Verifica allineamento EE in preparazione al pick (Supervisor).
    if name_attr == "ee":
        ok = bool(node.bb.get("SRM1_near_object", False) and node.bb.get("SRM2_near_object", False))
        node.get_logger().info(
            bt_fmt(
                "[CheckAlignment] EE "
                f"ok={ok} "
                f"(SRM1_base_goal_reached={bool(node.bb.get('SRM1_base_goal_reached', False))}, "
                f"SRM2_base_goal_reached={bool(node.bb.get('SRM2_base_goal_reached', False))}, "
                f"SRM1_near_object={bool(node.bb.get('SRM1_near_object', False))}, "
                f"SRM2_near_object={bool(node.bb.get('SRM2_near_object', False))})"
            )
        )
        return ok

    # Verifica pre-drop strict: movebase completato + pre-pose validata.
    if "inapositiontodrop" in name_attr:
        ok = bool(node.bb.get("movebase_done", False)) and bool(node.bb.get("drop_prepose_ok", False))
        node.get_logger().info(
            bt_fmt(
                "[CheckAlignment] InAPositionToDrop "
                f"ok={ok} "
                f"(movebase_done={bool(node.bb.get('movebase_done', False))}, "
                f"drop_prepose_ok={bool(node.bb.get('drop_prepose_ok', False))})"
            )
        )
        return ok

    # In questa demo gli altri check restano permissivi (non bloccano il flusso).
    node.get_logger().info(bt_fmt(f"[CheckAlignment] permissive check ({name_attr or 'generic'})"))
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
    In questa fase non genera comandi diretti: attende che i due BT locali
    completino l'approach e segnino i flag di allineamento.
    """
    node = _require_node()
    timer_key = f"{get_current_bt_name()}_CorrectBasePos"
    t0 = node.get_action_timer(timer_key)
    if t0 is None:
        t0 = node.start_action_timer(timer_key)
        node.get_logger().info(bt_fmt("[CorrectBasePos] waiting SRM1/SRM2 alignment"))

    if _arm_pair_aligned(node):
        node.clear_action_timer(timer_key)
        node.get_logger().info(bt_fmt("[CorrectBasePos] alignment satisfied"))
        return True

    elapsed = node.get_clock().now().nanoseconds / 1e9 - t0
    timeout_s = float(getattr(getattr(node, "cfg", object()), "correct_base_pos_timeout_s", 0.0) or 0.0)
    # Default: timeout disabled (0.0) -> gate remains blocking until true alignment.
    # This avoids premature progression to LiftObj when SRM1/SRM2 are still far.
    if timeout_s > 0.0 and elapsed >= timeout_s:
        node.clear_action_timer(timer_key)
        node.get_logger().warn(
            bt_fmt(
                "[CorrectBasePos] timeout waiting alignment, continuing "
                f"(elapsed={elapsed:.2f}s)"
            )
        )
        return True

    return None


def AdjustPositioning():
    """
    Micro-correzione pre-drop:
    - valida idoneita' geometrica rispetto al target di deposito
    - se non idoneo, esegue breve correzione TP (base-only o base+arm lock)
    - ritorna SUCCESS solo su convergenza (blocking di default)
    """
    node = _require_node()
    man_cfg = node.cfg.manipulation
    if not bool(getattr(man_cfg, "adjust_positioning_enable", True)):
        node.bb["drop_prepose_ok"] = True
        node.get_logger().info(bt_fmt("[AdjustPositioning] disabled, bypass"))
        return True

    # Lazy import to avoid circular dependency at module import time.
    from bt_xml_demo.bt_action_motion import (
        _execute_tp_full_control,
        _get_live_package_xyz,
        _get_live_tp_state,
        _init_tp_arm_joint_stage,
        _init_tp_base_stage,
        _pkg_xyz_for_alignment,
        _resolve_drop_target_xyz,
        _resolve_pkg_reference_xyz,
        _ros_now_s,
    )

    t0 = node.get_action_timer("AdjustPositioning")
    if t0 is None:
        t0 = node.start_action_timer("AdjustPositioning")
        node.bb["drop_prepose_ok"] = False
        _reset_adjust_positioning_runtime(node)
        node.bb["adjust_positioning_stage"] = "evaluate"
        node.bb["adjust_positioning_t0"] = float(_ros_now_s(node))
        node.get_logger().info(bt_fmt("[AdjustPositioning] start micro-correction"))

    live_state = _get_live_tp_state(node)
    if live_state is None:
        node._warn_throttled(
            "adjust_positioning_wait_data",
            bt_fmt("[AdjustPositioning] waiting sensor data (joint_states/odom)"),
            period_s=1.0,
        )
        rclpy.spin_once(node, timeout_sec=0.01)
        return None
    left_arm_jp, right_arm_jp, left_base, right_base = live_state

    drop_target_xyz = node.bb.get("drop_target_xyz", None)
    if not (isinstance(drop_target_xyz, (list, tuple)) and len(drop_target_xyz) >= 3):
        drop_target_xyz, drop_src = _resolve_drop_target_xyz(node)
        if isinstance(drop_target_xyz, (list, tuple)) and len(drop_target_xyz) >= 3:
            node.bb["drop_target_xyz"] = [float(drop_target_xyz[0]), float(drop_target_xyz[1]), float(drop_target_xyz[2])]
            node.bb["drop_target_source"] = str(drop_src)
        else:
            node._warn_throttled(
                "adjust_positioning_wait_target",
                bt_fmt("[AdjustPositioning] waiting valid drop target"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None

    # Current package reference pose for error metrics.
    pkg_xyz = _pkg_xyz_for_alignment(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
    )
    if pkg_xyz is None:
        pkg_xyz = _get_live_package_xyz(node)
    if pkg_xyz is None:
        pkg_xyz = _resolve_pkg_reference_xyz(
            node,
            left_arm_jp=left_arm_jp,
            right_arm_jp=right_arm_jp,
            left_base=left_base,
            right_base=right_base,
        )

    xy_tol = float(getattr(man_cfg, "adjust_positioning_xy_tol", 0.10))
    z_tol = float(getattr(man_cfg, "adjust_positioning_z_tol", 0.04))
    # In this BT stage we mainly do planar micro-corrections. Keep Z gating optional
    # to avoid deadlock when vertical convergence is delegated to Drop/descend.
    require_z_gate = bool(getattr(man_cfg, "adjust_positioning_require_z", False))
    ee_dist_tol = float(getattr(man_cfg, "adjust_positioning_ee_dist_tol", 0.05))
    base_pair_tol = float(getattr(man_cfg, "adjust_positioning_base_pair_tol", 0.10))
    release_z_offset = float(getattr(man_cfg, "drop_release_z_offset", 0.30))
    z_target = float(drop_target_xyz[2]) + release_z_offset

    pkg_xy_err = float("inf")
    pkg_z_err = float("inf")
    if isinstance(pkg_xyz, (list, tuple)) and len(pkg_xyz) >= 3:
        pkg_xy_err = float(
            math.hypot(
                float(pkg_xyz[0]) - float(drop_target_xyz[0]),
                float(pkg_xyz[1]) - float(drop_target_xyz[1]),
            )
        )
        pkg_z_err = abs(float(pkg_xyz[2]) - z_target)

    # Pair geometry from bases (used only for base formation consistency).
    pair_dx = float(right_base[0]) - float(left_base[0])
    pair_dy = float(right_base[1]) - float(left_base[1])
    pair_norm = float(math.hypot(pair_dx, pair_dy))
    nominal_pair = node.bb.get("drop_base_pair_xy", None)
    if not (isinstance(nominal_pair, (list, tuple)) and len(nominal_pair) >= 2):
        nominal_pair = [pair_dx, pair_dy]
        node.bb["drop_base_pair_xy"] = [pair_dx, pair_dy]
    nominal_pair_norm = float(math.hypot(float(nominal_pair[0]), float(nominal_pair[1])))
    base_pair_err = abs(pair_norm - nominal_pair_norm)

    # EE spacing error must be computed from live end-effector poses, not base spacing.
    nominal_dist = float(node._pkg_hold_nominal_dist) if getattr(node, "_pkg_hold_nominal_dist", None) is not None else float("nan")
    ee_dist_err = 0.0
    if np.isfinite(nominal_dist):
        ee_dist = float("nan")
        try:
            ee_live = node._get_live_ee_by_side(
                left_arm_jp=left_arm_jp,
                right_arm_jp=right_arm_jp,
                left_base=left_base,
                right_base=right_base,
            )
            l_ee = ee_live.get("left", None)
            r_ee = ee_live.get("right", None)
            if isinstance(l_ee, np.ndarray) and isinstance(r_ee, np.ndarray) and l_ee.shape[0] >= 3 and r_ee.shape[0] >= 3:
                ee_dist = float(np.linalg.norm(np.asarray(l_ee[:3], dtype=np.float32) - np.asarray(r_ee[:3], dtype=np.float32)))
        except Exception:
            ee_dist = float("nan")
        if np.isfinite(ee_dist):
            ee_dist_err = abs(float(ee_dist) - float(nominal_dist))
        else:
            # Conservative fallback if EE pose is unavailable.
            ee_dist_err = abs(pair_norm - nominal_dist)

    z_gate_ok = bool(pkg_z_err <= z_tol) if require_z_gate else True
    ready = bool(
        pkg_xy_err <= xy_tol
        and z_gate_ok
        and ee_dist_err <= ee_dist_tol
        and base_pair_err <= base_pair_tol
    )
    node.bb["adjust_positioning_last_err"] = {
        "pkg_xy_err": float(pkg_xy_err),
        "pkg_z_err": float(pkg_z_err),
        "ee_dist_err": float(ee_dist_err),
        "base_pair_err": float(base_pair_err),
        "z_gate_required": bool(require_z_gate),
        "z_gate_ok": bool(z_gate_ok),
    }

    if ready:
        node.bb["drop_prepose_ok"] = True
        node.clear_action_timer("AdjustPositioning")
        _reset_adjust_positioning_runtime(node)
        node.stop_all_movement()
        node.get_logger().info(
            bt_fmt(
                "[AdjustPositioning] pre-drop pose ready "
                f"(xy_err={pkg_xy_err:.3f}, z_err={pkg_z_err:.3f}, "
                f"ee_dist_err={ee_dist_err:.3f}, base_pair_err={base_pair_err:.3f})"
            )
        )
        return True

    # Build one micro-correction TP stage (single init, then track until converged).
    stage = str(node.bb.get("adjust_positioning_stage", "evaluate")).strip().lower()
    if stage == "evaluate":
        # Translate both bases by the package->target displacement (not base-center->target),
        # so the held package is driven toward the drop target directly.
        if isinstance(pkg_xyz, (list, tuple)) and len(pkg_xyz) >= 2:
            dx = float(drop_target_xyz[0]) - float(pkg_xyz[0])
            dy = float(drop_target_xyz[1]) - float(pkg_xyz[1])
        else:
            center_x = 0.5 * (float(left_base[0]) + float(right_base[0]))
            center_y = 0.5 * (float(left_base[1]) + float(right_base[1]))
            dx = float(drop_target_xyz[0]) - center_x
            dy = float(drop_target_xyz[1]) - center_y
        left_target_xy = [float(left_base[0]) + dx, float(left_base[1]) + dy]
        right_target_xy = [float(right_base[0]) + dx, float(right_base[1]) + dy]
        node.bb["adjust_positioning_left_target_xy"] = left_target_xy
        node.bb["adjust_positioning_right_target_xy"] = right_target_xy

        period_s = float(max(getattr(man_cfg, "adjust_positioning_traj_time", 1.5), 0.2))
        kp_xy = float(getattr(man_cfg, "drop_base_kp_xy", 1.20))
        ok_base = _init_tp_base_stage(
            node,
            left_base_goal_xy=left_target_xy,
            right_base_goal_xy=right_target_xy,
            period_s=period_s,
            kp_xy=kp_xy,
            kp_yaw=0.0,
            arm_active=bool(node.bb.get("package_attached", False)),
        )
        ok_arm = True
        if bool(node.bb.get("package_attached", False)):
            ok_arm = _init_tp_arm_joint_stage(
                node,
                left_arm_goal=np.asarray(left_arm_jp, dtype=np.float32),
                right_arm_goal=np.asarray(right_arm_jp, dtype=np.float32),
                period_s=period_s,
                kp_arm=float(getattr(man_cfg, "transport_lock_arm_kp", node.approach_jtc_arm_kp)),
            )
            if bool(ok_base and ok_arm) and (node.approach_jtc_task is not None):
                try:
                    node.approach_jtc_task.activate()
                    node.approach_jtc_task.set_activation("base", True)
                    node.approach_jtc_task.set_activation("arm", True)
                except Exception:
                    pass

        if not bool(ok_base and ok_arm):
            node._warn_throttled(
                "adjust_positioning_tp_init_fail",
                bt_fmt("[AdjustPositioning] TP init failed, retrying"),
                period_s=1.0,
            )
            rclpy.spin_once(node, timeout_sec=0.01)
            return None
        node.bb["adjust_positioning_stage"] = "track"

    _execute_tp_full_control(
        node,
        left_arm_jp=left_arm_jp,
        right_arm_jp=right_arm_jp,
        left_base=left_base,
        right_base=right_base,
        arm_clip_abs=float(getattr(man_cfg, "hold_arm_cmd_abs_max", node.tp_arm_cmd_abs_max)),
        base_xy_abs_max=float(getattr(man_cfg, "drop_base_cmd_xy_abs_max", 0.12)),
        base_wz_abs_max=float(node.tp_base_cmd_wz_abs_max),
    )

    node._info_throttled(
        "adjust_positioning_track",
        bt_fmt(
            "[AdjustPositioning] tracking "
            f"xy_err={pkg_xy_err:.3f}/{xy_tol:.3f}, "
            f"z_err={pkg_z_err:.3f}/{z_tol:.3f}, "
            f"ee_dist_err={ee_dist_err:.3f}/{ee_dist_tol:.3f}, "
            f"base_pair_err={base_pair_err:.3f}/{base_pair_tol:.3f}"
        ),
        period_s=1.0,
    )

    timeout_s = float(getattr(man_cfg, "adjust_positioning_timeout_s", 0.0))
    elapsed = _ros_now_s(node) - float(node.bb.get("adjust_positioning_t0", _ros_now_s(node)))
    if timeout_s > 0.0 and elapsed >= timeout_s:
        node.stop_all_movement()
        node.clear_action_timer("AdjustPositioning")
        _reset_adjust_positioning_runtime(node)
        if bool(getattr(man_cfg, "adjust_positioning_soft_continue", False)):
            node.bb["drop_prepose_ok"] = True
            node.get_logger().warn(
                bt_fmt(
                    "[AdjustPositioning] timeout reached, soft-continue enabled "
                    f"(elapsed={elapsed:.2f}s)"
                )
            )
            return True

        node.bb["drop_prepose_ok"] = False
        node.get_logger().warn(
            bt_fmt(
                "[AdjustPositioning] timeout reached, keeping blocking mode "
                f"(elapsed={elapsed:.2f}s)"
            )
        )
        t0 = node.start_action_timer("AdjustPositioning")
        node.bb["adjust_positioning_t0"] = float(t0)
        node.bb["adjust_positioning_stage"] = "evaluate"

    rclpy.spin_once(node, timeout_sec=0.01)
    return None


def ReturnToPallet():
    """
    Ritorno al pallet dopo il rilascio.
    Mock: SUCCESS immediato.
    """
    node = _require_node()
    node.get_logger().info(bt_fmt("[ReturnToPallet] (mock)"))
    return True
