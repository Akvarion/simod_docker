#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Runtime entrypoint helpers for BT XML demo."""

import os
import time

import rclpy
from behaviortree.tree import Tree
from ament_index_python.packages import get_package_share_directory

from bt_xml_demo.core import BTDemoNode
import bt_xml_demo.actions as actions
from bt_xml_demo.actions import (
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
    ApproachObject,
    LiftObj,
    MoveBase,
    Drop,
    Release,
    assign_bt_name,
)
def main():
    rclpy.init()

    # Nodo ROS
    bt_node = BTDemoNode()
    actions.set_bt_node(bt_node)
    bt_node.bb.setdefault("SRM1_done", False)
    bt_node.bb.setdefault("SRM2_done", False)

    # -------------------------------------------------------------------------
    # Individua i file XML
    # -------------------------------------------------------------------------
    try:
        pkg_share = get_package_share_directory("ps_try")
        xml_base_path = os.path.join(pkg_share, "scripts", "behaviortree_XML_demo")
    except Exception:
        # Path di sviluppo: stessa cartella del file + subdir
        script_dir = os.path.dirname(os.path.abspath(__file__))
        xml_base_path = os.path.abspath(os.path.join(script_dir, "..", "behaviortree_XML_demo"))

    xmlfile_supervisor = os.path.join(xml_base_path, "SRM_Supervisor.xml")
    xmlfile_srm1 = os.path.join(xml_base_path, "SRM1.xml")
    xmlfile_srm2 = os.path.join(xml_base_path, "SRM2.xml")

    for f in (xmlfile_supervisor, xmlfile_srm1, xmlfile_srm2):
        if not os.path.exists(f):
            bt_node.get_logger().error(f"XML non trovato: {f}")
            rclpy.shutdown()
            return

    bt_node.get_logger().info("XML BT trovati, costruzione alberi...")

    # -------------------------------------------------------------------------
    # Crea le Tree e genera dagli XML
    # -------------------------------------------------------------------------
    bt_tree_supervisor = Tree()
    bt_tree_srm1 = Tree()
    bt_tree_srm2 = Tree()

    # Lista completa delle funzioni disponibili (foglie)
    all_funcs = [
        # mock / condition
        Sync, FindObj, CalculateGoal, CheckAlignment, Set,
        CloseGripper, SaySomething, NearObj, DataReceived, Controller,
        CorrectBasePos, AdjustPositioning, ReturnToPallet,
        # azioni di movimento
        ApproachObject, LiftObj, MoveBase, Drop, Release
    ]

    bt_tree_supervisor.gen_tree_from_btcpp(xmlfile_supervisor, all_funcs)
    bt_tree_srm1.gen_tree_from_btcpp(xmlfile_srm1, all_funcs)
    bt_tree_srm2.gen_tree_from_btcpp(xmlfile_srm2, all_funcs)

    assign_bt_name(bt_tree_supervisor, "Supervisor")
    assign_bt_name(bt_tree_srm1, "SRM1")
    assign_bt_name(bt_tree_srm2, "SRM2")

    bt_node.get_logger().info("Behavior Tree caricati dagli XML.")

    # -------------------------------------------------------------------------
    # Loop di esecuzione: tick dei tre alberi in parallelo concettuale
    # -------------------------------------------------------------------------
    rate_hz = 10.0
    tick_dt = 1.0 / rate_hz

    bt_node.get_logger().info("Inizio esecuzione BT (Ctrl+C per uscire).")

    try:
        while rclpy.ok():
            result_srm1 = bt_tree_srm1.run()
            result_srm2 = bt_tree_srm2.run()
            result_supervisor = bt_tree_supervisor.run()
            # result_srm2 = True
            # result_supervisor = True

            if result_srm1 is True and not bt_node.bb.get("SRM1_done", False):
                bt_node.bb["SRM1_done"] = True
                bt_node.get_logger().info("[SRM1] ha completato l'albero (flag bb)")

            if result_srm2 is True and not bt_node.bb.get("SRM2_done", False):
                bt_node.bb["SRM2_done"] = True
                bt_node.get_logger().info("[SRM2] ha completato l'albero (flag bb)")

            # Avanza esecuzione ROS (callback servizi ecc.)
            rclpy.spin_once(bt_node, timeout_sec=0.01)

            # Stop opzionale quando tutti completati
            # (se vuoi, puoi usare una variabile d'ambiente o parametro)
            if result_supervisor is True and result_srm1 is True and result_srm2 is True:
                bt_node.get_logger().info("Tutti i Behavior Tree hanno riportato SUCCESS.")
                break

            time.sleep(tick_dt)

    except KeyboardInterrupt:
        bt_node.get_logger().info("Shutdown richiesto da tastiera.")
    finally:
        if bt_node is not None:
            try:
                bt_node.stop_all_movement()
            except Exception:
                pass
            bt_node.destroy_node()
        actions.set_bt_node(None)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
