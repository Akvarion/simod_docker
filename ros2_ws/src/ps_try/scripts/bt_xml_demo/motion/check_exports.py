#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Guardrail check for bt_action_motion compatibility surface."""

from __future__ import annotations

EXPECTED = {
    "_execute_tp_full_control",
    "_get_live_package_xyz",
    "_get_live_tp_state",
    "_init_tp_arm_joint_stage",
    "_init_tp_base_stage",
    "_pkg_xyz_for_alignment",
    "_resolve_drop_target_xyz",
    "_resolve_pkg_reference_xyz",
    "_ros_now_s",
    "ApproachObject",
    "LiftObj",
    "MoveBase",
    "Drop",
    "Release",
}


def main() -> int:
    import bt_xml_demo.bt_action_motion as m

    missing = sorted(name for name in EXPECTED if not hasattr(m, name))
    if missing:
        print("MISSING_EXPORTS:")
        for name in missing:
            print(name)
        return 1

    print("OK: bt_action_motion compatibility exports present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
