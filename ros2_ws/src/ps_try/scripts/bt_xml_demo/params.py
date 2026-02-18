#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import yaml


def _warn(logger, msg: str):
    if logger is not None:
        try:
            logger.warn(msg)
            return
        except Exception:
            pass
    print(msg)


def resolve_demo_params_path(pkg_share: Optional[str] = None, explicit_path: Optional[str] = None) -> Optional[str]:
    """Resolve bt_demo_params.yaml path without env-driven overrides.

    Priority:
    1) explicit_path argument
    2) package share path
    3) source-tree relative path
    4) container default path
    """
    here = os.path.abspath(os.path.dirname(__file__))
    if not pkg_share:
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("ps_try")
        except Exception:
            pkg_share = None

    candidates = [
        explicit_path or "",
        os.path.join(pkg_share, "config", "bt_demo_params.yaml") if pkg_share else "",
        os.path.abspath(os.path.join(here, "..", "..", "config", "bt_demo_params.yaml")),
        "/ros2_ws/src/ps_try/config/bt_demo_params.yaml",
    ]

    seen = set()
    for c in candidates:
        if not c:
            continue
        p = os.path.abspath(c)
        if p in seen:
            continue
        seen.add(p)
        if os.path.isfile(p):
            return p
    return None


def load_demo_params(
    pkg_share: Optional[str] = None,
    explicit_path: Optional[str] = None,
    logger=None,
) -> Tuple[Dict[str, Any], Optional[str]]:
    path = resolve_demo_params_path(pkg_share=pkg_share, explicit_path=explicit_path)
    if path is None:
        return {}, None

    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh) or {}
        if not isinstance(raw, dict):
            _warn(logger, f"[bt_demo_params] Invalid YAML root in '{path}', expected map.")
            return {}, path
        return raw, path
    except Exception as exc:
        _warn(logger, f"[bt_demo_params] Failed to load '{path}': {exc}")
        return {}, path


def get_path(data: Dict[str, Any], dotted_key: str, default: Any) -> Any:
    cur: Any = data
    for tok in dotted_key.split("."):
        if not isinstance(cur, dict) or tok not in cur:
            return default
        cur = cur[tok]
    return cur


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _as_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        txt = value.strip().lower()
        if txt in ("1", "true", "yes", "on"):
            return True
        if txt in ("0", "false", "no", "off"):
            return False
    return bool(default)


def _as_float_list(value: Any, expected_len: int, default: List[float]) -> List[float]:
    if not isinstance(value, (list, tuple)) or len(value) != expected_len:
        return [float(v) for v in default]
    out: List[float] = []
    try:
        for v in value:
            out.append(float(v))
    except Exception:
        return [float(v) for v in default]
    return out


@dataclass
class PhasesConfig:
    approach_time: float = 31.8
    descend_and_pick_time: float = 20.5
    collect_time: float = 10.0
    transport_time: float = 25.0
    descend_and_place_time: float = 12.0
    release_time: float = 1.0


@dataclass
class MotionProfilesConfig:
    left_arm_vel: List[float] = field(default_factory=lambda: [0.075, -0.01, 0.01, -0.01, -0.01, -0.01])
    right_arm_vel: List[float] = field(default_factory=lambda: [-0.09, -0.01, 0.01, -0.01, 0.01, 0.021])
    left_arm_pick: List[float] = field(default_factory=lambda: [-0.001, -0.08, 0.08, 0.0, -0.12, -0.055])
    right_arm_pick: List[float] = field(default_factory=lambda: [0.001, -0.12, 0.08, 0.0, 0.12, 0.055])
    collect_left_base_xy_vel: List[float] = field(default_factory=lambda: [0.001, -0.15])
    collect_right_base_xy_vel: List[float] = field(default_factory=lambda: [-0.0, -0.15])
    left_arm_collect: List[float] = field(default_factory=lambda: [0.0, 0.08, -0.08, 0.0, 0.0, -0.020])
    right_arm_collect: List[float] = field(default_factory=lambda: [0.0, 0.08, -0.08, 0.0, 0.0, -0.025])
    left_transport_vel_xy: List[float] = field(default_factory=lambda: [-0.24, 0.0])
    right_transport_vel_xy: List[float] = field(default_factory=lambda: [-0.24, -0.055])
    place_left_base_xy_vel: List[float] = field(default_factory=lambda: [0.0, 0.09])
    place_right_base_xy_vel: List[float] = field(default_factory=lambda: [0.0, 0.06])
    left_arm_place: List[float] = field(default_factory=lambda: [-0.001, -0.12, 0.08, 0.0, 0.0, 0.05])
    right_arm_place: List[float] = field(default_factory=lambda: [0.001, -0.12, 0.08, 0.0, -0.0, 0.0])
    left_arm_release: List[float] = field(default_factory=lambda: [0.01, 0.0, 0.0, 0.0, 0.1, 0.0])
    right_arm_release: List[float] = field(default_factory=lambda: [-0.01, 0.0, 0.0, 0.0, -0.1, 0.0])


@dataclass
class PackageConfig:
    link_name: str = "pacco_clone_1::link_1"


@dataclass
class TPViewerConfig:
    enable: bool = True
    host: str = "127.0.0.1"
    port: int = 55001
    hz: float = 60.0
    autostart: bool = True


@dataclass
class TPConfig:
    robot_cfg_path: str = ""
    ee_cfg_path: str = ""
    viewer: TPViewerConfig = field(default_factory=TPViewerConfig)
    cmd_min_period: float = 0.03


@dataclass
class ApproachConfig:
    test_duration: float = 31.8
    test_traj_time: float = 31.8
    arm_cmd_abs_max: float = 0.25
    base_cmd_xy_abs_max: float = 0.12
    base_cmd_wz_abs_max: float = 0.20
    base_cmd_ramp_time: float = 1.0
    base_goal_tol: float = 0.08
    base_kp_x: float = 0.9
    base_kp_y: float = 0.9
    mock_pallet_forward: float = 1.20
    mock_pallet_standoff: float = 0.85
    mock_lateral_half: float = 0.35
    base_left_offset_x: float = -0.50
    base_left_offset_y: float = -0.50
    base_right_offset_x: float = 0.50
    base_right_offset_y: float = -0.50
    ee_left_offset_x: float = -0.35
    ee_left_offset_y: float = -0.10
    ee_right_offset_x: float = 0.35
    ee_right_offset_y: float = -0.10
    ee_offset_z: float = 0.40
    ee_orient_mode: str = "keep"
    ee_left_rpy: str = ""
    ee_right_rpy: str = ""
    mock_pallet_model: str = "pacco_clone_1,pacco,pallet2"
    mock_world_file: str = ""
    left_base_model: str = "left_srm,left_summit_xls"
    right_base_model: str = "right_srm,right_summit_xls"
    base_pose_source: str = "aligned_odom"
    base_keep_lane: bool = False
    force_full_2d: bool = True
    base_auto_swap_slots: bool = True
    base_swap_hyst: float = 0.10
    base_align_yaw: bool = False
    arm_profile_scale: float = 0.18
    arm_max_delta: float = 0.60
    base_target_mode: str = "pallet_offset"
    arm_target_mode: str = "pallet_offset"
    arm_joint_tol: float = 0.10
    arm_ee_pos_tol: float = 0.03
    min_exec_time: float = 2.0
    goal_cache_file: str = "/tmp/simod_approach_goal_joints.json"
    arm_delay_enable: bool = True
    arm_enable_dist: float = 2.0
    arm_home_traj_time: float = 4.0
    arm_enable_traj_time: float = 8.0
    left_arm_home: str = ""
    right_arm_home: str = ""
    left_arm_goal: str = ""
    right_arm_goal: str = ""
    mock_pallet_x: str = ""
    mock_pallet_y: str = ""
    mock_pallet_z: str = ""
    approach_only: bool = True
    task_mode: str = "hybrid"
    estimated_timeout: float = 2.0
    jtc_base_kp_xy: float = 1.2
    jtc_base_kp_yaw: float = 1.0
    jtc_arm_kp: float = 1.3
    use_base: bool = True
    base_ctrl: str = "tp"
    goal_cache_enabled: bool = False


@dataclass
class DemoConfig:
    phases: PhasesConfig = field(default_factory=PhasesConfig)
    motion_profiles: MotionProfilesConfig = field(default_factory=MotionProfilesConfig)
    package: PackageConfig = field(default_factory=PackageConfig)
    approach: ApproachConfig = field(default_factory=ApproachConfig)
    tp: TPConfig = field(default_factory=TPConfig)


def _parse_phases(raw: Dict[str, Any]) -> PhasesConfig:
    defaults = PhasesConfig()
    return PhasesConfig(
        approach_time=_as_float(raw.get("approach_time"), defaults.approach_time),
        descend_and_pick_time=_as_float(raw.get("descend_and_pick_time"), defaults.descend_and_pick_time),
        collect_time=_as_float(raw.get("collect_time"), defaults.collect_time),
        transport_time=_as_float(raw.get("transport_time"), defaults.transport_time),
        descend_and_place_time=_as_float(raw.get("descend_and_place_time"), defaults.descend_and_place_time),
        release_time=_as_float(raw.get("release_time"), defaults.release_time),
    )


def _parse_motion_profiles(raw: Dict[str, Any]) -> MotionProfilesConfig:
    defaults = MotionProfilesConfig()
    return MotionProfilesConfig(
        left_arm_vel=_as_float_list(raw.get("left_arm_vel"), 6, defaults.left_arm_vel),
        right_arm_vel=_as_float_list(raw.get("right_arm_vel"), 6, defaults.right_arm_vel),
        left_arm_pick=_as_float_list(raw.get("left_arm_pick"), 6, defaults.left_arm_pick),
        right_arm_pick=_as_float_list(raw.get("right_arm_pick"), 6, defaults.right_arm_pick),
        collect_left_base_xy_vel=_as_float_list(raw.get("collect_left_base_xy_vel"), 2, defaults.collect_left_base_xy_vel),
        collect_right_base_xy_vel=_as_float_list(raw.get("collect_right_base_xy_vel"), 2, defaults.collect_right_base_xy_vel),
        left_arm_collect=_as_float_list(raw.get("left_arm_collect"), 6, defaults.left_arm_collect),
        right_arm_collect=_as_float_list(raw.get("right_arm_collect"), 6, defaults.right_arm_collect),
        left_transport_vel_xy=_as_float_list(raw.get("left_transport_vel_xy"), 2, defaults.left_transport_vel_xy),
        right_transport_vel_xy=_as_float_list(raw.get("right_transport_vel_xy"), 2, defaults.right_transport_vel_xy),
        place_left_base_xy_vel=_as_float_list(raw.get("place_left_base_xy_vel"), 2, defaults.place_left_base_xy_vel),
        place_right_base_xy_vel=_as_float_list(raw.get("place_right_base_xy_vel"), 2, defaults.place_right_base_xy_vel),
        left_arm_place=_as_float_list(raw.get("left_arm_place"), 6, defaults.left_arm_place),
        right_arm_place=_as_float_list(raw.get("right_arm_place"), 6, defaults.right_arm_place),
        left_arm_release=_as_float_list(raw.get("left_arm_release"), 6, defaults.left_arm_release),
        right_arm_release=_as_float_list(raw.get("right_arm_release"), 6, defaults.right_arm_release),
    )


def _parse_package(raw: Dict[str, Any]) -> PackageConfig:
    defaults = PackageConfig()
    return PackageConfig(link_name=str(raw.get("link_name", defaults.link_name)))


def _parse_tp(raw: Dict[str, Any]) -> TPConfig:
    defaults = TPConfig()
    viewer_raw = raw.get("viewer", {}) if isinstance(raw.get("viewer", {}), dict) else {}
    viewer_defaults = defaults.viewer
    viewer = TPViewerConfig(
        enable=_as_bool(viewer_raw.get("enable"), viewer_defaults.enable),
        host=str(viewer_raw.get("host", viewer_defaults.host)),
        port=int(_as_float(viewer_raw.get("port"), float(viewer_defaults.port))),
        hz=_as_float(viewer_raw.get("hz"), viewer_defaults.hz),
        autostart=_as_bool(viewer_raw.get("autostart"), viewer_defaults.autostart),
    )
    return TPConfig(
        robot_cfg_path=str(raw.get("robot_cfg_path", defaults.robot_cfg_path)),
        ee_cfg_path=str(raw.get("ee_cfg_path", defaults.ee_cfg_path)),
        viewer=viewer,
        cmd_min_period=_as_float(raw.get("cmd_min_period"), defaults.cmd_min_period),
    )


def _parse_approach(raw: Dict[str, Any], phases: PhasesConfig) -> ApproachConfig:
    defaults = ApproachConfig(
        test_duration=phases.approach_time,
        test_traj_time=phases.approach_time,
    )
    return ApproachConfig(
        test_duration=_as_float(raw.get("test_duration"), defaults.test_duration),
        test_traj_time=_as_float(raw.get("test_traj_time"), defaults.test_traj_time),
        arm_cmd_abs_max=_as_float(raw.get("arm_cmd_abs_max"), defaults.arm_cmd_abs_max),
        base_cmd_xy_abs_max=_as_float(raw.get("base_cmd_xy_abs_max"), defaults.base_cmd_xy_abs_max),
        base_cmd_wz_abs_max=_as_float(raw.get("base_cmd_wz_abs_max"), defaults.base_cmd_wz_abs_max),
        base_cmd_ramp_time=_as_float(raw.get("base_cmd_ramp_time"), defaults.base_cmd_ramp_time),
        base_goal_tol=_as_float(raw.get("base_goal_tol"), defaults.base_goal_tol),
        base_kp_x=_as_float(raw.get("base_kp_x"), defaults.base_kp_x),
        base_kp_y=_as_float(raw.get("base_kp_y"), defaults.base_kp_y),
        mock_pallet_forward=_as_float(raw.get("mock_pallet_forward"), defaults.mock_pallet_forward),
        mock_pallet_standoff=_as_float(raw.get("mock_pallet_standoff"), defaults.mock_pallet_standoff),
        mock_lateral_half=_as_float(raw.get("mock_lateral_half"), defaults.mock_lateral_half),
        base_left_offset_x=_as_float(raw.get("base_left_offset_x"), defaults.base_left_offset_x),
        base_left_offset_y=_as_float(raw.get("base_left_offset_y"), defaults.base_left_offset_y),
        base_right_offset_x=_as_float(raw.get("base_right_offset_x"), defaults.base_right_offset_x),
        base_right_offset_y=_as_float(raw.get("base_right_offset_y"), defaults.base_right_offset_y),
        ee_left_offset_x=_as_float(raw.get("ee_left_offset_x"), defaults.ee_left_offset_x),
        ee_left_offset_y=_as_float(raw.get("ee_left_offset_y"), defaults.ee_left_offset_y),
        ee_right_offset_x=_as_float(raw.get("ee_right_offset_x"), defaults.ee_right_offset_x),
        ee_right_offset_y=_as_float(raw.get("ee_right_offset_y"), defaults.ee_right_offset_y),
        ee_offset_z=_as_float(raw.get("ee_offset_z"), defaults.ee_offset_z),
        ee_orient_mode=str(raw.get("ee_orient_mode", defaults.ee_orient_mode)),
        ee_left_rpy=str(raw.get("ee_left_rpy", defaults.ee_left_rpy)),
        ee_right_rpy=str(raw.get("ee_right_rpy", defaults.ee_right_rpy)),
        mock_pallet_model=str(raw.get("mock_pallet_model", defaults.mock_pallet_model)),
        mock_world_file=str(raw.get("mock_world_file", defaults.mock_world_file)),
        left_base_model=str(raw.get("left_base_model", defaults.left_base_model)),
        right_base_model=str(raw.get("right_base_model", defaults.right_base_model)),
        base_pose_source=str(raw.get("base_pose_source", defaults.base_pose_source)),
        base_keep_lane=_as_bool(raw.get("base_keep_lane"), defaults.base_keep_lane),
        force_full_2d=_as_bool(raw.get("force_full_2d"), defaults.force_full_2d),
        base_auto_swap_slots=_as_bool(raw.get("base_auto_swap_slots"), defaults.base_auto_swap_slots),
        base_swap_hyst=_as_float(raw.get("base_swap_hyst"), defaults.base_swap_hyst),
        base_align_yaw=_as_bool(raw.get("base_align_yaw"), defaults.base_align_yaw),
        arm_profile_scale=_as_float(raw.get("arm_profile_scale"), defaults.arm_profile_scale),
        arm_max_delta=_as_float(raw.get("arm_max_delta"), defaults.arm_max_delta),
        base_target_mode=str(raw.get("base_target_mode", defaults.base_target_mode)),
        arm_target_mode=str(raw.get("arm_target_mode", defaults.arm_target_mode)),
        arm_joint_tol=_as_float(raw.get("arm_joint_tol"), defaults.arm_joint_tol),
        arm_ee_pos_tol=_as_float(raw.get("arm_ee_pos_tol"), defaults.arm_ee_pos_tol),
        min_exec_time=_as_float(raw.get("min_exec_time"), defaults.min_exec_time),
        goal_cache_file=str(raw.get("goal_cache_file", defaults.goal_cache_file)),
        arm_delay_enable=_as_bool(raw.get("arm_delay_enable"), defaults.arm_delay_enable),
        arm_enable_dist=_as_float(raw.get("arm_enable_dist"), defaults.arm_enable_dist),
        arm_home_traj_time=_as_float(raw.get("arm_home_traj_time"), defaults.arm_home_traj_time),
        arm_enable_traj_time=_as_float(raw.get("arm_enable_traj_time"), defaults.arm_enable_traj_time),
        left_arm_home=str(raw.get("left_arm_home", defaults.left_arm_home)),
        right_arm_home=str(raw.get("right_arm_home", defaults.right_arm_home)),
        left_arm_goal=str(raw.get("left_arm_goal", defaults.left_arm_goal)),
        right_arm_goal=str(raw.get("right_arm_goal", defaults.right_arm_goal)),
        mock_pallet_x=str(raw.get("mock_pallet_x", defaults.mock_pallet_x)),
        mock_pallet_y=str(raw.get("mock_pallet_y", defaults.mock_pallet_y)),
        mock_pallet_z=str(raw.get("mock_pallet_z", defaults.mock_pallet_z)),
        approach_only=_as_bool(raw.get("approach_only"), defaults.approach_only),
        task_mode=str(raw.get("task_mode", defaults.task_mode)),
        estimated_timeout=_as_float(raw.get("estimated_timeout"), defaults.estimated_timeout),
        jtc_base_kp_xy=_as_float(raw.get("jtc_base_kp_xy"), defaults.jtc_base_kp_xy),
        jtc_base_kp_yaw=_as_float(raw.get("jtc_base_kp_yaw"), defaults.jtc_base_kp_yaw),
        jtc_arm_kp=_as_float(raw.get("jtc_arm_kp"), defaults.jtc_arm_kp),
        use_base=_as_bool(raw.get("use_base"), defaults.use_base),
        base_ctrl=str(raw.get("base_ctrl", defaults.base_ctrl)),
        goal_cache_enabled=_as_bool(raw.get("goal_cache_enabled"), defaults.goal_cache_enabled),
    )


def load_demo_config(
    pkg_share: Optional[str] = None,
    explicit_path: Optional[str] = None,
    logger=None,
) -> Tuple[DemoConfig, Optional[str]]:
    raw, path = load_demo_params(pkg_share=pkg_share, explicit_path=explicit_path, logger=logger)
    if not isinstance(raw, dict):
        return DemoConfig(), path

    phases = _parse_phases(raw.get("phases", {}) if isinstance(raw.get("phases", {}), dict) else {})
    motion_profiles = _parse_motion_profiles(
        raw.get("motion_profiles", {}) if isinstance(raw.get("motion_profiles", {}), dict) else {}
    )
    package = _parse_package(raw.get("package", {}) if isinstance(raw.get("package", {}), dict) else {})
    approach = _parse_approach(raw.get("approach", {}) if isinstance(raw.get("approach", {}), dict) else {}, phases)
    tp = _parse_tp(raw.get("tp", {}) if isinstance(raw.get("tp", {}), dict) else {})

    return DemoConfig(
        phases=phases,
        motion_profiles=motion_profiles,
        package=package,
        approach=approach,
        tp=tp,
    ), path
