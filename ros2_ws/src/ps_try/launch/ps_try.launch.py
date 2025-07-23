#!/usr/bin/env python3


from launch import LaunchDescription, LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
import launch_ros.parameter_descriptions
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction


def generate_launch_description():
        # Planning scene monitor
    planning_scene_monitor_parameters = {'publish_planning_scene': True,
                                         'publish_geometry_updates': True,
                                         'publish_state_updates': True,
                                         'publish_transforms_updates': True}

    moveit_config = (
        MoveItConfigsBuilder("dual",package_name='srm_simod_moveit_config')
        .robot_description(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/srm.urdf.xacro")
        .robot_description_semantic(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/dual.srdf")
        .robot_description_kinematics(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .trajectory_execution(file_path=get_package_share_directory('ur')+"/config/lr_controller.yaml")
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .sensors_3d(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/sensors_3d.yaml")
        .joint_limits(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )
    # Demo node
    ps_demo = Node(
        package="ps_try",
        executable="ps_try",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ],
    )

    return LaunchDescription([ps_demo])