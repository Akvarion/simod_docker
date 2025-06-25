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

    description_package = "ur"

    ur_type_left             = 'ur5e'
    namespace_left           = 'left'
    tf_prefix_left           = 'ur_left_'
    xyz_base_left            = '"-0.66 0 0"'      #no base
    xyz_left                 = '"0 0 0.543"'
    rpy_left                 = '"0 -1.57 0"'
    finger_tip_cor_left      = '"0.157 0 0.0018"'
    robot_ip_left            = "192.168.0.102"
    parent_hand_left         = tf_prefix_left + 'tool0'
    parent_ur_left           = 'left_summit_base_footprint' #'world'
    initial_positions_file_l = 'initial_positions_left'
    physical_parameters_l    = 'physical_parameters_left'
    left_controller          = 'left_controller.yaml'
    vel_controller_left      = 'ur_left_joint_group_vel_controller'

    ur_type_right            = 'ur5e'
    tf_prefix_right          = 'ur_right_'
    namespace_right          = 'right'
    xyz_base_right           = '"0.66 0 0"'
    xyz_right                = '"0 0 0.543"'
    rpy_right                = '"0 -1.57 0"'
    finger_tip_cor_right     = '"0.174 0 0.0018"'
    robot_ip_right           = "192.168.0.103"
    parent_hand_right        = tf_prefix_right + 'tool0'
    parent_ur_right          = 'right_summit_base_footprint' #'world'
    initial_positions_file_r = 'initial_positions_right'
    physical_parameters_r    = 'physical_parameters_right'
    right_controller         = 'right_controller.yaml'
    vel_controller_right     = 'ur_right_joint_group_vel_controller'



    hardware_interface   = 'hardware_interface/PositionJointInterface'


    gazebo               = 'True'
    spawn_gazebo_robot   = 'True'
    spawn_gazebo_base    = 'True'
    rviz                 = 'True'
    gazebo_path = os.path.join(get_package_share_directory('ur'), 'xacro','gazebo.world')

    gazebo = IncludeLaunchDescription(
                 (os.path.join(get_package_share_directory('gazebo_ros'), 'launch','gazebo.launch.py')),
                 launch_arguments={'physics': 'False', 'world': gazebo_path}.items(),
                 condition=IfCondition(gazebo))
    

    #Open gazebo and spawn robots
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "xacro", "srm.urdf.xacro"]),
        ]
    )
    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

    robot_spawner_left = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([get_package_share_directory(description_package), '/launch', '/single_robot.launch.py']),
                        launch_arguments={
                            'spawn_gazebo_robot': spawn_gazebo_robot,
                            'ur_type': ur_type_left, 
                            'namespace': namespace_left,
                            'tf_prefix': tf_prefix_left,
                            'base_prefix': 'left_summit',
                            'parent_ur': parent_ur_left,
                            'parent_hand': parent_hand_left,
                            'xyz': xyz_left,
                            'rpy_base_ur': '"0 0 -1.57"',
                            'xyz_base': xyz_base_left,
                            'rpy': rpy_left,
                            'finger_tip_cor': finger_tip_cor_left,
                            'hardware_interface': hardware_interface,
                            'robot_ip': robot_ip_left,
                            'initial_positions_file': initial_positions_file_l,
                            'physical_parameters_file': physical_parameters_l,
                            'controller_file': left_controller,
                            'vel_controller': vel_controller_left,
                            'spawn_gazebo_base': spawn_gazebo_base,
                            'parent_base': 'world',
                            'script_sender_port': '50002',
                            'reverse_port': '50005',
                            'script_command_port': '50008',
                            'trajectory_port': '50007',
                            }.items())
    
    robot_spawner_right = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([get_package_share_directory(description_package), '/launch', '/single_robot.launch.py']),
                        launch_arguments={
                            'spawn_gazebo_robot': spawn_gazebo_robot,
                            'ur_type': ur_type_right, 
                            'namespace': namespace_right,
                            'tf_prefix': tf_prefix_right,
                            'base_prefix': 'right_summit',
                            'parent_ur': parent_ur_right,
                            'parent_hand': parent_hand_right,
                            'xyz': xyz_right,
                            'rpy_base_ur': '"0 0 1.57"',
                            'xyz_base': xyz_base_right,
                            'rpy': rpy_right,
                            'finger_tip_cor': finger_tip_cor_right,
                            'hardware_interface': hardware_interface,
                            'robot_ip': robot_ip_right,
                            'initial_positions_file': initial_positions_file_r,
                            'physical_parameters_file': physical_parameters_r,
                            'controller_file': right_controller,
                            'vel_controller': vel_controller_right,
                            'spawn_gazebo_base': spawn_gazebo_base,
                            'parent_base': 'world',
                            'script_sender_port': '50006',
                            'reverse_port': '50001',
                            'script_command_port': '50004',
                            'trajectory_port': '50003',
                            }.items())
    
    # joint_pub_ros1 = Node(
    #     package="py_pubsub",
    #     executable="ros1_bridge",
    #     output="screen",
    #     arguments=["-model_name_right", "right_robot",
    #                "-model_name_left", "left_robot",
    #                ],
    #     condition=IfCondition(spawn_gazebo_base)
    #     )

    # Planning Functionality
    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
            ],
        },
    }
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

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    #Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        #namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {'moveit_controller_manager': 'moveit_ros_control_interface/MoveItControllerManager'},
            {'robot_description': robot_description_param},
            {'publish_planning_scene': True}
        ],
    )
    
    rviz_config_file = (
        os.path.join(get_package_share_directory('ur'), 'rviz','rviz_config.rviz')
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],        
    )

    joint_state_merger = Node(
        package="ur",
        executable="joint_state_merger.py",
        name="joint_state_merger",
        output="screen"
    )

    delayed_joint_state_merger = TimerAction(
        period=8.0, #10 seconds delay to allow spawns and such
        actions=[joint_state_merger],
    )

    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     name='controller_manager',
    #     parameters=[
    #         moveit_config.robot_description,
    #         get_package_share_directory('ur')+"/config/lr_controller.yaml"
    #         ],
    #     output='screen',
    # )

    # delayed_controller_manager = TimerAction(
    #     period=4.0,  # 10 seconds delay to allow spawns and such
    #     actions=[controller_manager],
    # )
    gazebo_moveit_bridge = Node(
        package="ur",
        executable="gazebo_moveit_bridge.py",
        name="gazebo_moveit_bridge",
        output="screen"
    )
    delayed_moveit_bridge = TimerAction(
        period=5.0,  # 10 seconds delay to allow spawns and such
        actions=[gazebo_moveit_bridge],
    )

    ld = LaunchDescription()
    ld.add_action(gazebo)
   #ld.add_action(joint_pub_ros1)
    ld.add_action(robot_spawner_left)
    ld.add_action(robot_spawner_right)
    ld.add_action(delayed_joint_state_merger)
    ld.add_action(delayed_moveit_bridge)
    ld.add_action(rviz)
    ld.add_action(run_move_group_node)
    return ld