#!/usr/bin/env python3

from launch import LaunchDescription, LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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

    gazebo_path = os.path.join(get_package_share_directory('ur'), 'xacro','gazebo.world')

    gazebo = IncludeLaunchDescription(
                 (os.path.join(get_package_share_directory('gazebo_ros'), 'launch','gazebo.launch.py')),
                 launch_arguments={'physics': 'False', 'world': gazebo_path}.items(),
                 condition=IfCondition(gazebo))
    
    
    #Open gazebo and spawn robots
  
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
    
    joint_pub_ros1 = Node(
        package="py_pubsub",
        executable="ros1_bridge",
        output="screen",
        arguments=["-model_name_right", "right_robot",
                   "-model_name_left", "left_robot",
                   ],
        condition=IfCondition(spawn_gazebo_base)
        )

 
    ld = LaunchDescription()
    ld.add_action(gazebo)
    #ld.add_action(joint_pub_ros1)
    ld.add_action(robot_spawner_left)
    ld.add_action(robot_spawner_right)
    
    return ld