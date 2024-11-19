#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
import launch_ros.parameter_descriptions
from launch.actions import OpaqueFunction

def generate_launch_description():
   
    # # General arguments
    namespace = DeclareLaunchArgument('namespace')
    description_package = DeclareLaunchArgument('description_package', default_value="base")
    description_file = DeclareLaunchArgument('description_file', default_value="base_spawn.urdf.xacro")
    spawn_gazebo_base = DeclareLaunchArgument('spawn_gazebo_base', default_value='""')
    parent_base = DeclareLaunchArgument('parent_base', default_value='""')
  
  
    return LaunchDescription(

        [namespace, description_package, description_file, spawn_gazebo_base, parent_base,
        OpaqueFunction(function=launch_setup) ]
        ) 

def launch_setup(context, *args, **kwargs):

    # # General arguments
    description_package = LaunchConfiguration("description_package").perform(context)
    description_file    = LaunchConfiguration("description_file").perform(context)
    namespace           = LaunchConfiguration("namespace").perform(context)
    parent_base         = LaunchConfiguration("parent_base").perform(context)
    spawn_gazebo_base   = LaunchConfiguration("spawn_gazebo_base").perform(context)

    namespaced_robot_description = ['/', namespace, '/robot_description']

    # #Robot description
    robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([FindPackageShare(description_package), "xacro", description_file]),
    " ",
    "namespace:=",
    namespace, 
    " ",
    "parent_base:=",
    parent_base, 
    " ",
    "sim_gazebo:=",
    spawn_gazebo_base, 
    " ",
    "publish_bf:=",
    "true",
    " ",
    "hq:=",
    "true",
    " ",    
  ])
    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)
  
    # Publish robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{
        'robot_description': robot_description_param,
        'publish_frequency': 100.0,
        }],
    )

    joint_pub_ros1 = Node(
        package="py_pubsub",
        executable="ros1_bridge",
        output="both",
        )

    joint_publisher = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace=namespace,
            remappings=[('joint_states', '/joint_states')],
            )

    # print("gggggg")
    # Spawn from topic                                                      
    gazebo_spawn_base_description = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', namespaced_robot_description,
                    '-entity', namespace + '_base',
                    '-timeout', '5',
                    '-unpause'
                    ],
        condition=IfCondition(LaunchConfiguration('spawn_gazebo_base'))
    )


    return [
       
        robot_state_publisher_node,
        joint_publisher,
        #joint_pub_ros1,
        gazebo_spawn_base_description,    
     
    ]

