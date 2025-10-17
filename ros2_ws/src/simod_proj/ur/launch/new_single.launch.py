#!/usr/bin/env python3
import traceback
import pprint
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
import launch_ros.parameter_descriptions
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction, RegisterEventHandler
import subprocess
import os

import xacro

def generate_launch_description():
    robot_ = DeclareLaunchArgument('robot', default_value='left', description='L/R Robot')

    return LaunchDescription([robot_, OpaqueFunction(function=launch_setup)])
    
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError as e:
        print(f"[ERROR] EnvironmentError in load_file: {e}")
        traceback.print_exc()
        return []


def getRobotOffset(arg):
    if "right" in arg:
        return '0.66'
    else:
        return '-0.66'

# CONFIGURATION_TRANSLATION = {
#     'ur_lift': ['ur_left_shoulder_lift_joint'],
#     'ur_wrist_yaw': ['ur_left_wrist_3_joint'],
#     'base': ['position/x', 'position/theta'],
#     'ur': ['ur_left_shoulder_pan_joint', 'ur_left_shoulder_lift_joint', 'ur_left_elbow_joint', 'ur_left_wrist_1_joint', 'ur_left_wrist_2_joint', 'ur_left_wrist_3_joint']
# }

    
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError as e:
        print(f"[ERROR] EnvironmentError in load_yaml: {e}")
        traceback.print_exc()
        return []

# def load_joint_limits_from_config(mode='default'):
#     """Translate the values from the robot configuration to params to be used by MoveIt."""
#     params = {'joint_limits': {}}

#     with open('/path/to/robot_joint_limits.yaml') as f:
#         robot_params = yaml.safe_load(f)
#     try:
#         for config_name, joint_names in CONFIGURATION_TRANSLATION.items():
#             config = robot_params[config_name]
#             config_limits = config['motion'].get(mode, {})
#             result = {}
#             for name, abrev in [('velocity', 'vel'), ('acceleration', 'accel')]:
#                 if abrev in config_limits:
#                     result[f'has_{name}_limits'] = True
#                     result[f'max_{name}'] = config_limits[abrev]
#                 elif f'{abrev}_m' in config_limits:
#                     result[f'has_{name}_limits'] = True
#                     result[f'max_{name}'] = config_limits[f'{abrev}_m']
#                 else:
#                     result[f'has_{name}_limits'] = False
#             for joint_name in joint_names:
#                 params['joint_limits'][joint_name] = dict(result)
#     except (KeyError, ModuleNotFoundError):
#         # We may reach here if HELLO_FLEET_ID or HELLO_FLEET_PATH is not set
#         # or stretch_body.device is not on the PYTHONPATH
#         # in which case we load the defaults
#         print('Load from default')
#         return load_yaml('stretch_moveit_config', 'config/default_joint_limits.yaml')
#     return params

def launch_setup(context, *args, **kwargs):
    
    robot_ = LaunchConfiguration('robot').perform(context)
    # Load urdf file
    with open("/ros2_ws/"+robot_+"_resolved_paletta.urdf","r") as robot_description_file:
        robot_description_content = robot_description_file.read()
        robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)
    

    # Publish robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_,
        name="robot_state_publisher",
        output="both",
        parameters=[{'use_sim_time': True},{'robot_description': robot_description_param}],
    )

    # static_tf_real = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",

    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=[(xyz_base.split()[0]+'"').replace('"', ''), ('"'+xyz_base.split()[1]+'"').replace('"', ''), ('"'+xyz_base.split()[2]).replace('"', ''), "0", "0", "0", "1", parent_base, base_prefix+"_odom"],
    # )
    static_tf_wheel_back_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=robot_,
        name=robot_+"_static_transform_publisher_back_left",
        output="log",
        arguments=["0.25", "0.15", "0", "0", "0", "0", "1", robot_+"_summit_base_link", robot_+"_summit_back_left_wheel_link"],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_wheel_back_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=robot_,
        name=robot_+"_static_transform_publisher_back_right",
        output="log",
        arguments=["0.25", "-0.15", "0", "0", "0", "0", "1", robot_+"_summit_base_link", robot_+"_summit_back_right_wheel_link"],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_wheel_front_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=robot_,
        name=robot_+"_static_transform_publisher_front_left",
        output="log",
        arguments=["-0.25", "0.15", "0", "0", "0", "0", "1", robot_+"_summit_base_link", robot_+"_summit_front_left_wheel_link"],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_wheel_front_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=robot_,
        name=robot_+"_static_transform_publisher_front_right",
        output="log",
        arguments=["-0.25", "-0.15", "0", "0", "0", "0", "1", robot_+"_summit_base_link", robot_+"_summit_front_right_wheel_link"],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_,
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', robot_+'_summit_odom'],
        parameters=[{'use_sim_time': True}],
    )

    # static_tf_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', '1', 'left_robot', "left_summit_odom"]
    # )
    static_tf_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=robot_,
        name="static_transform_publisher",
        output="log",
        arguments=['0', '0', '0', "0", "0", "0", "1", robot_+'_summit_odom', robot_+'_summit_base_footprint'],
        parameters=[{'use_sim_time': True}],
    )

    delayed_static_tf_world = TimerAction(
        period=2.0,  # 2 seconds delay to allow spawns and such
        actions=[static_tf_world],
    )
    delayed_static_tf_odom= TimerAction(
        period=2.25,  # 2 seconds delay to allow spawns and such
        actions=[static_tf_odom],
    )

    # delayed_static_tf_base = TimerAction(
    #     period=2.5,  # 2.5 seconds delay to allow spawns and such
    #     actions=[static_tf_base],
    # )

    # Spawn from topic                                                      
    gazebo_spawn_robot_description = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
                    '-entity', robot_+'_robot',
                    '-topic', '/'+robot_+'/robot_description',
                    '-x', getRobotOffset(robot_),
                    '-y', '0',
                    '-z', '0',
                    '-timeout', '5',
                    '-unpause'
                    ]
    )

    velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        #namespace=robot_,
        arguments=["ur_" + robot_ + "_joint_group_vel_controller", '-c', '/' + robot_ + '/controller_manager'],
        output='screen',
    )

    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_,
        arguments=['joint_state_broadcaster', '-c', '/' + robot_ + '/controller_manager', "--controller-manager-timeout", "10"],
        output='screen',
    )

    real_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ur"), "config", "/ros2_ws/src/simod_proj/ur/config/" + robot_ + "_controller.yaml"]
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            {'robot_description': robot_description_param},
            launch_ros.parameter_descriptions.ParameterFile(real_joint_controllers, allow_substs=True),
        ],
        output="screen",
    )
       
    # planning_context
    # file = open(namespace+'_resolved.urdf.xacro','w')
    # file.write(robot_description_content.perform(context))   
    # file.close()

    gazebo_path = os.path.join(get_package_share_directory('ur'), 'xacro','gazebo.world')

    gazebo = IncludeLaunchDescription(
                 (os.path.join(get_package_share_directory('gazebo_ros'), 'launch','gazebo.launch.py')),
                 launch_arguments={'physics': 'False', 'world': gazebo_path}.items(),
                 )
    
  
    # ##############
    # # MOVEIT STUFF
    # ##############
    #     # Planning scene monitor
    # planning_scene_monitor_parameters = {'publish_planning_scene': True,
    #                                      'publish_geometry_updates': True,
    #                                      'publish_state_updates': True,
    #                                      'publish_transforms_updates': True,
    #                                      }
    # planning_pipelines_config = {
    #     "default_planning_pipeline": "ompl",
    #     "planning_pipelines": ["ompl"],
    #     "ompl": {
    #         "planning_plugins": ["ompl_interface/OMPLPlanner"],
    #         "request_adapters": [
    #             "default_planner_request_adapters/ResolveConstraintFrames",
    #             "default_planner_request_adapters/FixWorkspaceBounds",
    #             "default_planner_request_adapters/FixStartStateBounds",
    #             "default_planner_request_adapters/FixStartStateCollision",
    #         ],
    #         "response_adapters": [
    #             "default_planner_response_adapters/AddTimeOptimalParameterization",
    #             "default_planner_response_adapters/ValidateSolution",
    #         ],
    #     },
    # }
    # ompl_planning_yaml = load_yaml("left_srm_simod_moveit_config", "config/ompl_planning.yaml")
    # planning_pipelines_config["ompl"].update(ompl_planning_yaml)
    # # parameters_pipeline={
    # #     "default_planning_pipeline": "ompl",
    # #     "planning_pipelines": ["ompl"],
    # #     "ompl": {
    # #         "planning_plugins": ["ompl_interface/OMPLPlanner"],
    # #         "planner_configs": {
    # #             "RRTConnect": {
    # #                 "type": "geometric::RRTConnect",
    # #                 "range": 0.0
    # #             }
    # #         }
    # #     }   
    # # }

    # # Trajectory control
    # controllers_yaml = load_yaml("left_srm_simod_moveit_config", "config/moveit_controllers.yaml")

    # moveit_controllers_config = {'moveit_simple_controller_manager': controllers_yaml,
    #                       'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    
    # trajectory_execution_config = {'moveit_manage_controllers': True,
    #                         'trajectory_execution.allowed_execution_duration_scaling': 1.2,
    #                         'trajectory_execution.allowed_goal_duration_margin': 0.5,
    #                         'trajectory_execution.allowed_start_tolerance': 0.01}


    # # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    # move_group_capabilities = {
    #     "capabilities": "move_group/ExecuteTaskSolutionCapability"
    # }
    # robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('ur'),
    #                                                            'xacro',
    #                                                            'left_resolved_paletta.urdf'),
    #                                             )
    # robot_description = {'robot_description': robot_description_config.toxml()}
    # robot_description_semantic_config = load_file('left_srm_simod_moveit_config', 'config/dual.srdf')
    # robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # kinematics_yaml = load_yaml('left_srm_simod_moveit_config', 'config/kinematics.yaml')
    # robot_description_kinematics = {
    #     "robot_description_kinematics": kinematics_yaml
    # }

    # sensors_yaml = load_yaml('left_srm_simod_moveit_config', 'config/sensors_3d.yaml')
    # joint_limits_yaml = load_yaml('left_srm_simod_moveit_config', 'config/joint_limits.yaml')
    # #Start the actual move_group node/action server
    # #moveit_config = MoveItConfigsBuilder("left_srm_simod").to_dict()

    # run_move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     #namespace=namespace,
    #     output="screen",
    #     parameters=[
    #         {'use_sim_time': True},
    #         {"tf_buffer_cache_time": 30.0},
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #         sensors_yaml,
    #         joint_limits_yaml,
    #         planning_pipelines_config,
    #         trajectory_execution_config,
    #         moveit_controllers_config,
    #         planning_scene_monitor_parameters
    #     ],
    # )
    # rviz_config_file = (
    #     os.path.join(get_package_share_directory('ur'), 'rviz','rviz_config.rviz')
    # )
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     # arguments=['-d', rviz_config_file],
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #         planning_pipelines_config,
    #         {'use_sim_time': True},
    #     ],        
    # )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        namespace=robot_,
        parameters=[
            {'use_sim_time': True},
            get_package_share_directory(robot_+'_srm_simod_moveit_config')+"/config/ros2_controllers.yaml"],
        output='screen',
        
    )
    delayed_controller_manager = TimerAction(
        period=3.0,  # 3 seconds delay to allow spawns and such
        actions=[controller_manager],
    )
    
    # register_handler = RegisterEventHandler(
    #     event_handler = OnProcessExit(
    #         target_action=gazebo_spawn_robot_description,
    #         on_exit=TimerAction(period=2.0, actions=[run_move_group_node]),
    #     )
    # )

    # delayed_rviz_node = TimerAction(
    #     period=10.0,  # 10 seconds delay to allow spawns and such
    #     actions=[rviz],
    # )
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=2.0,  # 2 seconds delay to allow spawns and such
        actions=[joint_state_broadcaster_spawner],
    )


    gazebo_spawn_robot_description_delayed = TimerAction(
        period=4.0,  # 4 seconds delay to allow gazebo to start
        actions=[gazebo_spawn_robot_description],
    )
    
    return [
    #    register_handler,
    #    gazebo,
        gazebo_spawn_robot_description_delayed,
        robot_state_publisher_node,
    #   static_tf_real,
        # joint_state_publisher_node
        static_tf_wheel_back_left,
        static_tf_wheel_back_right,
        static_tf_wheel_front_left,
        static_tf_wheel_front_right,
        delayed_static_tf_world,
        # delayed_static_tf_odom,
        #static_tf_right_world,
        #static_tf_right,
    #    delayed_static_tf_base,
    #   ur_control_node is creating conflict. Gazebo can work without it.
    #   ur_control_node,
    #   ros2_control_node,
        controller_manager,
        delayed_joint_state_broadcaster_spawner,
        velocity_controller,
    #    delayed_rviz_node,
        #delayed_move_group,
    ]

    