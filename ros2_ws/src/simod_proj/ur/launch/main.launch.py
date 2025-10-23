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
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
import traceback, yaml, xacro

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



    rviz                 = 'True'
    gazebo_path = os.path.join(get_package_share_directory('ur'), 'xacro','gazebo.world')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'physics': 'False', 'world': gazebo_path}.items()
    )

    #Open gazebo and spawn robots
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "xacro", "srm.urdf.xacro"]),
        ]
    )
    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

    robot_spawner_left = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'ur', 'new_single.launch.py', 'robot:=left'
        ],
        name='left_robot_spawner',
        output='screen'
    )

    robot_spawner_right = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'ur', 'new_single.launch.py', 'robot:=right'
        ],
        name='right_robot_spawner',
        output='screen'
    )

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
    # planning_pipelines_config = {
    #     "default_planning_pipeline": "ompl",
    #     "planning_pipelines": ["ompl"],
    #     "ompl": {
    #         "planning_plugins": ["ompl_interface/OMPLPlanner"],
    #         "request_adapters": [
    #             "default_planning_request_adapters/ResolveConstraintFrames",
    #             "default_planning_request_adapters/ValidateWorkspaceBounds",
    #             "default_planning_request_adapters/CheckStartStateBounds",
    #             "default_planning_request_adapters/CheckStartStateCollision",
    #         ],
    #         "response_adapters": [
    #             "default_planning_response_adapters/AddTimeOptimalParameterization",
    #             "default_planning_response_adapters/ValidateSolution",
    #         ],
    #     },
    # }
    # # Planning scene monitor
    # planning_scene_monitor_parameters = {'publish_planning_scene': True,
    #                                      'publish_geometry_updates': True,
    #                                      'publish_state_updates': True,
    #                                      'publish_transforms_updates': True}

    # moveit_config = (
    #     MoveItConfigsBuilder("dual",package_name='srm_simod_moveit_config')
    #     .robot_description(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/srm.urdf.xacro")
    #     .robot_description_semantic(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/dual.srdf")
    #     .robot_description_kinematics(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/kinematics.yaml")
    #     # .planning_pipelines(
    #     #     pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
    #     # )
    #     # .trajectory_execution(file_path=get_package_share_directory('ur')+"/config/lr_controller.yaml")
    #     # .planning_scene_monitor(planning_scene_monitor_parameters)
    #     # .sensors_3d(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/sensors_3d.yaml")
    #     # .joint_limits(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/joint_limits.yaml")
    #     # .pilz_cartesian_limits(file_path=get_package_share_directory('srm_simod_moveit_config')+"/config/pilz_cartesian_limits.yaml")
    #     .to_moveit_configs()
    # )

    # # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    # move_group_capabilities = {
    #     "capabilities": "move_group/ExecuteTaskSolutionCapability"
    # }

    #Start the actual move_group node/action server
    # run_move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     #namespace=namespace,
    #     output="screen",
    #     parameters=[
    #         moveit_config.to_dict(),
    #         move_group_capabilities,
    #         {'moveit_controller_manager': 'moveit_ros_control_interface/MoveItControllerManager'},
    #         {'robot_description': robot_description_param},
    #         {'publish_planning_scene': True}
    #     ],
    # )
    
    ##############
    # MOVEIT STUFF
    ##############
        # Planning scene monitor
    planning_scene_monitor_parameters = {'publish_planning_scene': True,
                                         'publish_geometry_updates': True,
                                         'publish_state_updates': True,
                                         'publish_transforms_updates': True,
                                         }
    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planner_request_adapters/ResolveConstraintFrames",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
            ],
            "response_adapters": [
                "default_planner_response_adapters/AddTimeOptimalParameterization",
                "default_planner_response_adapters/ValidateSolution",
            ],
        },
    }
    ompl_planning_yaml = load_yaml("left_srm_simod_moveit_config", "config/ompl_planning.yaml")
    planning_pipelines_config["ompl"].update(ompl_planning_yaml)
    # parameters_pipeline={
    #     "default_planning_pipeline": "ompl",
    #     "planning_pipelines": ["ompl"],
    #     "ompl": {
    #         "planning_plugins": ["ompl_interface/OMPLPlanner"],
    #         "planner_configs": {
    #             "RRTConnect": {
    #                 "type": "geometric::RRTConnect",
    #                 "range": 0.0
    #             }
    #         }
    #     }   
    # }

    # Trajectory control
    controllers_yaml = load_yaml("left_srm_simod_moveit_config", "config/moveit_controllers.yaml")

    moveit_controllers_config = {'moveit_simple_controller_manager': controllers_yaml,
                          'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    
    trajectory_execution_config = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}


    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }
    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('ur'),
                                                               'xacro',
                                                               'left_resolved_paletta.urdf'),
                                                )
    robot_description = {'robot_description': robot_description_config.toxml()}
    robot_description_semantic_config = load_file('left_srm_simod_moveit_config', 'config/dual.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('left_srm_simod_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_yaml
    }

    sensors_yaml = load_yaml('left_srm_simod_moveit_config', 'config/sensors_3d.yaml')
    joint_limits_yaml = load_yaml('left_srm_simod_moveit_config', 'config/joint_limits.yaml')
    #Start the actual move_group node/action server
    #moveit_config = MoveItConfigsBuilder("left_srm_simod").to_dict()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        #namespace=namespace,
        output="screen",
        parameters=[
            {'use_sim_time': True},
            {"tf_buffer_cache_time": 30.0},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            sensors_yaml,
            joint_limits_yaml,
            planning_pipelines_config,
            trajectory_execution_config,
            moveit_controllers_config,
            planning_scene_monitor_parameters
        ],
    )

    ##############
    # RVIZ
    ##############

    dual_robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('ur'),
                                                               'xacro',
                                                               'srm.urdf.xacro'),
                                                )

    dual_robot_description = {'robot_description': dual_robot_description_config.toxml()}

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
            {'use_sim_time': True},
            dual_robot_description
        ],        
    )
    ##############
    # ADDITIONAL NODES
    ##############
    # Merge joint states from both robots
    joint_state_merger = Node(
        package="ur",
        executable="joint_state_merger.py",
        name="joint_state_merger",
        output="screen"
    )
    # Sync Gazebo scene with MoveIt planning scene
    gazebo_scene_sync = Node(
        package="ur",
        executable="gazebo_scene_sync.py",
        name="gazebo_scene_sync",
        output="screen"
    )
    ##############
    # Action Server for MoveIt to control the robots in Gazebo
    ##############

    simod_mac_node = Node(
        package="simod_moveit_action_controller",
        executable="simod_mac",
        name="simod_moveit_action_controller",
        output="screen"
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

    delayed_moveit_bridge = Node(
        package="ur",
        executable="gazebo_moveit_bridge.py",
        name="gazebo_moveit_bridge",
        output="screen"
    )


    ##############
    # HANDLERS/TIMERS
    ##############


    right_spawner_handler = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action=robot_spawner_left,
            on_start=TimerAction(period=10.0, actions=[robot_spawner_right]),
        )
    )
    rviz_handler = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action=robot_spawner_right,
            on_start=TimerAction(period=10.0, actions=[rviz]),
        )
    )
    
    move_group_handler = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action=rviz,
            on_start=TimerAction(period=8.0, actions=[run_move_group_node]),
        )
    )
    
    action_controller_handler = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action=run_move_group_node,
            on_start=simod_mac_node
        )
    )
    
    
    # Add a delay to allow the robots to spawn before merging joint states
    joint_state_merger_handler = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action=robot_spawner_right,
            on_start=TimerAction(period=5.0, actions=[joint_state_merger]),
        )
    )
    # Add a delay to allow the robots to spawn before syncing the planning scene
    gazebo_scene_sync_handler = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action=rviz,
            on_start=TimerAction(period=4.0, actions=[gazebo_scene_sync]),
        )
    )

    ld = LaunchDescription()

    ld.add_action(gazebo)
   #ld.add_action(joint_pub_ros1)
    ld.add_action(robot_spawner_left)
    ld.add_action(right_spawner_handler)
    ld.add_action(rviz_handler)
    ld.add_action(run_move_group_node)
    ld.add_action(action_controller_handler)
   # ld.add_action(move_group_handler)
    ld.add_action(joint_state_merger_handler)
    ld.add_action(gazebo_scene_sync_handler)
   # ld.add_action(delayed_moveit_bridge)

    return ld