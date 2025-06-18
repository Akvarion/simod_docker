from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("srm_simod").to_dict()

    # Demo node
    ps_demo = Node(
        package="ps_try",
        executable="ps_try",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([ps_demo])