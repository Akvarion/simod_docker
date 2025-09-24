# ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.5, y: 0.0, z: 0.0}}" -1
# sleep 2
# ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
# sleep 2

ros2 topic pub /left/ur_left_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.0, 0.0, 0.0, 0, 0.1, 0.0]}" -1

ros2 topic pub /right/ur_right_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.0, 0.0, 0.0, 0, -0.1, 0.0]}" -1

sleep 4

ros2 topic pub /left/ur_left_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" -1

ros2 topic pub /right/ur_right_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" -1