#!/usr/bin/env bash
sleep 5
# 1) Muovi il right_summit
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' -1


ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' -1
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' -1
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' -1

# attendo 4 secondi
sleep 1

ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1

ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1

ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
  
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1

# 2) Ferma il right_summit
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1

# attendo 1 secondo
sleep 1

# 3) Muovi il left_summit
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' -1
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' -1
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' -1
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' -1

sleep 1

ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1

sleep 1

# 4) Ferma il left_summit
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
