#!/usr/bin/env python3
"""
Simple simultaneous base moving demo node for two robots in Gazebo.
- Moves both mobile bases via cmd_vel topics.
Usage: run this script inside a sourced ROS2 workspace (with Gazebo + MoveIt running):
python3 move_demo.py
Configure topics/frames with command-line args if your setup uses different names.
"""

import rclpy
from rclpy.node import Node
import argparse
import time
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
import tf2_ros


class ApproachObjDemo(Node):
    def __init__(self, args):
        super().__init__('move_demo')
        self.declare_parameter('object_name', args.object)
        self.object_name = args.object

        # Topics
        self.left_base_topic  = args.left_base_topic
        self.right_base_topic = args.right_base_topic

        # Publishers
        self.left_base_pub  = self.create_publisher(Twist, self.left_base_topic, 10)
        self.right_base_pub = self.create_publisher(Twist, self.right_base_topic, 10)

        # TF buffer/listener to read end-effector poses
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo set_model_state client
        self.set_model_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        if not self.set_model_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/gazebo/set_model_state service not available. Object attach will fail if not present.')

        # State machine
        self.start_time = time.time()
        self.state = 'approach'
        self.state_start = time.time()

        # Durations (seconds)
        self.durations = {
            'approach': 4.0,
            'descend_and_pick': 3.0,
            'transport': 5.0,
            'release': 1.0
        }

        # Destination velocities
        self.base_linear_speed  = 0.2
        self.base_angular_speed = 0.0

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # When object attached, keep updating its pose to follow EE
        self.attached = False

        self.get_logger().info('PickAndPlaceDemo initialized')

    def control_loop(self):
        now = time.time()
        elapsed = now - self.state_start

        if self.state == 'approach':
            # Move both bases forward
            twist = Twist()
            twist.linear.x  = self.base_linear_speed
            twist.linear.y  = 0.0
            twist.angular.z = 0.0

            # INFO log or Warning
            self.get_logger().info(f'Approaching object {self.object_name}...')
            self.left_base_pub.publish(twist)
            self.right_base_pub.publish(twist)

            if elapsed > self.durations['approach']:
                # stop motion
                self.get_logger().info('Reached approach position, stop and prepare to pick')
                self.stop_all_movement()

                self.get_logger().info('Demo finished')
                # stop timer and optionally shutdown
                self.timer.cancel()
                rclpy.shutdown()

    def stop_all_movement(self):
        twist_zero = Twist()
        self.left_base_pub.publish(twist_zero)
        self.right_base_pub.publish(twist_zero)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', default='pacco_clone_1', help='Gazebo model name of the object to pick')
    parser.add_argument('--left_base_topic', default='/left_summit/cmd_vel')
    parser.add_argument('--right_base_topic', default='/right_summit/cmd_vel')
    args = parser.parse_args()

    rclpy.init()
    node = ApproachObjDemo(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
