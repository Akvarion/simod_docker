#!/usr/bin/env python3
"""
Simple simultaneous pick-and-place demo node for two robots in Gazebo.
- Moves both mobile bases via cmd_vel topics.
- Sends simple velocity commands to arm joint_group_vel_controller topics.
- At pick, calls /gazebo/set_model_state to snap the object to the end-effector (simulated attach).

Usage: run this script inside a sourced ROS2 workspace (with Gazebo + MoveIt running):
python3 pick_and_place_demo.py --object box --left_ee ur_left_ee_link --right_ee ur_right_ee_link

Configure topics/frames with command-line args if your setup uses different names.
"""

import rclpy
from rclpy.node import Node
import argparse
import time
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import tf2_ros


class PickAndPlaceDemo(Node):
    def __init__(self, args):
        super().__init__('pick_and_place_demo')
        self.declare_parameter('object_name', args.object)
        self.object_name = args.object

        # Topics
        self.left_base_topic  = args.left_base_topic
        self.right_base_topic = args.right_base_topic
        self.left_arm_topic   = args.left_arm_topic
        self.right_arm_topic  = args.right_arm_topic

        # End-effector frames
        self.left_ee  = args.left_ee
        self.right_ee = args.right_ee

        # Publishers
        self.left_base_pub  = self.create_publisher(Twist, self.left_base_topic, 10)
        self.right_base_pub = self.create_publisher(Twist, self.right_base_topic, 10)
        self.left_arm_pub   = self.create_publisher(Float64MultiArray, self.left_arm_topic, 10)
        self.right_arm_pub  = self.create_publisher(Float64MultiArray, self.right_arm_topic, 10)

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
        self.arm_velocity       = 0.05  # generic joint velocity command magnitude

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

            # Small arm motions to prepare
            arm_msg = Float64MultiArray()
            arm_msg.data = [self.arm_velocity] * 6
            self.get_logger().info('Moving arms to approach pose...')
            self.left_arm_pub.publish(arm_msg)
            self.right_arm_pub.publish(arm_msg)

            if elapsed > self.durations['approach']:
                # stop motion
                self.get_logger().info('Reached approach position, stop and prepare to pick')
                self.stop_all_movement()
                self.transition('descend_and_pick')

        elif self.state == 'descend_and_pick':
            # Move arms to pick pose via small velocity commands
            arm_pick = Float64MultiArray()
            arm_pick.data = [0.02, -0.02, 0.01, 0.0, 0.0, 0.0]
            self.left_arm_pub.publish(arm_pick)
            self.right_arm_pub.publish(arm_pick)

            if elapsed > self.durations['descend_and_pick']:
                # Stop arms and attach object to left robot end-effector
                self.stop_all_movement()
                attached = self.attach_object_to_ee(self.object_name, self.left_ee)
                if attached:
                    self.attached = True
                    self.get_logger().info(f'Object {self.object_name} attached to {self.left_ee}')
                else:
                    self.get_logger().warn('Attach failed; continuing without attach')
                self.transition('transport')

        elif self.state == 'transport':
            # Move both bases to transport location (simultaneous)
            twist_left = Twist()
            twist_right = Twist()
            twist_left.linear.x = 0.0
            twist_left.linear.y = 0.1
            twist_right.linear.x = 0.0
            twist_right.linear.y = -0.1
            self.left_base_pub.publish(twist_left)
            self.right_base_pub.publish(twist_right)

            # If attached, keep updating object pose to follow left end-effector
            if self.attached:
                self.update_attached_object_pose(self.object_name, self.left_ee)

            if elapsed > self.durations['transport']:
                self.stop_all_movement()
                self.transition('release')

        elif self.state == 'release':
            # Stop sending attached updates; call set_model_state once to place object
            if self.attached:
                self.update_attached_object_pose(self.object_name, self.left_ee)
                self.attached = False
                self.get_logger().info('Released object')
            if elapsed > self.durations['release']:
                self.get_logger().info('Demo finished')
                # stop timer and optionally shutdown
                self.timer.cancel()

    def transition(self, new_state):
        self.state = new_state
        self.state_start = time.time()
        self.get_logger().info(f'Transition to {new_state}')

    def stop_all_movement(self):
        twist_zero = Twist()
        self.left_base_pub.publish(twist_zero)
        self.right_base_pub.publish(twist_zero)
        arm_zero = Float64MultiArray()
        arm_zero.data = [0.0] * 6
        self.left_arm_pub.publish(arm_zero)
        self.right_arm_pub.publish(arm_zero)

    def attach_object_to_ee(self, model_name, ee_frame):
        # Try to lookup EE pose
        try:
            trans = self.tf_buffer.lookup_transform('world', ee_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f'Failed to get transform for {ee_frame}: {e}')
            return False

        pose = Pose()
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z
        pose.orientation = trans.transform.rotation

        return self.call_set_model_state(model_name, pose, reference_frame='world')

    def update_attached_object_pose(self, model_name, ee_frame):
        try:
            trans = self.tf_buffer.lookup_transform('world', ee_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f'Failed to get transform for {ee_frame}: {e}')
            return False

        pose = Pose()
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z
        pose.orientation = trans.transform.rotation

        return self.call_set_model_state(model_name, pose, reference_frame='world')

    def call_set_model_state(self, model_name, pose: Pose, reference_frame='world'):
        if not self.set_model_state_client.service_is_ready():
            self.get_logger().warn('SetModelState service not ready')
            return False

        req = SetModelState.Request()
        state = ModelState()
        state.model_name = model_name
        state.pose = pose
        state.reference_frame = reference_frame
        req.model_state = state

        future = self.set_model_state_client.call_async(req)
        # don't block long; just schedule
        # but wait a short while for result
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception:
            pass
        if future.done() and future.result() is not None:
            res = future.result()
            return True
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', default='pacco_clone_1', help='Gazebo model name of the object to pick')
    parser.add_argument('--left_base_topic', default='/left_summit/cmd_vel')
    parser.add_argument('--right_base_topic', default='/right_summit/cmd_vel')
    parser.add_argument('--left_arm_topic', default='/left/ur_left_joint_group_vel_controller/commands')
    parser.add_argument('--right_arm_topic', default='/right/ur_right_joint_group_vel_controller/commands')
    parser.add_argument('--left_ee', default='ur_left_ee_link')
    parser.add_argument('--right_ee', default='ur_right_ee_link')
    args = parser.parse_args()

    rclpy.init()
    node = PickAndPlaceDemo(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
