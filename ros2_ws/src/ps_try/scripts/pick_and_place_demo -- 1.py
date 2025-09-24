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
from gazebo_msgs.srv import SetModelState, GetEntityState, SetEntityState, ApplyBodyWrench
from gazebo_msgs.msg import ModelState, EntityState
from geometry_msgs.msg import Wrench, Vector3
import tf2_ros
import math

# duration phase params
approach_time = 20.0
descend_and_pick_time = 12.0
transport_time = 20.0
release_time = 10.0

# Approach velocities
approach_left_base_xy_vel = (0.002, 0.24)
approach_right_base_xy_vel = (-0.022, 0.24)
approach_base_angular_vel = 0.0
left_arm_velocity =  [0.07, -0.01, 0.01, -0.01, -0.01, -0.01]   # generic joint velocity command magnitude
right_arm_velocity = [-0.09, -0.01, 0.01, -0.01, 0.01, 0.021]  # generic joint velocity command magnitude

# Arm pick pose commands (adjust as needed)
left_arm_pick_msg = [-0.001, -0.08, 0.08, 0.0, -0.12, -0.05]
right_arm_pick_msg = [0.001, -0.12, 0.08, 0.0, 0.12, 0.05]

# Transport velocities
left_transport_vel_x = -0.002
left_transport_vel_y = -0.24
right_transport_vel_x = 0.02
right_transport_vel_y = -0.24


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

        # Gazebo clients: prefer /gazebo/set_model_state but also support /state/* fallback
        self.set_model_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.get_entity_state_client = self.create_client(GetEntityState, '/state/get_entity_state')
        self.set_entity_state_client = self.create_client(SetEntityState, '/state/set_entity_state')
        self.apply_wrench_client = self.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')

        # wait briefly for any Gazebo state services to appear
        start = time.time()
        timeout = 5.0
        available = False
        while time.time() - start < timeout:
            if (self.set_model_state_client.service_is_ready() or
                    self.get_entity_state_client.service_is_ready() or
                    self.set_entity_state_client.service_is_ready()):
                available = True
                break
            time.sleep(0.1)

        if not available:
            self.get_logger().warn('/gazebo/set_model_state service not available. Object attach will fail if not present. Will try /state/set_entity_state if available.')

        # State machine
        self.start_time = time.time()
        self.state = 'approach'
        self.state_start = time.time()

        # Durations (seconds)
        self.durations = {
            'approach': approach_time,
            'descend_and_pick': descend_and_pick_time,
            'transport': transport_time,
            'release': release_time
        }

        # Destination velocities
        self.left_base_linear_speed  = approach_left_base_xy_vel
        self.right_base_linear_speed = approach_right_base_xy_vel
        self.base_angular_speed = approach_base_angular_vel
        self.left_arm_velocity  = left_arm_velocity
        self.right_arm_velocity = right_arm_velocity

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
            twist_left, twist_right = Twist(), Twist()
            twist_left.linear.x, twist_left.linear.y   = self.left_base_linear_speed
            twist_right.linear.x, twist_right.linear.y = self.right_base_linear_speed
            twist_left.angular.z, twist_right.angular.z = 0.0, 0.0

            # INFO log or Warning
            self.get_logger().info(f'Approaching object {self.object_name}...')
            self.left_base_pub.publish(twist_left)
            self.right_base_pub.publish(twist_right)

            # Small arm motions to prepare
            left_arm_msg, right_arm_msg = Float64MultiArray(), Float64MultiArray()
            left_arm_msg.data = self.left_arm_velocity
            right_arm_msg.data = self.right_arm_velocity
            self.get_logger().info('Moving arms to approach pose...')
            self.left_arm_pub.publish(left_arm_msg)
            self.right_arm_pub.publish(right_arm_msg)

            if elapsed > self.durations['approach']:
                # stop motion
                self.get_logger().info('Reached approach position, stop and prepare to pick')
                self.stop_all_movement()
                self.transition('descend_and_pick')

        elif self.state == 'descend_and_pick':
            # Move arms to pick pose via small velocity commands
            left_arm_pick  = Float64MultiArray()
            right_arm_pick = Float64MultiArray()
            left_arm_pick.data = left_arm_pick_msg
            right_arm_pick.data = right_arm_pick_msg
            self.get_logger().info('Moving arms to pick pose...')
            self.left_arm_pub.publish(left_arm_pick)
            self.right_arm_pub.publish(right_arm_pick)

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
            twist_left.linear.x = left_transport_vel_x
            twist_left.linear.y = left_transport_vel_y
            twist_right.linear.x = right_transport_vel_x
            twist_right.linear.y = right_transport_vel_y
            self.get_logger().info('Transporting object to destination...')
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
            # On failure, also try to dump known frames from the buffer to help debugging
            frames_info = ''
            try:
                frames_info = self.tf_buffer.all_frames_as_yaml()
            except Exception as _e2:
                frames_info = f'<failed to list frames: {_e2}>'
            self.get_logger().error(f'Failed to get transform for {ee_frame}:\n        {e}\nAvailable TF frames:\n{frames_info}')
            return False

        # Apply a constant upward force to simulate magnetic/suction grip
        wrench = Wrench()
        wrench.force = Vector3(x=0.0, y=0.0, z=0.0)  # Counteract gravity
        wrench.torque = Vector3(x=0.0, y=0.0, z=0.0)
        
        req = ApplyBodyWrench.Request()
        req.body_name = f'{model_name}::link_1'  # Adjust if your model's link name is different
        req.reference_frame = 'world'
        req.wrench = wrench
        req.duration.sec = 999999  # Keep force applied for a long time
        
        try:
            future = self.apply_wrench_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.result() is not None:
                self.get_logger().info(f'Applied gripping force to {model_name}')
        except Exception as e:
            self.get_logger().warn(f'Failed to apply gripping force: {e}')

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
            frames_info = ''
            try:
                frames_info = self.tf_buffer.all_frames_as_yaml()
            except Exception as _e2:
                frames_info = f'<failed to list frames: {_e2}>'
            self.get_logger().error(f'Failed to get transform for {ee_frame}: {e}\nAvailable TF frames:\n{frames_info}')
            return False

        pose = Pose()
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z
        pose.orientation = trans.transform.rotation

        return self.call_set_model_state(model_name, pose, reference_frame='world')

    def call_set_model_state(self, model_name, pose: Pose, reference_frame='world'):
        # First try the classic /gazebo/set_model_state if available
        if not self.set_model_state_client.service_is_ready():
            try:
                self.set_model_state_client.wait_for_service(timeout_sec=1.0)
            except Exception:
                pass
        if self.set_model_state_client.service_is_ready():
            req = SetModelState.Request()
            state = ModelState()
            state.model_name = model_name
            state.pose = pose
            state.reference_frame = reference_frame
            req.model_state = state
            future = self.set_model_state_client.call_async(req)
            # wait briefly for the result
            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            except Exception:
                pass
            if future.done() and future.result() is not None:
                res = future.result()
                if hasattr(res, 'success') and res.success:
                    return True
                else:
                    msg = getattr(res, 'status_message', '<no message>')
                    self.get_logger().warn(f'SetModelState failed for {model_name}: {msg}')

        # Fallback: /state/set_entity_state (preserve current z via /state/get_entity_state when possible)
        # Retry for a short while in case the service appears shortly after the node starts
        start_t = time.time()
        retry_timeout = 3.0
        while time.time() - start_t < retry_timeout:
            if not self.set_entity_state_client.service_is_ready():
                try:
                    self.set_entity_state_client.wait_for_service(timeout_sec=0.5)
                except Exception:
                    pass
            if not self.set_entity_state_client.service_is_ready():
                time.sleep(0.1)
                continue

            # try to get current z to avoid hitting ground/unsupported collisions
            z = None
            try:
                if not self.get_entity_state_client.service_is_ready():
                    try:
                        self.get_entity_state_client.wait_for_service(timeout_sec=0.5)
                    except Exception:
                        pass
                if self.get_entity_state_client.service_is_ready():
                    req_get = GetEntityState.Request()
                    req_get.name = model_name
                    fut = self.get_entity_state_client.call_async(req_get)
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
                    if fut.done() and fut.result() is not None:
                        resp = fut.result()
                        if hasattr(resp, 'state') and resp.state is not None and hasattr(resp.state, 'pose'):
                            z = getattr(resp.state.pose.position, 'z', None)
            except Exception:
                z = None

            if z is not None:
                pose.position.z = float(z)

            req2 = SetEntityState.Request()
            ent = EntityState()
            ent.name = model_name
            ent.pose = pose
            ent.reference_frame = reference_frame
            req2.state = ent
            fut2 = self.set_entity_state_client.call_async(req2)
            try:
                rclpy.spin_until_future_complete(self, fut2, timeout_sec=1.0)
            except Exception:
                pass
            if fut2.done() and fut2.result() is not None:
                res2 = fut2.result()
                if hasattr(res2, 'success') and res2.success:
                    return True
                else:
                    self.get_logger().warn(f'SetEntityState returned success=False for {model_name}')
            # else retry until timeout
            time.sleep(0.1)

        self.get_logger().error(f'No suitable Gazebo service available to set model state for {model_name}')
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', default='pacco_clone_1', help='Gazebo model name of the object to pick')
    parser.add_argument('--left_base_topic', default='/left_summit/cmd_vel')
    parser.add_argument('--right_base_topic', default='/right_summit/cmd_vel')
    parser.add_argument('--left_arm_topic', default='/left/ur_left_joint_group_vel_controller/commands')
    parser.add_argument('--right_arm_topic', default='/right/ur_right_joint_group_vel_controller/commands')
    parser.add_argument('--left_ee', default='ur_left_paletta')
    parser.add_argument('--right_ee', default='ur_right_paletta')
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

