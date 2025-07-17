#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import asyncio

class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge')
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.cmd_pub_base_left = self.create_publisher(
            Twist,
            '/left_summit/cmd_vel',
            10
        )
        self.cmd_pub_base_right = self.create_publisher(
            Twist,
            '/right_summit/cmd_vel',
            10
        )
        self.cmd_pub_arm_left = self.create_publisher(
            Float64MultiArray,
            '/left/ur_left_joint_group_vel_controller/commands',
            10
        )
        self.cmd_pub_arm_right= self.create_publisher(
            Float64MultiArray,
            '/right/ur_right_joint_group_vel_controller/commands',
            10
        )   

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received trajectory goal')
        traj = goal_handle.request.trajectory
        start_time = self.get_clock().now().nanoseconds / 1e9

        for point in traj.points:
            # Wait until the correct time_from_start
            now = self.get_clock().now().nanoseconds / 1e9
            wait_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9 - (now - start_time)
            if wait_time > 0:
                await asyncio.sleep(wait_time)

            msg = Float64MultiArray()
            msg.data = list(point.velocities)
            if traj.joint_names[0].startswith('ur_left_'):
                self.cmd_pub_arm_left.publish(msg)
            elif traj.joint_names[0].startswith('ur_right_'):
                self.cmd_pub_arm_right.publish(msg)
            elif traj.joint_names[0] == 'left_summit':
                twist = Twist()
                # Assuming [vx, vy, wz] order in velocities for the base
                twist.linear.x = point.velocities[0]
                twist.linear.y = point.velocities[1]
                twist.angular.z = point.velocities[2]
                self.cmd_pub_base_left.publish(twist)
            elif traj.joint_names[0] == 'right_summit':
                twist = Twist()
                twist.linear.x = point.velocities[0]
                twist.linear.y = point.velocities[1]
                twist.angular.z = point.velocities[2]
                self.cmd_pub_base_right.publish(twist)
            else :
                self.get_logger().warn('Unknown joint group prefix:'+traj.joint_names[0])

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()