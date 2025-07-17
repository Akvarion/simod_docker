#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class BridgeTester(Node):
    def __init__(self):
        super().__init__('bridge_tester')
        
        # Action clients for testing
        self.left_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left/ur_left_joint_group_pos_controller/follow_joint_trajectory'
        )
        
        self.right_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/right/ur_right_joint_group_pos_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Bridge tester initialized')

    def test_left_arm(self):
        """Test left arm trajectory execution"""
        self.get_logger().info('Testing left arm...')
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'ur_left_shoulder_pan_joint',
            'ur_left_shoulder_lift_joint',
            'ur_left_elbow_joint',
            'ur_left_wrist_1_joint',
            'ur_left_wrist_2_joint',
            'ur_left_wrist_3_joint'
        ]
        
        # Create simple test trajectory
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        point1.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        point1.time_from_start = Duration(sec=2)
        
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, -1.0, 1.0, -1.0, -1.0, 0.5]
        point2.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=4)
        
        goal.trajectory.points = [point1, point2]
        
        # Wait for server
        if not self.left_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Left arm action server not available')
            return False
        
        # Send goal
        future = self.left_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Left arm goal rejected')
            return False
        
        self.get_logger().info('Left arm goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        self.get_logger().info(f'Left arm result: {result.result}')
        
        return True

    def test_right_arm(self):
        """Test right arm trajectory execution"""
        self.get_logger().info('Testing right arm...')
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'ur_right_shoulder_pan_joint',
            'ur_right_shoulder_lift_joint',
            'ur_right_elbow_joint',
            'ur_right_wrist_1_joint',
            'ur_right_wrist_2_joint',
            'ur_right_wrist_3_joint'
        ]
        
        # Create simple test trajectory
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        point1.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        point1.time_from_start = Duration(sec=2)
        
        point2 = JointTrajectoryPoint()
        point2.positions = [-0.5, -1.0, 1.0, -1.0, -1.0, -0.5]
        point2.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=4)
        
        goal.trajectory.points = [point1, point2]
        
        # Wait for server
        if not self.right_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Right arm action server not available')
            return False
        
        # Send goal
        future = self.right_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Right arm goal rejected')
            return False
        
        self.get_logger().info('Right arm goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        self.get_logger().info(f'Right arm result: {result.result}')
        
        return True

    def run_tests(self):
        """Run all tests"""
        self.get_logger().info('Starting bridge tests...')
        
        # Test left arm
        left_success = self.test_left_arm()
        time.sleep(2)
        
        # Test right arm  
        right_success = self.test_right_arm()
        
        if left_success and right_success:
            self.get_logger().info('All tests passed!')
        else:
            self.get_logger().error('Some tests failed!')
        
        return left_success and right_success

def main(args=None):
    rclpy.init(args=args)
    tester = BridgeTester()
    
    try:
        success = tester.run_tests()
        if success:
            print("Bridge testing completed successfully!")
        else:
            print("Bridge testing failed!")
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()