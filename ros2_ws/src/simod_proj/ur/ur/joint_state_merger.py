#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_states = {}
        self.create_subscription(JointState, '/left/joint_states', self.cb, 10)
        self.create_subscription(JointState, '/right/joint_states', self.cb, 10)
        self.timer = self.create_timer(0.02, self.publish_merged)

    def cb(self, msg):
        self.joint_states[msg.header.stamp.sec + msg.header.stamp.nanosec] = msg

    def publish_merged(self):
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()
        names = []
        positions = []
        velocities = []
        efforts = []
        for msg in self.joint_states.values():
            names += msg.name
            positions += msg.position
            velocities += msg.velocity
            efforts += msg.effort
        merged.name = names
        merged.position = positions
        merged.velocity = velocities
        merged.effort = efforts
        if names:
            self.pub.publish(merged)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()