#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class FakeBaseStatePublisher(Node):
    def __init__(self):
        super().__init__('fake_base_state_publisher')
        self.declare_parameter('side')
        self.side = self.get_parameter('side').value
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_state)

    def publish_fake_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.side == 'left':
            msg.name = [
                'position',  # virtual joint (planar)
                'left_summit_back_left_wheel_joint',
                'left_summit_back_right_wheel_joint',
                'left_summit_front_left_wheel_joint',
                'left_summit_front_right_wheel_joint'
            ]
            msg.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        elif self.side == 'right':
            msg.name = [
                'position',  # virtual joint (planar)
                'right_summit_back_left_wheel_joint',
                'right_summit_back_right_wheel_joint',
                'right_summit_front_left_wheel_joint',
                'right_summit_front_right_wheel_joint'
            ]
            msg.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        else:  # both
            msg.name = [
                'left_summit_back_left_wheel_joint',
                'left_summit_back_right_wheel_joint',
                'left_summit_front_left_wheel_joint',
                'left_summit_front_right_wheel_joint',
                'right_summit_back_left_wheel_joint',
                'right_summit_back_right_wheel_joint',
                'right_summit_front_left_wheel_joint',
                'right_summit_front_right_wheel_joint'
            ]
            msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = FakeBaseStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()