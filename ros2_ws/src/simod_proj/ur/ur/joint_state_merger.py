#!/usr/bin/env python3

import rclpy, sys
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')
        self.declare_parameter('side')
        side = self.get_parameter('side').value
        self.get_logger().info(f"Selected side: {side}")
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_states = {}  # store latest JointState per side
        
        # Subscribe based on the provided side, pass side to callback
        if side in ('left', 'both'):
            self.create_subscription(JointState, '/left/joint_states', lambda msg: self.cb(msg, 'left'), 10)
        if side in ('right', 'both'):
            self.create_subscription(JointState, '/right/joint_states', lambda msg: self.cb(msg, 'right'), 10)
        
        self.timer = self.create_timer(0.02, self.publish_merged)

    # store latest message for that side
    def cb(self, msg, side):
        self.joint_states[side] = msg

    def publish_merged(self):
        merged = JointState()
        # use the newest stamp among inputs if available, otherwise node clock
        latest_ts = None
        for msg in self.joint_states.values():
            ts = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if latest_ts is None or ts > latest_ts:
                latest_ts = ts
        if latest_ts is not None:
            merged.header.stamp.sec, merged.header.stamp.nanosec = latest_ts
        else:
            merged.header.stamp = self.get_clock().now().to_msg()

        name_idx = {}
        names = []
        positions = []
        velocities = []
        efforts = []

        # deterministic iterate sides (sorted keys) so ordering is stable
        for side in sorted(self.joint_states.keys()):
            msg = self.joint_states[side]
            for i, name in enumerate(msg.name):
                if name in name_idx:
                    idx = name_idx[name]
                    if i < len(msg.position):
                        positions[idx] = msg.position[i]
                    if i < len(msg.velocity):
                        velocities[idx] = msg.velocity[i]
                    if i < len(msg.effort):
                        efforts[idx] = msg.effort[i]
                else:
                    idx = len(names)
                    name_idx[name] = idx
                    names.append(name)
                    positions.append(msg.position[i] if i < len(msg.position) else 0.0)
                    velocities.append(msg.velocity[i] if i < len(msg.velocity) else 0.0)
                    efforts.append(msg.effort[i] if i < len(msg.effort) else 0.0)

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