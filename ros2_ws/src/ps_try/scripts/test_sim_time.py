#!/usr/bin/env python3
import time
import datetime
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class GazeboTimeNode(Node):
    def __init__(self):
        super().__init__('gazebo_time_node')
        # Ensure we subscribe to /clock (enable sim time for node time API if desired)
        # self.declare_parameter('use_sim_time', True)
        # self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # use a QoS that matches Gazebo's /clock publisher (BEST_EFFORT)
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # create subscription with matching QoS so messages are received
        self.sub = self.create_subscription(Clock, '/clock', self.clock_cb, qos_profile=qos)
        self.wall_start = None    # wall-clock (monotonic) when first /clock arrives
        self.first_clock = None   # first sim time value (seconds float)
        self.get_logger().info('GazeboTimeNode started; waiting for /clock messages...')

    def clock_cb(self, msg: Clock):
        # convert Clock msg to float seconds
        sim_seconds = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9

        # on first message record wall start (monotonic) and sim start
        if self.wall_start is None:
            self.wall_start = time.monotonic()
            self.first_clock = sim_seconds

        # compute sim elapsed and wall elapsed
        sim_elapsed = sim_seconds - self.first_clock
        wall_elapsed = time.monotonic() - self.wall_start

        # compute RTF (guard division by zero)
        rtf = sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0.0

        # format sim time as H:M:S or seconds depending on magnitude
        sim_str = f"{sim_elapsed:.6f}s"
        wall_str = f"{wall_elapsed:.6f}s"

        # optional human-friendly timestamp (UTC) for sim_time interpreted as epoch-like value
        # dt = datetime.datetime.utcfromtimestamp(sim_seconds)  # not usually meaningful
        self.get_logger().info(f"Sim Time: {sim_str}    Real Time: {wall_str}    RTF: {rtf:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = GazeboTimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()