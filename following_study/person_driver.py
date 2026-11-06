#!/usr/bin/env python3
"""
Drives the person model (planar_move) along +x at a constant speed.

Parameters (config/study_params.yaml, section person_driver):
  speed        m/s, 0.0 = static
  start_delay  s of ROS (simulation) time to wait before walking
  stop_x       the person stops when its x reaches this value (front wall is at x = 30)
Publishes /person/walking (Bool) so recorders know when the person is moving.
"""

import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class PersonDriver(Node):

    def __init__(self):
        super().__init__('person_driver')
        for name in ('speed', 'start_delay', 'stop_x'):
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.speed = self.get_parameter('speed').value
        self.start_delay = self.get_parameter('start_delay').value
        self.stop_x = self.get_parameter('stop_x').value
        if None in (self.speed, self.start_delay, self.stop_x):
            raise RuntimeError('person_driver needs --params-file config/study_params.yaml')

        self._pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self._walk_pub = self.create_publisher(Bool, '/person/walking', 10)
        self.create_subscription(Odometry, '/person/odom', self._odom_cb, 10)
        self._x = None
        self._t0 = None
        self._stopped = False
        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            f'person_driver: speed={self.speed:.2f} m/s, start_delay={self.start_delay:.1f} s, '
            f'stop_x={self.stop_x:.1f} m')

    def _odom_cb(self, msg):
        self._x = msg.pose.pose.position.x

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        cmd = Twist()
        walking = False
        if self._x is not None and now > 0.0:
            if self._t0 is None:
                self._t0 = now
            if self._x >= self.stop_x and not self._stopped:
                self._stopped = True
                self.get_logger().info(f'person reached stop_x={self.stop_x:.1f} m, stopping')
            if not self._stopped and now - self._t0 >= self.start_delay and self.speed > 0.0:
                cmd.linear.x = float(self.speed)
                walking = True
        self._pub.publish(cmd)
        self._walk_pub.publish(Bool(data=walking))


def main(args=None):
    # Own SIGINT handling: Ctrl+C / launch shutdown -> KeyboardInterrupt -> zero command is sent
    signal.signal(signal.SIGINT, signal.default_int_handler)   # even if the parent ignored SIGINT
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PersonDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
