#!/usr/bin/env python3
"""
Person-following controller.

gap (see following_study/geometry.py) = distance along the person's heading from the
robot's FRONT edge to the person's BACK surface, using person_back_offset and
robot_front_offset from config/study_params.yaml.

    v = v_person + kp_gap * (gap - target_gap),  clamped to [0, v_max]
    w = kp_heading * (heading_err - k_lateral * lateral),  clamped to [-w_max, w_max]
    steady_state = |gap - target_gap| < steady_tol for steady_time seconds (ROS clock;
                   use_sim_time:=true in every launch file, so this is simulation time)

All parameters must come from the params file (no hidden defaults in code).
"""

import signal
import csv
import os
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64, Bool, String

from following_study.geometry import relation

DOUBLE_PARAMS = ['target_gap', 'kp_gap', 'kp_heading', 'k_lateral', 'v_max', 'w_max',
                 'v_person', 'steady_tol', 'steady_time',
                 'person_back_offset', 'robot_front_offset']
ODOM_TIMEOUT = 0.5   # s — stop the robot if ground truth stops arriving
DISCOVERY_WAIT = 5.0  # s (wall) — listen for an older follower before driving (DDS discovery ~3 s)


class FollowerNode(Node):

    def __init__(self):
        super().__init__('follower_node')
        for name in DOUBLE_PARAMS:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.declare_parameter('log_csv', '')
        self._read_params()

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._gap_pub = self.create_publisher(Float64, '/follower/gap', 10)
        self._err_pub = self.create_publisher(Float64, '/follower/gap_error', 10)
        self._steady_pub = self.create_publisher(Bool, '/follower/steady', 10)

        self.create_subscription(Odometry, '/person/odom', self._person_cb, 10)
        self.create_subscription(Odometry, '/robot/odom', self._robot_cb, 10)

        self._person = None
        self._robot = None
        self._last_odom = None
        self._steady_start = None

        self._csv_file = None
        self._csv = None
        log_path = self.get_parameter('log_csv').value
        if log_path:
            if os.path.dirname(log_path):
                os.makedirs(os.path.dirname(log_path), exist_ok=True)
            self._csv_file = open(log_path, 'w', newline='')
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(['time', 'person_x', 'person_y', 'person_yaw',
                                'robot_x', 'robot_y', 'robot_yaw',
                                'gap', 'gap_error', 'lateral', 'steady', 'v_cmd', 'w_cmd'])

        # --- single-follower guard -------------------------------------------------------
        # Every follower announces (start wall time, pid) on a latched topic. A follower that
        # hears an OLDER instance exits, so two controllers never fight over /cmd_vel,
        # independent of how long DDS discovery takes.
        self._born = time.time()
        self._me = f'{self._born:.6f}:{os.getpid()}'
        self._active = False
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._inst_pub = self.create_publisher(String, '/follower/instance', latched)
        self._inst_pub.publish(String(data=self._me))
        self.create_subscription(String, '/follower/instance', self._instance_cb, latched)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._guard_timer = self.create_timer(DISCOVERY_WAIT, self._activate, clock=steady)

        self.create_timer(0.02, self._control_loop)   # 50 Hz (ROS clock)
        self.get_logger().info(
            f'follower_node started: target_gap={self.p["target_gap"]:.2f} m, '
            f'v_person={self.p["v_person"]:.2f} m/s')

    def _read_params(self):
        try:
            self.p = {n: self.get_parameter(n).value for n in DOUBLE_PARAMS}
        except Exception as e:  # uninitialised parameter -> params file missing
            raise RuntimeError(
                'follower_node needs --params-file config/study_params.yaml') from e
        if any(v is None for v in self.p.values()):
            missing = [n for n, v in self.p.items() if v is None]
            raise RuntimeError(f'follower_node: parameters not set: {missing}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _person_cb(self, msg):
        self._person = msg.pose.pose
        self._last_odom = self._now()

    def _robot_cb(self, msg):
        self._robot = msg.pose.pose
        self._last_odom = self._now()

    def _instance_cb(self, msg):
        if msg.data == self._me:
            return
        other_born = float(msg.data.split(':')[0])
        if other_born < self._born:
            self.get_logger().fatal(
                f'an older follower_node is running ({msg.data}) -> exiting. '
                'Use start_follower:=false when a script starts its own follower.')
            raise SystemExit(2)
        self.get_logger().error(f'a newer follower_node appeared ({msg.data}); it should exit')

    def _activate(self):
        if not self._active:
            self._active = True
            self.get_logger().info('no older follower_node found, control active')
        self._guard_timer.cancel()

    def _control_loop(self):
        if not self._active:
            return
        self._read_params()   # allows `ros2 param set` at runtime (demo panel)
        p = self.p
        if self._person is None or self._robot is None:
            return
        now = self._now()
        if now - self._last_odom > ODOM_TIMEOUT:
            self._cmd_pub.publish(Twist())
            return

        rel = relation(self._person, self._robot,
                       p['person_back_offset'], p['robot_front_offset'])
        gap_error = rel.gap - p['target_gap']

        v = p['v_person'] + p['kp_gap'] * gap_error
        v = max(0.0, min(p['v_max'], v))
        w = p['kp_heading'] * (rel.heading_err - p['k_lateral'] * rel.lateral)
        w = max(-p['w_max'], min(p['w_max'], w))

        if abs(gap_error) < p['steady_tol']:
            if self._steady_start is None:
                self._steady_start = now
            steady = (now - self._steady_start) >= p['steady_time']
        else:
            self._steady_start = None
            steady = False

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self._cmd_pub.publish(cmd)
        self._gap_pub.publish(Float64(data=float(rel.gap)))
        self._err_pub.publish(Float64(data=float(gap_error)))
        self._steady_pub.publish(Bool(data=steady))

        if self._csv:
            pp, rp = self._person.position, self._robot.position
            self._csv.writerow([f'{now:.3f}', f'{pp.x:.4f}', f'{pp.y:.4f}', f'{rel.person_yaw:.4f}',
                                f'{rp.x:.4f}', f'{rp.y:.4f}', f'{rel.robot_yaw:.4f}',
                                f'{rel.gap:.4f}', f'{gap_error:.4f}', f'{rel.lateral:.4f}',
                                int(steady), f'{v:.4f}', f'{w:.4f}'])

    def destroy_node(self):
        try:
            self._cmd_pub.publish(Twist())
        except Exception:
            pass
        if self._csv_file:
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    # Own SIGINT handling: Ctrl+C / launch shutdown -> KeyboardInterrupt -> zero command is sent
    signal.signal(signal.SIGINT, signal.default_int_handler)   # even if the parent ignored SIGINT
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FollowerNode()
    code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        code = e.code
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
