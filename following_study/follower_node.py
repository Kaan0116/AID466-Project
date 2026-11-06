#!/usr/bin/env python3
"""
Kişi takip kontrolcüsü.

gap = kişinin arka yüzeyi ile robotun ön yüzeyi arasındaki +x mesafe.
  person_back  = person_x - 0.225   (silindir yarıçapı)
  robot_front  = robot_x  + 0.255   (base_length/2 = 0.255)
  gap = person_back - robot_front
"""

import math
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool


PERSON_RADIUS = 0.225   # m
ROBOT_HALF    = 0.255   # m  (0.51 / 2)


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class FollowerNode(Node):

    def __init__(self):
        super().__init__('follower_node')

        # Parametreler
        self.declare_parameter('target_gap',   1.2)
        self.declare_parameter('kp_gap',       0.8)
        self.declare_parameter('kp_heading',   1.5)
        self.declare_parameter('v_max',        1.5)
        self.declare_parameter('w_max',        1.2)
        self.declare_parameter('v_person',     0.0)   # statik mod
        self.declare_parameter('log_csv',      '')

        self._read_params()

        # Yayıncılar
        self._cmd_pub       = self.create_publisher(Twist,   '/cmd_vel',          10)
        self._gap_pub       = self.create_publisher(Float64, '/follower/gap',      10)
        self._err_pub       = self.create_publisher(Float64, '/follower/gap_error',10)
        self._steady_pub    = self.create_publisher(Bool,    '/follower/steady',   10)

        # Aboneler
        self._person_sub = self.create_subscription(
            Odometry, '/person/odom', self._person_cb, 10)
        self._robot_sub  = self.create_subscription(
            Odometry, '/robot/odom',  self._robot_cb,  10)

        # Durum
        self._person_pose = None
        self._robot_pose  = None
        self._steady_timer = 0.0
        self._steady_start = None

        # CSV log
        self._csv_file   = None
        self._csv_writer = None
        log_path = self.get_parameter('log_csv').get_parameter_value().string_value
        if log_path:
            os.makedirs(os.path.dirname(log_path), exist_ok=True) if os.path.dirname(log_path) else None
            self._csv_file   = open(log_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(['time', 'gap', 'gap_error', 'v_cmd', 'w_cmd'])

        # Kontrol döngüsü 50 Hz
        self._timer = self.create_timer(0.02, self._control_loop)
        self.get_logger().info('follower_node başlatıldı.')

    def _read_params(self):
        self.target_gap  = self.get_parameter('target_gap').get_parameter_value().double_value
        self.kp_gap      = self.get_parameter('kp_gap').get_parameter_value().double_value
        self.kp_heading  = self.get_parameter('kp_heading').get_parameter_value().double_value
        self.v_max       = self.get_parameter('v_max').get_parameter_value().double_value
        self.w_max       = self.get_parameter('w_max').get_parameter_value().double_value
        self.v_person    = self.get_parameter('v_person').get_parameter_value().double_value

    def _person_cb(self, msg: Odometry):
        self._person_pose = msg.pose.pose

    def _robot_cb(self, msg: Odometry):
        self._robot_pose = msg.pose.pose

    def _control_loop(self):
        self._read_params()

        if self._person_pose is None or self._robot_pose is None:
            return

        px = self._person_pose.position.x
        py = self._person_pose.position.y
        rx = self._robot_pose.position.x
        ry = self._robot_pose.position.y

        person_yaw = yaw_from_quat(self._person_pose.orientation)
        robot_yaw  = yaw_from_quat(self._robot_pose.orientation)

        # Kişiden robota vektör (kişi çerçevesinde)
        dx = rx - px
        dy = ry - py

        # +x yönündeki (ileri) mesafe
        # gap = kişi arka yüzeyi - robot ön yüzeyi
        # kişi +x yönünde yürüdüğü için basitçe:
        gap = -(dx * math.cos(person_yaw) + dy * math.sin(person_yaw)) \
              - PERSON_RADIUS - ROBOT_HALF

        gap_error = gap - self.target_gap

        # Hız kontrolü: feedforward (kişi hızı) + P kazancı
        # gap_error > 0 → robot çok uzak → hızlan; < 0 → çok yakın → yavaşla
        v = self.v_person + self.kp_gap * gap_error
        v = max(0.0, min(self.v_max, v))

        # Yön hatası: robot kişinin arkasına bakmalı (person_yaw yönüne)
        # Kişi +x yönünde ilerliyor; robot da +x bakmalı
        heading_error = person_yaw - robot_yaw
        # -pi..pi'ye normalize
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # Lateral hata: kişi ile aynı y'de olmak istiyoruz
        lateral_error = py - ry
        total_angular_error = heading_error + 0.4 * lateral_error

        w = self.kp_heading * total_angular_error
        w = max(-self.w_max, min(self.w_max, w))

        # Steady-state flag: |gap_error| < 0.04 m, 2 s boyunca
        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(gap_error) < 0.04:
            if self._steady_start is None:
                self._steady_start = now
            steady = (now - self._steady_start) >= 2.0
        else:
            self._steady_start = None
            steady = False

        # Yayın
        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        self._cmd_pub.publish(cmd)

        g_msg = Float64(); g_msg.data = float(gap)
        self._gap_pub.publish(g_msg)

        e_msg = Float64(); e_msg.data = float(gap_error)
        self._err_pub.publish(e_msg)

        s_msg = Bool(); s_msg.data = steady
        self._steady_pub.publish(s_msg)

        if self._csv_writer:
            self._csv_writer.writerow([
                f'{now:.3f}', f'{gap:.4f}', f'{gap_error:.4f}',
                f'{v:.4f}', f'{w:.4f}'
            ])

    def destroy_node(self):
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
