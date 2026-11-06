#!/usr/bin/env python3
"""
capture_stills.py — STEP 5: one rear-camera + one overview still per target gap.

Run it through the launch file (it starts Gazebo without a follower, so this script owns the
ONLY follower_node):
    ros2 launch following_study capture_stills.launch.py [gui:=false]

For every gap in study_params.yaml (study.stills.gaps):
  1. delete + respawn the person (static) and the robot (spawn_offset behind the target)
  2. start follower_node (v_person = 0) and wait for /follower/steady (simulation time)
  3. stop the follower, send a zero command, wait settle_time
  4. compute the actual gap from /person/odom and /robot/odom (shared geometry module);
     retry (restart the follower) if it is outside +-tolerance
  5. save frames whose timestamp is after the settle point:
       rear_gap_{gap:.1f}m.png, overview_gap_{gap:.1f}m.png
Outputs: capture_log.csv (poses, gaps, image stamps), contact_sheet.png (rear + overview).
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from ament_index_python.packages import get_package_prefix
from cv_bridge import CvBridge
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64

from following_study import study_config
from following_study.geometry import relation, robot_spawn_x


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class StillsCapture(Node):

    def __init__(self, cfg_path: str, out_dir: str, gaps=None):
        super().__init__('stills_capture')
        self.cfg_path = cfg_path
        self.cfg = study_config.load(cfg_path)
        self.fp = study_config.follower_params(self.cfg)
        self.spawn_cfg = study_config.study(self.cfg)['spawn']
        self.sc = study_config.study(self.cfg)['stills']
        self.gaps = [float(g) for g in (gaps or self.sc['gaps'])]
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)

        share = os.path.join(get_package_prefix('following_study'), 'share', 'following_study')
        with open(os.path.join(share, 'models', 'person', 'model.sdf')) as f:
            self.person_xml = f.read()
        with open(os.path.join(share, 'urdf', 'robot.urdf')) as f:
            self.robot_xml = f.read()
        self.follower_exe = os.path.join(get_package_prefix('following_study'),
                                         'lib', 'following_study', 'follower_node')

        self.bridge = CvBridge()
        self.person = None
        self.robot = None
        self.rear = None
        self.over = None
        self.steady = False
        self.follower_gap_seen = False
        self.follower = None

        self.create_subscription(Odometry, '/person/odom', lambda m: setattr(self, 'person', m), 10)
        self.create_subscription(Odometry, '/robot/odom', lambda m: setattr(self, 'robot', m), 10)
        self.create_subscription(Image, '/person/rear_cam/image_raw',
                                 lambda m: setattr(self, 'rear', m), 2)
        self.create_subscription(Image, '/overview_cam/image_raw',
                                 lambda m: setattr(self, 'over', m), 2)
        self.create_subscription(Bool, '/follower/steady', self._steady_cb, 10)
        self.create_subscription(Float64, '/follower/gap', self._gap_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

    # ------------------------------------------------------------------ helpers
    def _steady_cb(self, m):
        self.steady = m.data

    def _gap_cb(self, _):
        self.follower_gap_seen = True

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_until(self, pred, sim_timeout: float, wall_timeout: float = None) -> bool:
        """Spin (single thread) until pred() is true or the timeout elapses."""
        t_sim0, t_wall0 = self.now(), time.time()
        wall_timeout = wall_timeout or (sim_timeout * 10.0 + 30.0)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
            if self.now() - t_sim0 > sim_timeout or time.time() - t_wall0 > wall_timeout:
                return False
        return False

    def sleep_sim(self, seconds: float):
        t0 = self.now()
        self.wait_until(lambda: self.now() - t0 >= seconds, seconds + 1.0)

    def call(self, client, req, what: str):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        res = fut.result()
        if res is None or not res.success:
            msg = res.status_message if res else 'timeout'
            self.get_logger().warn(f'{what}: {msg}')
            return False
        return True

    # ------------------------------------------------------------------ steps
    def ensure_single_follower(self):
        """Refuse to run if another follower_node is alive (the old two-follower bug)."""
        self.wait_until(lambda: False, sim_timeout=1e9, wall_timeout=2.0)   # discovery
        names = [n for n, _ in self.get_node_names_and_namespaces()]
        if 'follower_node' in names:
            raise RuntimeError(
                'Another follower_node is running. Start Gazebo with start_follower:=false '
                '(capture_stills.launch.py does this) so this script owns the only follower.')

    def respawn(self, target_gap: float):
        for name in ('person', 'study_robot'):
            req = DeleteEntity.Request()
            req.name = name
            fut = self.delete_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)   # may not exist yet
        self.sleep_sim(0.5)

        robot_x = robot_spawn_x(self.sc_person_x, target_gap, self.fp['person_back_offset'],
                                self.fp['robot_front_offset'], self.spawn_cfg['spawn_offset'])
        for name, xml, x, z in (('person', self.person_xml, self.sc_person_x, 0.0),
                                ('study_robot', self.robot_xml, robot_x,
                                 self.spawn_cfg['robot_z'])):
            req = SpawnEntity.Request()
            req.name, req.xml, req.reference_frame = name, xml, 'world'
            req.initial_pose.position.x = float(x)
            req.initial_pose.position.z = float(z)
            if not self.call(self.spawn_cli, req, f'spawn {name}'):
                raise RuntimeError(f'could not spawn {name}')
        t_spawn = self.now()
        ok = self.wait_until(lambda: self.person is not None and self.robot is not None
                             and stamp_s(self.person) > t_spawn and stamp_s(self.robot) > t_spawn,
                             sim_timeout=10.0)
        if not ok:
            raise RuntimeError('no ground-truth odometry after spawn')
        self.sleep_sim(1.0)   # let the robot settle on its wheels

    def start_follower(self, target_gap: float):
        self.stop_follower()
        self.steady = False
        self.follower_gap_seen = False
        log_csv = os.path.join(self.out_dir, f'follower_gap_{target_gap:.1f}m.csv')
        cmd = [self.follower_exe, '--ros-args', '--params-file', self.cfg_path,
               '-p', f'target_gap:={target_gap}', '-p', 'v_person:=0.0',
               '-p', 'use_sim_time:=true', '-p', f'log_csv:={log_csv}']
        self.follower = subprocess.Popen(cmd, start_new_session=True)

    def stop_follower(self):
        if self.follower and self.follower.poll() is None:
            os.killpg(self.follower.pid, signal.SIGINT)
            try:
                self.follower.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.follower.pid, signal.SIGKILL)
        self.follower = None
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def capture_gap(self, target_gap: float) -> dict:
        tol = self.sc['tolerance']
        self.respawn(target_gap)
        row = None
        for attempt in range(1, self.sc['max_attempts'] + 1):
            self.start_follower(target_gap)
            ok = self.wait_until(lambda: self.follower_gap_seen and self.steady,
                                 sim_timeout=self.sc['steady_timeout'])
            if not ok:
                self.get_logger().warn(f'gap={target_gap:.1f}: steady_state not reached '
                                       f'(attempt {attempt})')
            self.stop_follower()
            self.sleep_sim(self.sc['settle_time'])
            t_settled = self.now()

            # fresh poses and frames, all stamped after the settle point
            got = self.wait_until(
                lambda: (stamp_s(self.person) >= t_settled and stamp_s(self.robot) >= t_settled
                         and self.rear is not None and self.over is not None
                         and stamp_s(self.rear) >= t_settled and stamp_s(self.over) >= t_settled),
                sim_timeout=5.0)
            if not got:
                raise RuntimeError('no fresh odometry/images after settling')
            person, robot, rear, over = self.person, self.robot, self.rear, self.over
            rel = relation(person.pose.pose, robot.pose.pose,
                           self.fp['person_back_offset'], self.fp['robot_front_offset'])
            within = abs(rel.gap - target_gap) <= tol
            self.get_logger().info(
                f'gap={target_gap:.1f}: actual={rel.gap:.4f} m '
                f'(err {rel.gap - target_gap:+.4f}) steady={ok} '
                f'{"OK" if within else "OUT OF TOLERANCE"}')
            row = {
                'target_gap': f'{target_gap:.1f}', 'actual_gap': f'{rel.gap:.4f}',
                'error': f'{rel.gap - target_gap:+.4f}', 'within_tolerance': within,
                'tolerance': tol, 'steady_reached': ok, 'attempt': attempt,
                'person_x': f'{person.pose.pose.position.x:.4f}',
                'person_y': f'{person.pose.pose.position.y:.4f}',
                'person_yaw': f'{rel.person_yaw:.4f}',
                'robot_x': f'{robot.pose.pose.position.x:.4f}',
                'robot_y': f'{robot.pose.pose.position.y:.4f}',
                'robot_yaw': f'{rel.robot_yaw:.4f}',
                'lateral': f'{rel.lateral:.4f}',
                'pose_stamp': f'{stamp_s(person):.3f}',
                'rear_stamp': f'{stamp_s(rear):.3f}', 'overview_stamp': f'{stamp_s(over):.3f}',
                'rear_image': f'rear_gap_{target_gap:.1f}m.png',
                'overview_image': f'overview_gap_{target_gap:.1f}m.png',
                'image_width': rear.width, 'image_height': rear.height,
            }
            if within:
                break
        cv2.imwrite(os.path.join(self.out_dir, row['rear_image']),
                    self.bridge.imgmsg_to_cv2(rear, 'bgr8'))
        cv2.imwrite(os.path.join(self.out_dir, row['overview_image']),
                    self.bridge.imgmsg_to_cv2(over, 'bgr8'))
        return row

    def run(self):
        self.sc_person_x = float(self.spawn_cfg['person_x'])
        for cli in (self.delete_cli, self.spawn_cli):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'waiting for {cli.srv_name} ...')
        self.wait_until(lambda: self.now() > 0.0, sim_timeout=1e9, wall_timeout=60.0)
        self.ensure_single_follower()

        rows = []
        try:
            for g in self.gaps:
                rows.append(self.capture_gap(g))
        finally:
            self.stop_follower()
            if rows:
                path = os.path.join(self.out_dir, 'capture_log.csv')
                with open(path, 'w', newline='') as f:
                    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                    w.writeheader()
                    w.writerows(rows)
                self.get_logger().info(f'log: {path}')
        build_contact_sheet(self.out_dir, self.gaps)
        bad = [r['target_gap'] for r in rows if not r['within_tolerance']]
        if bad:
            self.get_logger().error(f'out of tolerance: {bad}')
        return not bad


def build_contact_sheet(out_dir: str, gaps, cols: int = 4, thumb_w: int = 427):
    """Rear stills (top block) and overview stills (bottom block), labelled per gap."""
    blocks = []
    for kind in ('rear', 'overview'):
        thumbs = []
        for g in gaps:
            img = cv2.imread(os.path.join(out_dir, f'{kind}_gap_{g:.1f}m.png'))
            if img is None:
                img = np.zeros((720, 1280, 3), np.uint8)
            h = int(round(img.shape[0] * thumb_w / img.shape[1]))
            img = cv2.resize(img, (thumb_w, h))
            label = np.full((30, thumb_w, 3), 255, np.uint8)
            cv2.putText(label, f'{kind}  gap={g:.1f} m', (6, 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
            thumbs.append(np.vstack([label, img]))
        while len(thumbs) % cols:
            thumbs.append(np.full_like(thumbs[0], 255))
        blocks += [np.hstack(thumbs[i:i + cols]) for i in range(0, len(thumbs), cols)]
    path = os.path.join(out_dir, 'contact_sheet.png')
    cv2.imwrite(path, np.vstack(blocks))
    return path


def main():
    argv = remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default=os.path.expanduser('~/ros2_ws/results/stills'))
    ap.add_argument('--params-file', default='')
    ap.add_argument('--gaps', nargs='+', type=float, help='override study.stills.gaps')
    args = ap.parse_args(argv)

    rclpy.init(args=sys.argv)
    node = StillsCapture(args.params_file or study_config.default_config_path(),
                         os.path.expanduser(args.out_dir), args.gaps)
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_follower()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
