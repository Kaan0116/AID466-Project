#!/usr/bin/env python3
"""
capture_stills.py — Her hedef mesafe için koridor fotoğrafı çeker.

Kullanım:
  # gzserver zaten çalışıyor olmalı (simulation.launch.py ile başlatılmış):
  python3 capture_stills.py

Çıktılar:
  ~/ros2_ws/results/stills/rear_gap_{gap:.1f}m.png
  ~/ros2_ws/results/stills/overview_gap_{gap:.1f}m.png
  ~/ros2_ws/results/stills/capture_log.csv
  ~/ros2_ws/results/stills/contact_sheet.png
"""

import os
import sys
import time
import csv
import subprocess
import threading
import argparse

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
import cv2
import numpy as np
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


TARGET_GAPS  = [0.4, 0.6, 0.9, 1.2, 1.5, 2.0, 3.0]
OUTPUT_DIR   = os.path.expanduser('~/ros2_ws/results/stills')
LOG_CSV      = os.path.join(OUTPUT_DIR, 'capture_log.csv')
TOLERANCE    = 0.03   # ± m

PERSON_X     = 10.0   # kişi spawn konumu
PERSON_RADIUS= 0.225  # m
ROBOT_HALF   = 0.255  # m (base_length/2)
ROBOT_Z      = 0.225  # m (zemin üstü yükseklik)


def _read_sdf(path: str) -> str:
    with open(path) as f:
        return f.read()


class StillsNode(Node):

    def __init__(self, pkg_share: str, urdf_path: str):
        super().__init__('stills_capture')
        self.bridge     = CvBridge()
        self.pkg_share  = pkg_share
        self.urdf_path  = urdf_path
        self.person_sdf = os.path.join(pkg_share, 'models', 'person', 'model.sdf')

        self._steady  = False
        self._gap     = None
        self._rear    = None
        self._over    = None
        self._follower_proc = None

        self.create_subscription(Bool,   '/follower/steady',          self._steady_cb, 10)
        self.create_subscription(Float64,'/follower/gap',              self._gap_cb,    10)
        self.create_subscription(Image,  '/person/rear_cam/image_raw', self._rear_cb,   10)
        self.create_subscription(Image,  '/overview_cam/image_raw',    self._over_cb,   10)

        self._delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self._spawn_cli  = self.create_client(SpawnEntity,  '/spawn_entity')

    def _steady_cb(self, m): self._steady = m.data
    def _gap_cb(self, m):    self._gap    = m.data
    def _rear_cb(self, m):   self._rear   = m
    def _over_cb(self, m):   self._over   = m

    def _wait_for_services(self):
        for cli in [self._delete_cli, self._spawn_cli]:
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'{cli.srv_name} bekleniyor...')

    def _delete_entity(self, name: str):
        req = DeleteEntity.Request()
        req.name = name
        fut = self._delete_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def _spawn_entity(self, name: str, xml: str, x: float, y=0.0, z=0.0):
        req = SpawnEntity.Request()
        req.name            = name
        req.xml             = xml
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.reference_frame = 'world'
        fut = self._spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() and fut.result().success:
            self.get_logger().info(f'Spawn OK: {name} @ x={x:.2f}')
        else:
            self.get_logger().warn(f'Spawn BAŞARISIZ: {name}')

    def _start_follower(self, target_gap: float, v_person=0.0):
        """Follower node'u subprocess olarak başlat."""
        if self._follower_proc and self._follower_proc.poll() is None:
            self._follower_proc.terminate()
            self._follower_proc.wait(timeout=5)

        self._steady = False
        self._gap    = None

        cmd = [
            'ros2', 'run', 'following_study', 'follower_node',
            '--ros-args',
            '-p', f'target_gap:={target_gap}',
            '-p', f'v_person:={v_person}',
            '-p', 'kp_gap:=4.0',
            '-p', 'kp_heading:=2.0',
            '-p', 'v_max:=1.5',
        ]
        self._follower_proc = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _stop_follower(self):
        if self._follower_proc and self._follower_proc.poll() is None:
            self._follower_proc.terminate()
            try:
                self._follower_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._follower_proc.kill()
        # Robotu durdur
        import rclpy
        from geometry_msgs.msg import Twist
        pub = self.create_publisher(Twist, '/cmd_vel', 1)
        pub.publish(Twist())
        time.sleep(0.5)
        self.destroy_publisher(pub)

    def wait_for_steady(self, timeout=60.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._steady:
                return True
        return False

    def grab_images(self):
        """Son gelen kareleri döndür."""
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._rear, self._over, self._gap

    def run_all_gaps(self):
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self._wait_for_services()

        with open(OUTPUT_DIR + '/robot.urdf') as f:
            robot_xml = f.read()
        person_xml = _read_sdf(self.person_sdf)

        log_rows = []

        for target_gap in TARGET_GAPS:
            self.get_logger().info(f'\n=== target_gap={target_gap:.1f}m ===')

            # 1. Eski entity'leri temizle
            self._delete_entity('person')
            self._delete_entity('study_robot')
            time.sleep(1.5)

            # 2. Robot başlangıç konumu: kişinin arkasında, yaklaşık olarak
            robot_x = PERSON_X - PERSON_RADIUS - target_gap - ROBOT_HALF - 0.5
            robot_x = max(0.5, robot_x)

            self._spawn_entity('person',      person_xml, PERSON_X, 0.0, 0.0)
            self._spawn_entity('study_robot', robot_xml,  robot_x,  0.0, ROBOT_Z)
            time.sleep(2.0)

            # 3. Follower başlat
            self._start_follower(target_gap, v_person=0.0)

            # 4. Steady-state bekle
            ok = self.wait_for_steady(timeout=60.0)
            if not ok:
                self.get_logger().warn(f'Zaman aşımı! gap={target_gap:.1f}m — devam ediyorum.')

            # 5. Follower durdur, osilasyonun geçmesini bekle
            self._stop_follower()
            time.sleep(0.5)

            # 6. Görüntü al
            rear_msg, over_msg, actual_gap = self.grab_images()

            rear_path = os.path.join(OUTPUT_DIR, f'rear_gap_{target_gap:.1f}m.png')
            over_path = os.path.join(OUTPUT_DIR, f'overview_gap_{target_gap:.1f}m.png')

            if rear_msg:
                cv2.imwrite(rear_path, self.bridge.imgmsg_to_cv2(rear_msg, 'bgr8'))
            else:
                self.get_logger().warn('Arka kamera görüntüsü alınamadı!')

            if over_msg:
                cv2.imwrite(over_path, self.bridge.imgmsg_to_cv2(over_msg, 'bgr8'))
            else:
                self.get_logger().warn('Genel bakış görüntüsü alınamadı!')

            # 7. Doğrula
            if actual_gap is not None:
                within = abs(actual_gap - target_gap) <= TOLERANCE
                status = 'OK' if within else f'TOLERANS_DIŞI (Δ={abs(actual_gap-target_gap):.3f}m)'
                self.get_logger().info(
                    f'  actual_gap={actual_gap:.3f}m  hedef={target_gap:.1f}m  [{status}]')
            else:
                within  = False
                actual_gap = float('nan')

            log_rows.append({
                'target_gap':       target_gap,
                'actual_gap':       f'{actual_gap:.4f}',
                'within_tolerance': within,
                'steady_reached':   ok,
                'rear_image':       os.path.basename(rear_path),
                'overview_image':   os.path.basename(over_path),
            })

        # CSV yaz
        with open(LOG_CSV, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()))
            w.writeheader()
            w.writerows(log_rows)
        self.get_logger().info(f'Log: {LOG_CSV}')

        # Contact sheet
        self._build_contact_sheet()

    def _build_contact_sheet(self):
        imgs = []
        for g in TARGET_GAPS:
            p = os.path.join(OUTPUT_DIR, f'rear_gap_{g:.1f}m.png')
            img = cv2.imread(p)
            if img is None:
                img = np.zeros((240, 427, 3), dtype=np.uint8)
            else:
                img = cv2.resize(img, (427, 240))
            cv2.putText(img, f'gap={g:.1f}m', (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            imgs.append(img)

        # 2 x 4 ızgara (7 resim, son hücre boş)
        while len(imgs) % 4 != 0:
            imgs.append(np.zeros_like(imgs[0]))

        rows = [np.hstack(imgs[i:i+4]) for i in range(0, len(imgs), 4)]
        sheet = np.vstack(rows)
        out = os.path.join(OUTPUT_DIR, 'contact_sheet.png')
        cv2.imwrite(out, sheet)
        self.get_logger().info(f'Contact sheet: {out}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pkg-share', default='',
                        help='following_study share dizini (otomatik bulunur)')
    args = parser.parse_args()

    pkg_share = args.pkg_share or get_package_share_directory('following_study')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    # robot.urdf'i OUTPUT_DIR'e kopyala (spawn için)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    import shutil
    shutil.copy(urdf_path, os.path.join(OUTPUT_DIR, 'robot.urdf'))

    rclpy.init()
    node = StillsNode(pkg_share, urdf_path)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_all_gaps()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
