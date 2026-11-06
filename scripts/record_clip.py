#!/usr/bin/env python3
"""
20 saniyelik birinci şahıs video kaydı.
Arka kamera görüntüsünü /person/rear_cam/image_raw'dan alır,
ffmpeg pipeline ile MP4 olarak yazar.
"""

import os
import sys
import time
import subprocess
import signal
import threading
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


DURATION   = 20.0   # saniye
FPS        = 30
WIDTH      = 1280
HEIGHT     = 720
OUTPUT_DIR = os.path.expanduser('~/ros2_ws/results/videos')


class VideoRecorder(Node):

    def __init__(self, gap: float, output_path: str):
        super().__init__('video_recorder')
        self.bridge  = CvBridge()
        self.gap     = gap
        self._proc   = None
        self._done   = False
        self._t_start = None

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        self.output_path = output_path

        # ffmpeg'e ham BGR24 akışı gönder
        self._proc = subprocess.Popen([
            'ffmpeg', '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', f'{WIDTH}x{HEIGHT}',
            '-pix_fmt', 'bgr24',
            '-r', str(FPS),
            '-i', 'pipe:0',
            '-vcodec', 'libx264',
            '-pix_fmt', 'yuv420p',
            '-preset', 'fast',
            '-crf', '18',
            output_path
        ], stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)

        self.create_subscription(
            Image, '/person/rear_cam/image_raw', self._img_cb, 10)

        self.get_logger().info(
            f'Kayıt başlıyor: gap={gap:.1f}m -> {output_path}')

    def _img_cb(self, msg: Image):
        if self._done:
            return
        if self._t_start is None:
            self._t_start = time.time()

        elapsed = time.time() - self._t_start
        if elapsed >= DURATION:
            self._finish()
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if img.shape[1] != WIDTH or img.shape[0] != HEIGHT:
                img = cv2.resize(img, (WIDTH, HEIGHT))

            # Zaman damgası
            ts = f't={elapsed:.1f}s  gap={self.gap:.1f}m'
            cv2.putText(img, ts, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

            self._proc.stdin.write(img.tobytes())
        except Exception as e:
            self.get_logger().warn(f'Frame hatası: {e}')

    def _finish(self):
        if self._done:
            return
        self._done = True
        try:
            self._proc.stdin.close()
            self._proc.wait(timeout=10)
        except Exception:
            self._proc.kill()
        self.get_logger().info(f'Video kaydedildi: {self.output_path}')

    @property
    def finished(self):
        return self._done

    def destroy_node(self):
        self._finish()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gap', type=float, default=1.2,
                        help='Hedef takip mesafesi (m)')
    args, _ = parser.parse_known_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out = os.path.join(OUTPUT_DIR, f'clip_gap_{args.gap:.1f}m.mp4')

    rclpy.init()
    node = VideoRecorder(gap=args.gap, output_path=out)

    t0 = time.time()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 > DURATION + 5.0:
            break

    node.destroy_node()
    rclpy.shutdown()
    print(f'Bitti: {out}')


if __name__ == '__main__':
    main()
