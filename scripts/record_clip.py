#!/usr/bin/env python3
"""
record_clip.py — STEP 6: exactly `duration` s of rear-camera video at `fps` (study_params.yaml).

Run it through the launch file (it sets the follower target and makes the person walk):
    ros2 launch following_study record_clip.launch.py gap:=1.2 [gui:=false]

Timing is done with the image header stamps (simulation time), not the wall clock:
slot k is filled with the newest frame whose stamp is <= t_start + k/fps (a missing
frame is replaced by the previous one and counted). The MP4 therefore always has
duration*fps frames, plays at fps, and covers exactly `duration` s of simulation time,
whatever the real-time factor. No text is drawn on the clip; --annotate writes a separate
annotated copy for checking.

Recording starts when the person is walking AND (/follower/steady is true OR
warmup_timeout s of walking have passed).
Outputs (out_dir): clip_gap_{gap:.1f}m.mp4, clip_gap_{gap:.1f}m_frames.csv
(per-frame stamp + gap from ground truth), [clip_gap_{gap:.1f}m_annotated.mp4]
"""

import argparse
import csv
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from following_study import study_config
from following_study.geometry import relation

PIX_FMT = {'rgb8': 'rgb24', 'bgr8': 'bgr24'}


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def ffmpeg_writer(path, w, h, fps, pix_fmt):
    return subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', pix_fmt,
         '-s', f'{w}x{h}', '-r', str(fps), '-i', 'pipe:0',
         '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18', '-preset', 'medium',
         '-r', str(fps), path], stdin=subprocess.PIPE)


class ClipRecorder(Node):

    def __init__(self, gap, out_dir, cfg_path, annotate):
        super().__init__('clip_recorder')
        cfg = study_config.load(cfg_path)
        self.fp = study_config.follower_params(cfg)
        c = study_config.study(cfg)['clips']
        self.fps = int(c['fps'])
        self.n_frames = int(round(c['duration'] * self.fps))
        self.warmup = float(c['warmup_timeout'])
        self.gap = gap
        self.annotate = annotate
        os.makedirs(out_dir, exist_ok=True)
        base = os.path.join(out_dir, f'clip_gap_{gap:.1f}m')
        self.mp4, self.frames_csv, self.ann_mp4 = base + '.mp4', base + '_frames.csv', \
            base + '_annotated.mp4'

        self.person = self.robot = None
        self.steady = False
        self.walking_since = None
        self.t_start = None
        self.k = 0
        self.prev = None
        self.duplicated = 0
        self.proc = self.ann = None
        self.rows = []
        self.done = False

        qos = QoSProfile(depth=60, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/person/rear_cam/image_raw', self._img_cb, qos)
        self.create_subscription(Odometry, '/person/odom', lambda m: setattr(self, 'person', m), 10)
        self.create_subscription(Odometry, '/robot/odom', lambda m: setattr(self, 'robot', m), 10)
        self.create_subscription(Bool, '/follower/steady', lambda m: setattr(self, 'steady', m.data), 10)
        self.create_subscription(Bool, '/person/walking', self._walk_cb, 10)
        self.get_logger().info(f'waiting for walking + steady_state (gap={gap:.1f} m) ...')

    def _walk_cb(self, m):
        now = self.get_clock().now().nanoseconds * 1e-9
        if m.data and self.walking_since is None:
            self.walking_since = now
        if not m.data and self.t_start is not None and not self.done:
            self.get_logger().error('person stopped during the recording (corridor too short?)')

    def _ready(self, t) -> bool:
        return (self.walking_since is not None and
                (self.steady or t - self.walking_since >= self.warmup))

    def _write(self, msg, slot_t):
        self.proc.stdin.write(bytes(msg.data))
        gap = float('nan')
        if self.person and self.robot:
            gap = relation(self.person.pose.pose, self.robot.pose.pose,
                           self.fp['person_back_offset'], self.fp['robot_front_offset']).gap
        self.rows.append([self.k, f'{slot_t - self.t_start:.4f}', f'{stamp_s(msg):.4f}',
                          f'{gap:.4f}',
                          f'{self.person.pose.pose.position.x:.4f}' if self.person else '',
                          f'{self.robot.pose.pose.position.x:.4f}' if self.robot else ''])
        if self.ann:
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3).copy()
            cv2.putText(img, f't={slot_t - self.t_start:5.2f}s gap={gap:.3f}m target={self.gap:.1f}m',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            self.ann.stdin.write(img.tobytes())
        self.k += 1

    def _img_cb(self, msg):
        if self.done:
            return
        s = stamp_s(msg)
        if self.t_start is None:
            if not self._ready(s):
                return
            if msg.encoding not in PIX_FMT:
                raise RuntimeError(f'unsupported encoding {msg.encoding}')
            self.t_start = s
            self.proc = ffmpeg_writer(self.mp4, msg.width, msg.height, self.fps,
                                      PIX_FMT[msg.encoding])
            if self.annotate:
                self.ann = ffmpeg_writer(self.ann_mp4, msg.width, msg.height, self.fps,
                                         PIX_FMT[msg.encoding])
            self.get_logger().info(
                f'recording {self.n_frames} frames ({self.n_frames / self.fps:.1f} s @ {self.fps} fps), '
                f'steady={self.steady}, frame {msg.width}x{msg.height}')
        half = 0.5 / self.fps
        while self.k < self.n_frames:
            slot_t = self.t_start + self.k / self.fps
            if slot_t > s + half:
                break                               # this frame is too early for the next slot
            if slot_t < s - half and self.prev is not None:
                self._write(self.prev, slot_t)      # frame missing for this slot -> repeat
                self.duplicated += 1
            else:
                self._write(msg, slot_t)
        self.prev = msg
        if self.k >= self.n_frames:
            self.finish()

    def finish(self):
        if self.done:
            return
        self.done = True
        for p in (self.proc, self.ann):
            if p:
                p.stdin.close()
                p.wait(timeout=120)
        with open(self.frames_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['frame', 't_video', 'image_stamp', 'gap', 'person_x', 'robot_x'])
            w.writerows(self.rows)
        self.get_logger().info(f'{self.mp4}: {self.k} frames, {self.duplicated} repeated')


def main():
    argv = remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--gap', type=float, required=True)
    ap.add_argument('--out-dir', default=os.path.expanduser('~/ros2_ws/results/videos'))
    ap.add_argument('--params-file', default='')
    ap.add_argument('--annotate', action='store_true',
                    help='also write an annotated copy (time/gap overlay)')
    ap.add_argument('--timeout', type=float, default=600.0, help='wall-clock safety timeout [s]')
    args = ap.parse_args(argv)

    rclpy.init(args=sys.argv)
    node = ClipRecorder(args.gap, os.path.expanduser(args.out_dir),
                        args.params_file or study_config.default_config_path(), args.annotate)
    t0 = time.time()
    try:
        while rclpy.ok() and not node.done and time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = node.done and node.k == node.n_frames
    if not node.done:
        node.get_logger().error('timeout: clip incomplete')
        node.finish()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
