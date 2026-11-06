#!/usr/bin/env python3
"""
demo_panel.py — HRI Çalışması Demo Paneli

Kullanım:
  # Önce simülasyonu başlat:
  #   terminal 1: gazebo komutu (gzserver + gzclient)
  #   terminal 2: spawn_entity komutları
  # Sonra:
  python3 demo_panel.py

Özellikler:
  - 5 farklı takip mesafesi butonu
  - Canlı birinci şahıs kamera (kişinin arkaya bakan kamerası)
  - Canlı genel bakış kamerası
  - Kişiyi yürüt / durdur
  - Anlık mesafe ve hata göstergesi
"""

import os
import sys
import time
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

import tkinter as tk
from tkinter import font as tkfont
from PIL import Image as PILImage, ImageTk

# ─── Sabitler ──────────────────────────────────────────────────────────────
DISTANCES   = [0.6, 0.9, 1.2, 1.5, 2.0]   # 5 hedef mesafe
PERSON_SPEED = 0.8   # m/s
CAM_W, CAM_H = 640, 360   # panel önizleme boyutu
UPDATE_MS    = 66         # ~15 fps


# ─── ROS 2 Node ────────────────────────────────────────────────────────────
class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_panel')
        self.bridge = CvBridge()

        # Durum
        self.gap       = None
        self.steady    = False
        self.rear_img  = None
        self.over_img  = None
        self.walking   = False
        self.target_gap = 1.2
        self._follower_proc = None

        # Yayıncılar
        self._person_cmd = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self._robot_stop = self.create_publisher(Twist, '/cmd_vel', 10)

        # Aboneler
        self.create_subscription(Float64, '/follower/gap',
                                 lambda m: setattr(self, 'gap', m.data), 10)
        self.create_subscription(Bool, '/follower/steady',
                                 lambda m: setattr(self, 'steady', m.data), 10)
        self.create_subscription(Image, '/person/rear_cam/image_raw',
                                 self._rear_cb, 10)
        self.create_subscription(Image, '/overview_cam/image_raw',
                                 self._over_cb, 10)

        # Kişi hareket timer (50 Hz)
        self._walk_timer = self.create_timer(0.02, self._walk_cb)

    def _rear_cb(self, msg):
        try:
            self.rear_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _over_cb(self, msg):
        try:
            self.over_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _walk_cb(self):
        cmd = Twist()
        if self.walking:
            cmd.linear.x = float(PERSON_SPEED)
        self._person_cmd.publish(cmd)

    def set_target_gap(self, gap: float):
        self.target_gap = gap
        # Çalışan follower'ın parametresini güncelle
        subprocess.Popen(
            ['ros2', 'param', 'set', '/follower_node', 'target_gap', str(gap)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def start_follower(self):
        """Follower node'u başlat."""
        if self._follower_proc and self._follower_proc.poll() is None:
            return  # zaten çalışıyor
        cmd = [
            'ros2', 'run', 'following_study', 'follower_node',
            '--ros-args',
            '-p', f'target_gap:={self.target_gap}',
            '-p', 'kp_gap:=4.0',
            '-p', 'kp_heading:=2.0',
            '-p', 'v_max:=1.5',
        ]
        self._follower_proc = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def stop_follower(self):
        if self._follower_proc and self._follower_proc.poll() is None:
            self._follower_proc.terminate()
            try:
                self._follower_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._follower_proc.kill()
        self._robot_stop.publish(Twist())

    def destroy_node(self):
        self.stop_follower()
        self.walking = False
        self._person_cmd.publish(Twist())
        super().destroy_node()


# ─── Tkinter GUI ───────────────────────────────────────────────────────────
class DemoPanel:
    def __init__(self, node: DemoNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('HRI Takip Mesafesi Demo')
        self.root.configure(bg='#1e1e2e')
        self.root.resizable(False, False)

        self._active_gap = tk.DoubleVar(value=1.2)
        self._build_ui()
        self.root.after(UPDATE_MS, self._update_loop)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _build_ui(self):
        pad = dict(padx=8, pady=4)
        DARK   = '#1e1e2e'
        PANEL  = '#313244'
        ACCENT = '#cba6f7'
        GREEN  = '#a6e3a1'
        RED    = '#f38ba8'
        TEXT   = '#cdd6f4'
        YELLOW = '#f9e2af'

        title_font  = tkfont.Font(family='Helvetica', size=14, weight='bold')
        button_font = tkfont.Font(family='Helvetica', size=12, weight='bold')
        info_font   = tkfont.Font(family='Courier',   size=11)

        # ── Başlık ──────────────────────────────────────────
        tk.Label(self.root, text='HRI Takip Mesafesi Demo',
                 bg=DARK, fg=ACCENT, font=title_font).pack(pady=(12, 4))

        # ── Mesafe butonları ────────────────────────────────
        btn_frame = tk.Frame(self.root, bg=DARK)
        btn_frame.pack(pady=6)

        self._dist_btns = {}
        for d in DISTANCES:
            btn = tk.Button(
                btn_frame,
                text=f'{d:.1f} m',
                width=7, height=2,
                font=button_font,
                bg=PANEL, fg=TEXT,
                activebackground=ACCENT,
                relief='flat', bd=0,
                command=lambda x=d: self._on_distance(x)
            )
            btn.pack(side='left', padx=4)
            self._dist_btns[d] = btn
        self._highlight_btn(1.2)

        # ── Yürüt / Durdur ──────────────────────────────────
        ctrl_frame = tk.Frame(self.root, bg=DARK)
        ctrl_frame.pack(pady=6)

        self._walk_btn = tk.Button(
            ctrl_frame, text='▶  Yürümeye Başla',
            font=button_font, width=18, height=2,
            bg='#40a02b', fg='white',
            activebackground=GREEN,
            relief='flat', bd=0,
            command=self._toggle_walk
        )
        self._walk_btn.pack(side='left', padx=6)

        tk.Button(
            ctrl_frame, text='■  Follower Başlat',
            font=button_font, width=18, height=2,
            bg='#1793d1', fg='white',
            activebackground='#89dceb',
            relief='flat', bd=0,
            command=self._start_follower
        ).pack(side='left', padx=6)

        # ── Durum bilgisi ───────────────────────────────────
        info_frame = tk.Frame(self.root, bg=PANEL, bd=0)
        info_frame.pack(fill='x', padx=12, pady=6)

        self._lbl_gap    = tk.Label(info_frame, text='Mesafe: ---',
                                    bg=PANEL, fg=TEXT, font=info_font, anchor='w')
        self._lbl_gap.grid(row=0, column=0, sticky='w', padx=10, pady=4)

        self._lbl_error  = tk.Label(info_frame, text='Hata:  ---',
                                    bg=PANEL, fg=TEXT, font=info_font, anchor='w')
        self._lbl_error.grid(row=0, column=1, sticky='w', padx=10, pady=4)

        self._lbl_steady = tk.Label(info_frame, text='Durum: Bekleniyor',
                                    bg=PANEL, fg=YELLOW, font=info_font, anchor='w')
        self._lbl_steady.grid(row=0, column=2, sticky='w', padx=10, pady=4)

        # ── Kamera görüntüleri ──────────────────────────────
        cam_frame = tk.Frame(self.root, bg=DARK)
        cam_frame.pack(padx=12, pady=6)

        # Sol: Birinci şahıs
        left = tk.Frame(cam_frame, bg=PANEL)
        left.grid(row=0, column=0, padx=6)
        tk.Label(left, text='🎥  Kişinin Kamerası (Birinci Şahıs)',
                 bg=PANEL, fg=ACCENT,
                 font=tkfont.Font(family='Helvetica', size=10)).pack(pady=4)
        self._rear_lbl = tk.Label(left, bg='black',
                                  width=CAM_W, height=CAM_H)
        self._rear_lbl.pack()

        # Sağ: Genel bakış
        right = tk.Frame(cam_frame, bg=PANEL)
        right.grid(row=0, column=1, padx=6)
        tk.Label(right, text='📷  Genel Bakış',
                 bg=PANEL, fg=ACCENT,
                 font=tkfont.Font(family='Helvetica', size=10)).pack(pady=4)
        self._over_lbl = tk.Label(right, bg='black',
                                  width=CAM_W, height=CAM_H)
        self._over_lbl.pack()

        # ── Alt bilgi ───────────────────────────────────────
        tk.Label(self.root,
                 text='Mesafe = robot önü ile kişi arkası arası  |  '
                      'Hız: kişi 0.8 m/s',
                 bg=DARK, fg='#6c7086',
                 font=tkfont.Font(family='Helvetica', size=9)
                 ).pack(pady=(4, 10))

    def _on_distance(self, d: float):
        self._active_gap.set(d)
        self._highlight_btn(d)
        self.node.set_target_gap(d)

    def _highlight_btn(self, active: float):
        for d, btn in self._dist_btns.items():
            if abs(d - active) < 0.01:
                btn.configure(bg='#cba6f7', fg='#1e1e2e')
            else:
                btn.configure(bg='#313244', fg='#cdd6f4')

    def _toggle_walk(self):
        self.node.walking = not self.node.walking
        if self.node.walking:
            self._walk_btn.configure(text='⏸  Yürümeyi Durdur', bg='#e64553')
        else:
            self._walk_btn.configure(text='▶  Yürümeye Başla',  bg='#40a02b')

    def _start_follower(self):
        self.node.start_follower()

    def _update_loop(self):
        # Durum etiketleri
        gap = self.node.gap
        tgt = self._active_gap.get()

        if gap is not None:
            err = gap - tgt
            self._lbl_gap.configure(
                text=f'Mesafe: {gap:>6.3f} m')
            color = '#a6e3a1' if abs(err) < 0.05 else '#f38ba8'
            self._lbl_error.configure(
                text=f'Hata:  {err:>+6.3f} m', fg=color)
        else:
            self._lbl_gap.configure(text='Mesafe: ---')
            self._lbl_error.configure(text='Hata:  ---')

        if self.node.steady:
            self._lbl_steady.configure(
                text='Durum: ✓ KARARLI', fg='#a6e3a1')
        else:
            self._lbl_steady.configure(
                text='Durum: ⟳ Yakınsıyor...', fg='#f9e2af')

        # Kamera görüntüleri
        self._update_cam(self._rear_lbl, self.node.rear_img)
        self._update_cam(self._over_lbl, self.node.over_img)

        self.root.after(UPDATE_MS, self._update_loop)

    def _update_cam(self, label: tk.Label, frame):
        if frame is None:
            return
        try:
            img = cv2.resize(frame, (CAM_W, CAM_H))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil = PILImage.fromarray(img)
            imgtk = ImageTk.PhotoImage(image=pil)
            label.imgtk = imgtk   # referansı tut
            label.configure(image=imgtk)
        except Exception:
            pass

    def _on_close(self):
        self.node.destroy_node()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# ─── Ana fonksiyon ─────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = DemoNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    panel = DemoPanel(node)
    panel.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
