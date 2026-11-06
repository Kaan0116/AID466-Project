#!/usr/bin/env python3
"""
demo_panel.py — HRI Demo Paneli (kompakt)
Kullanım: python3 demo_panel.py
"""

import threading
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import tkinter as tk
from tkinter import font as tkfont
from PIL import Image as PILImage, ImageTk

DISTANCES    = [0.6, 0.9, 1.2, 1.5, 2.0]
PERSON_SPEED = 0.8
CAM_W, CAM_H = 480, 270
UPDATE_MS    = 100   # 10 fps


class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_panel')
        self.bridge     = CvBridge()
        self.gap        = None
        self.steady     = False
        self.rear_img   = None
        self.over_img   = None
        self.walking    = False
        self.target_gap = 1.2
        self._follower_proc = None

        self._person_cmd = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self._robot_stop = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Float64, '/follower/gap',
                                 lambda m: setattr(self, 'gap', m.data), 10)
        self.create_subscription(Bool, '/follower/steady',
                                 lambda m: setattr(self, 'steady', m.data), 10)
        self.create_subscription(Image, '/person/rear_cam/image_raw',
                                 self._rear_cb, 10)
        self.create_subscription(Image, '/overview_cam/image_raw',
                                 self._over_cb, 10)

        self.create_timer(0.02, self._walk_cb)

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
        subprocess.Popen(
            ['ros2', 'param', 'set', '/follower_node', 'target_gap', str(gap)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def start_follower(self):
        if self._follower_proc and self._follower_proc.poll() is None:
            return
        cmd = ['ros2', 'run', 'following_study', 'follower_node',
               '--ros-args',
               '-p', f'target_gap:={self.target_gap}',
               '-p', 'kp_gap:=4.0',
               '-p', 'kp_heading:=2.0',
               '-p', 'v_max:=1.5']
        self._follower_proc = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def stop_follower(self):
        if self._follower_proc and self._follower_proc.poll() is None:
            self._follower_proc.terminate()
        self._robot_stop.publish(Twist())

    def destroy_node(self):
        self.stop_follower()
        self.walking = False
        self._person_cmd.publish(Twist())
        super().destroy_node()


class DemoPanel:
    DARK   = '#1e1e2e'
    PANEL  = '#313244'
    ACCENT = '#cba6f7'
    GREEN  = '#a6e3a1'
    RED    = '#f38ba8'
    TEXT   = '#cdd6f4'
    YELLOW = '#f9e2af'

    def __init__(self, node: DemoNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('HRI Demo')
        self.root.configure(bg=self.DARK)
        self.root.resizable(False, False)
        self._active_gap = tk.DoubleVar(value=1.2)
        self._cam_mode   = 0   # 0=rear, 1=overview
        self._build_ui()
        self.root.after(UPDATE_MS, self._update_loop)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _build_ui(self):
        bf = tkfont.Font(family='Helvetica', size=11, weight='bold')
        sf = tkfont.Font(family='Courier',   size=10)

        # Başlık
        tk.Label(self.root, text='HRI Takip Mesafesi Demo',
                 bg=self.DARK, fg=self.ACCENT,
                 font=tkfont.Font(family='Helvetica', size=12, weight='bold')
                 ).pack(pady=(8, 2))

        # Mesafe butonları
        bf_frame = tk.Frame(self.root, bg=self.DARK)
        bf_frame.pack(pady=4)
        self._dist_btns = {}
        for d in DISTANCES:
            b = tk.Button(bf_frame, text=f'{d:.1f}m', width=6, height=1,
                          font=bf, bg=self.PANEL, fg=self.TEXT,
                          relief='flat', bd=0,
                          command=lambda x=d: self._on_distance(x))
            b.pack(side='left', padx=3)
            self._dist_btns[d] = b
        self._highlight_btn(1.2)

        # Kontrol butonları
        cf = tk.Frame(self.root, bg=self.DARK)
        cf.pack(pady=4)
        self._walk_btn = tk.Button(cf, text='▶ Yürüt', font=bf, width=10,
                                   bg='#40a02b', fg='white', relief='flat', bd=0,
                                   command=self._toggle_walk)
        self._walk_btn.pack(side='left', padx=4)
        tk.Button(cf, text='⚙ Follower', font=bf, width=10,
                  bg='#1793d1', fg='white', relief='flat', bd=0,
                  command=self.node.start_follower).pack(side='left', padx=4)

        # Durum
        sf_frame = tk.Frame(self.root, bg=self.PANEL)
        sf_frame.pack(fill='x', padx=10, pady=4)
        self._lbl_gap    = tk.Label(sf_frame, text='Mesafe: ---',
                                    bg=self.PANEL, fg=self.TEXT, font=sf, anchor='w')
        self._lbl_gap.grid(row=0, column=0, padx=8, pady=3, sticky='w')
        self._lbl_error  = tk.Label(sf_frame, text='Hata: ---',
                                    bg=self.PANEL, fg=self.TEXT, font=sf, anchor='w')
        self._lbl_error.grid(row=0, column=1, padx=8, pady=3, sticky='w')
        self._lbl_steady = tk.Label(sf_frame, text='Durum: Bekliyor',
                                    bg=self.PANEL, fg=self.YELLOW, font=sf, anchor='w')
        self._lbl_steady.grid(row=0, column=2, padx=8, pady=3, sticky='w')

        # Kamera — tek pencere, butonla değiştir
        cam_outer = tk.Frame(self.root, bg=self.DARK)
        cam_outer.pack(padx=10, pady=4)

        cam_tabs = tk.Frame(cam_outer, bg=self.DARK)
        cam_tabs.pack()
        self._cam_btns = []
        for i, label in enumerate(['Kişi Kamerası', 'Genel Bakış']):
            b = tk.Button(cam_tabs, text=label, font=sf, width=14,
                          bg=self.ACCENT if i == 0 else self.PANEL,
                          fg=self.DARK if i == 0 else self.TEXT,
                          relief='flat', bd=0,
                          command=lambda x=i: self._switch_cam(x))
            b.pack(side='left', padx=2, pady=2)
            self._cam_btns.append(b)

        self._cam_lbl = tk.Label(cam_outer, bg='black',
                                 width=CAM_W, height=CAM_H)
        self._cam_lbl.pack()

    def _switch_cam(self, idx):
        self._cam_mode = idx
        for i, b in enumerate(self._cam_btns):
            if i == idx:
                b.configure(bg=self.ACCENT, fg=self.DARK)
            else:
                b.configure(bg=self.PANEL, fg=self.TEXT)

    def _on_distance(self, d):
        self._active_gap.set(d)
        self._highlight_btn(d)
        self.node.set_target_gap(d)

    def _highlight_btn(self, active):
        for d, b in self._dist_btns.items():
            if abs(d - active) < 0.01:
                b.configure(bg=self.ACCENT, fg=self.DARK)
            else:
                b.configure(bg=self.PANEL, fg=self.TEXT)

    def _toggle_walk(self):
        self.node.walking = not self.node.walking
        if self.node.walking:
            self._walk_btn.configure(text='⏸ Durdur', bg=self.RED)
        else:
            self._walk_btn.configure(text='▶ Yürüt',  bg='#40a02b')

    def _update_loop(self):
        gap = self.node.gap
        tgt = self._active_gap.get()
        if gap is not None:
            err = gap - tgt
            self._lbl_gap.configure(text=f'Mesafe: {gap:.3f}m')
            self._lbl_error.configure(
                text=f'Hata: {err:+.3f}m',
                fg=self.GREEN if abs(err) < 0.05 else self.RED)
        if self.node.steady:
            self._lbl_steady.configure(text='Durum: ✓ KARARLI', fg=self.GREEN)
        else:
            self._lbl_steady.configure(text='Durum: ⟳ Yakınlaşıyor', fg=self.YELLOW)

        frame = self.node.rear_img if self._cam_mode == 0 else self.node.over_img
        if frame is not None:
            try:
                img = cv2.resize(frame, (CAM_W, CAM_H))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                pil = PILImage.fromarray(img)
                imgtk = ImageTk.PhotoImage(image=pil)
                self._cam_lbl.imgtk = imgtk
                self._cam_lbl.configure(image=imgtk)
            except Exception:
                pass

        self.root.after(UPDATE_MS, self._update_loop)

    def _on_close(self):
        self.node.destroy_node()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = DemoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    DemoPanel(node).run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
