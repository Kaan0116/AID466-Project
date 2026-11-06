#!/usr/bin/env python3
"""
Sistem değerlendirmesi: her hedef mesafe için 10 tekrar çalıştır,
RMSE, çarpışma sayısı ve hedef kaybı raporla.
"""

import os
import csv
import math
import time
import threading
import argparse
import subprocess
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


OUTPUT_DIR = os.path.expanduser('~/ros2_ws/results/eval')
EVAL_CSV   = os.path.join(OUTPUT_DIR, 'eval_results.csv')
N_REPS     = 10
RUN_DURATION = 25.0   # saniye / tekrar


class RunCollector(Node):
    """Tek bir değerlendirme koşusu için veri toplar."""

    def __init__(self, target_gap: float):
        super().__init__('run_collector')
        self.target_gap = target_gap
        self.times      = []
        self.gaps       = []
        self.collision  = False
        self.target_lost = False
        self._last_gap_time = None

        self.create_subscription(Float64, '/follower/gap',    self._gap_cb,  10)
        self.create_subscription(Float64, '/follower/gap_error', self._err_cb, 5)

    def _gap_cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        g = msg.data
        self.times.append(t)
        self.gaps.append(g)
        self._last_gap_time = t

        # Çarpışma: gap < 0
        if g < 0.0:
            self.collision = True

        # Hedef kaybı: gap > 5 m
        if g > 5.0:
            self.target_lost = True

    def _err_cb(self, msg):
        pass   # ileride genişletilebilir

    def compute_rmse(self):
        if not self.gaps:
            return float('nan')
        errors = [(g - self.target_gap) ** 2 for g in self.gaps]
        return math.sqrt(sum(errors) / len(errors))


def run_single(target_gap: float, rep: int, run_csv: str):
    """Tek bir koşuyu çalıştır, veri topla ve RMSE döndür."""
    rclpy.init()
    collector = RunCollector(target_gap)

    t0 = time.time()
    while rclpy.ok() and (time.time() - t0) < RUN_DURATION:
        rclpy.spin_once(collector, timeout_sec=0.05)

    rmse      = collector.compute_rmse()
    collision = collector.collision
    lost      = collector.target_lost
    times     = list(collector.times)
    gaps      = list(collector.gaps)

    # Koşu verisini CSV'ye yaz
    with open(run_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time', 'gap', 'target_gap'])
        t_base = times[0] if times else 0.0
        for t, g in zip(times, gaps):
            w.writerow([f'{t-t_base:.3f}', f'{g:.4f}', target_gap])

    collector.destroy_node()
    rclpy.shutdown()
    return rmse, collision, lost, times, gaps


def plot_run(times, gaps, target_gap, out_path, title=''):
    t0 = times[0] if times else 0.0
    ts = [t - t0 for t in times]
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, gaps, linewidth=1.2, label='Ölçülen mesafe')
    ax.axhline(target_gap, color='red', linestyle='--', linewidth=1.5,
               label=f'Hedef {target_gap:.1f} m')
    ax.fill_between(ts,
                    target_gap - 0.03, target_gap + 0.03,
                    alpha=0.15, color='green', label='±0.03 m band')
    ax.set_xlabel('Zaman (s)')
    ax.set_ylabel('Mesafe (m)')
    ax.set_title(title or f'gap={target_gap:.1f}m')
    ax.legend()
    ax.grid(True, alpha=0.4)
    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gaps', nargs='+', type=float,
                        default=[0.6, 1.2, 2.0])
    parser.add_argument('--reps', type=int, default=N_REPS)
    args = parser.parse_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    all_rows = []

    for tg in args.gaps:
        rmse_list = []
        col_count = 0
        lost_count = 0
        all_times_gaps = []

        for rep in range(1, args.reps + 1):
            print(f'  gap={tg:.1f}m  tekrar {rep}/{args.reps}...')
            run_csv = os.path.join(OUTPUT_DIR, f'run_gap{tg:.1f}_rep{rep:02d}.csv')

            # Bu çağrı normalde ayrı bir process'te çalışmalı
            # (Gazebo zaten ayakta, follower yeniden başlatılıyor)
            rmse, col, lost, times, gaps = run_single(tg, rep, run_csv)

            rmse_list.append(rmse)
            if col:  col_count  += 1
            if lost: lost_count += 1
            all_times_gaps.append((times, gaps))

        avg_rmse = float(np.mean([r for r in rmse_list if not math.isnan(r)]))

        # Gap-zaman grafiği (ilk tekrar)
        if all_times_gaps[0][0]:
            plot_path = os.path.join(OUTPUT_DIR, f'gap_plot_{tg:.1f}m.png')
            t0_ts, g0_gs = all_times_gaps[0]
            plot_run(t0_ts, g0_gs, tg, plot_path,
                     title=f'gap={tg:.1f}m — 1. tekrar')
            print(f'  Grafik: {plot_path}')

        all_rows.append({
            'target_gap': tg,
            'n_reps': args.reps,
            'gap_rmse': f'{avg_rmse:.4f}',
            'collisions': col_count,
            'target_lost': lost_count,
        })
        print(f'  RMSE={avg_rmse:.4f}  çarpışma={col_count}  kayıp={lost_count}')

    with open(EVAL_CSV, 'w', newline='') as f:
        fieldnames = ['target_gap', 'n_reps', 'gap_rmse', 'collisions', 'target_lost']
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(all_rows)

    print(f'\nSonuçlar: {EVAL_CSV}')


if __name__ == '__main__':
    main()
