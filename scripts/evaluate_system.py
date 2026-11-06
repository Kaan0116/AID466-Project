#!/usr/bin/env python3
"""
evaluate_system.py — STEP 7: system evaluation, `reps` runs per gap (study_params.yaml: study.eval).

    ros2 run following_study evaluate_system.py            # all gaps / reps from the YAML
    ros2 run following_study evaluate_system.py --gaps 1.2 --reps 2

Every run is independent and reproducible:
  * a fresh headless Gazebo (simulation.launch.py gui:=false) with seed = base_seed + 100*i + rep
  * the robot start pose is jittered with a numpy RNG seeded with the same seed
  * person walks at study.eval.person_speed, follower_node is the only controller
  * rosbag2 + follower CSV are recorded per run
Metrics, computed from ground-truth poses (/person/odom, /robot/odom) with the shared gap geometry:
  rmse_steady  RMSE of (gap - target) for t >= settle_time  (rmse_all: whole run)
  collision    any /robot/bumper contact with the person, or gap <= 0
  target_lost  |bearing to the person| > lost_bearing (robot camera half-FOV) or
               gap > target + lost_gap_margin
Outputs (out_dir): eval_runs.csv (one row per run), eval_results.csv (per gap),
  gap_<g>/rep_<nn>/{run.csv, follower.csv, bag/, launch.log, gap_vs_time.png},
  gap_vs_time_<g>m.png (all reps overlaid)
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.utilities import remove_ros_args  # noqa: E402
from gazebo_msgs.msg import ContactsState  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from following_study import study_config  # noqa: E402
from following_study.geometry import relation  # noqa: E402


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Collector(Node):
    """Pairs every robot odometry message with the latest person odometry."""

    def __init__(self, fp):
        super().__init__('eval_collector')
        self.fp = fp
        self.create_subscription(Odometry, '/person/odom', self._person_cb, 50)
        self.create_subscription(Odometry, '/robot/odom', self._robot_cb, 50)
        self.create_subscription(ContactsState, '/robot/bumper', self._bumper_cb, 50)
        self.create_subscription(Bool, '/person/walking', self._walk_cb, 10)
        self.reset()

    def reset(self):
        self.person = None
        self.samples = []      # (t, px, py, pyaw, rx, ry, ryaw, gap, lateral, bearing, walking)
        self.contacts = set()
        self.walking = False

    def _person_cb(self, m):
        self.person = m

    def _walk_cb(self, m):
        self.walking = m.data

    def _robot_cb(self, m):
        if self.person is None:
            return
        rel = relation(self.person.pose.pose, m.pose.pose,
                       self.fp['person_back_offset'], self.fp['robot_front_offset'])
        pp, rp = self.person.pose.pose.position, m.pose.pose.position
        self.samples.append((stamp_s(m), pp.x, pp.y, rel.person_yaw, rp.x, rp.y, rel.robot_yaw,
                             rel.gap, rel.lateral, rel.bearing, self.walking))

    def _bumper_cb(self, m):
        for st in m.states:
            other = st.collision2_name if 'study_robot' in st.collision1_name else st.collision1_name
            self.contacts.add(other.split('::')[0])


def start_sim(args_list, log_path):
    log = open(log_path, 'w')
    return subprocess.Popen(['ros2', 'launch', 'following_study', 'simulation.launch.py']
                            + args_list, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True), log


def stop_sim(proc, log):
    if proc.poll() is None:
        for sig, wait in ((signal.SIGINT, 30), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
    log.close()


def run_once(node, target_gap, seed, ev, run_dir, cfg_path):
    rng = np.random.default_rng(seed)
    jitter = {'robot_dx': round(float(rng.uniform(-ev['jitter_x'], ev['jitter_x'])), 4),
              'robot_y': round(float(rng.uniform(-ev['jitter_y'], ev['jitter_y'])), 4),
              'robot_yaw': round(float(rng.uniform(-ev['jitter_yaw'], ev['jitter_yaw'])), 4)}
    os.makedirs(run_dir, exist_ok=True)
    sim_args = [f'target_gap:={target_gap}', f'person_speed:={ev["person_speed"]}',
                f'person_x:={ev["person_x"]}', 'gui:=false', f'seed:={seed}',
                f'params_file:={cfg_path}', 'record_bag:=true',
                f'bag_path:={os.path.join(run_dir, "bag")}',
                f'log_csv:={os.path.join(run_dir, "follower.csv")}'] + \
               [f'{k}:={v:.4f}' for k, v in jitter.items()]
    node.reset()
    proc, log = start_sim(sim_args, os.path.join(run_dir, 'launch.log'))
    t_wall0 = time.time()
    try:
        while rclpy.ok() and time.time() - t_wall0 < ev['duration'] * 10 + 120:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.samples and node.samples[-1][0] - node.samples[0][0] >= ev['duration']:
                break
            if proc.poll() is not None:
                break
    finally:
        stop_sim(proc, log)
    return jitter, list(node.samples), set(node.contacts)


def metrics(samples, contacts, target_gap, ev):
    if len(samples) < 10:
        return None
    a = np.array([s[:10] for s in samples], dtype=float)
    t = a[:, 0] - a[0, 0]
    gap, bearing = a[:, 7], a[:, 9]
    err = gap - target_gap
    steady = t >= ev['settle_time']
    lost_mask = (np.abs(bearing) > ev['lost_bearing']) | (gap > target_gap + ev['lost_gap_margin'])
    dt = np.diff(t, prepend=t[0])
    return {
        'n_samples': len(t), 'duration_s': f'{t[-1]:.2f}',
        'rmse_steady': f'{math.sqrt(np.mean(err[steady] ** 2)):.4f}' if steady.any() else 'nan',
        'rmse_all': f'{math.sqrt(np.mean(err ** 2)):.4f}',
        'mean_err_steady': f'{np.mean(err[steady]):+.4f}' if steady.any() else 'nan',
        'max_abs_err_steady': f'{np.max(np.abs(err[steady])):.4f}' if steady.any() else 'nan',
        'min_gap': f'{gap.min():.4f}',
        'collision': bool('person' in contacts or gap.min() <= 0.0),
        'contacts': ';'.join(sorted(contacts)),
        'target_lost': bool(lost_mask.any()),
        'lost_time_s': f'{float(np.sum(dt[lost_mask])):.2f}',
    }, t, gap


def plot(series, target_gap, title, path, tol=0.03):
    fig, ax = plt.subplots(figsize=(10, 4))
    for label, t, g in series:
        ax.plot(t, g, linewidth=1.0, label=label)
    ax.axhline(target_gap, color='red', linestyle='--', linewidth=1.2,
               label=f'target {target_gap:.1f} m')
    ax.axhspan(target_gap - tol, target_gap + tol, alpha=0.15, color='green',
               label=f'±{tol} m')
    ax.set_xlabel('simulation time [s]')
    ax.set_ylabel('gap [m]')
    ax.set_title(title)
    ax.grid(True, alpha=0.4)
    ax.legend(fontsize=7, ncol=2 if len(series) > 4 else 1)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gaps', nargs='+', type=float)
    ap.add_argument('--reps', type=int)
    ap.add_argument('--out-dir', default=os.path.expanduser('~/ros2_ws/results/eval'))
    ap.add_argument('--params-file', default='')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    cfg_path = args.params_file or study_config.default_config_path()
    cfg = study_config.load(cfg_path)
    fp = study_config.follower_params(cfg)
    ev = study_config.study(cfg)['eval']
    gaps = args.gaps or [float(g) for g in ev['gaps']]
    reps = args.reps or int(ev['reps'])
    out = os.path.expanduser(args.out_dir)
    os.makedirs(out, exist_ok=True)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Collector(fp)
    run_rows, gap_rows = [], []
    try:
        for gi, g in enumerate(gaps):
            series, rows = [], []
            for rep in range(1, reps + 1):
                seed = int(ev['base_seed']) + 100 * gi + rep
                run_dir = os.path.join(out, f'gap_{g:.1f}', f'rep_{rep:02d}')
                print(f'[eval] gap={g:.1f} m  rep {rep}/{reps}  seed={seed}', flush=True)
                jitter, samples, contacts = run_once(node, g, seed, ev, run_dir, cfg_path)
                res = metrics(samples, contacts, g, ev)
                if res is None:
                    print(f'[eval]   FAILED: no data (see {run_dir}/launch.log)', flush=True)
                    row = {'target_gap': g, 'rep': rep, 'seed': seed, **jitter, 'status': 'failed'}
                else:
                    m, t, gap = res
                    with open(os.path.join(run_dir, 'run.csv'), 'w', newline='') as f:
                        w = csv.writer(f)
                        w.writerow(['t', 'person_x', 'person_y', 'person_yaw', 'robot_x',
                                    'robot_y', 'robot_yaw', 'gap', 'lateral', 'bearing', 'walking'])
                        for s in samples:
                            w.writerow([f'{s[0] - samples[0][0]:.3f}'] +
                                       [f'{v:.4f}' for v in s[1:10]] + [int(s[10])])
                    plot([(f'rep {rep}', t, gap)], g, f'gap={g:.1f} m, rep {rep}, seed {seed}',
                         os.path.join(run_dir, 'gap_vs_time.png'))
                    series.append((f'rep {rep}', t, gap))
                    row = {'target_gap': g, 'rep': rep, 'seed': seed, **jitter,
                           'status': 'ok', **m}
                    print(f'[eval]   rmse_steady={m["rmse_steady"]} collision={m["collision"]} '
                          f'lost={m["target_lost"]}', flush=True)
                rows.append(row)
                run_rows.append(row)
            if series:
                plot(series, g, f'gap={g:.1f} m, {len(series)} runs',
                     os.path.join(out, f'gap_vs_time_{g:.1f}m.png'))
            ok = [r for r in rows if r['status'] == 'ok']
            rm = [float(r['rmse_steady']) for r in ok if r['rmse_steady'] != 'nan']
            gap_rows.append({
                'target_gap': g, 'n_runs': len(rows), 'n_ok': len(ok),
                'rmse_mean': f'{np.mean(rm):.4f}' if rm else 'nan',
                'rmse_std': f'{np.std(rm, ddof=1):.4f}' if len(rm) > 1 else 'nan',
                'rmse_max': f'{np.max(rm):.4f}' if rm else 'nan',
                'collisions': sum(1 for r in ok if r['collision']),
                'target_lost': sum(1 for r in ok if r['target_lost']),
            })
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if run_rows:
            keys = []
            for r in run_rows:
                keys += [k for k in r if k not in keys]
            with open(os.path.join(out, 'eval_runs.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                w.writerows(run_rows)
        if gap_rows:
            with open(os.path.join(out, 'eval_results.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(gap_rows[0].keys()))
                w.writeheader()
                w.writerows(gap_rows)
            print(f'[eval] results: {os.path.join(out, "eval_results.csv")}')


if __name__ == '__main__':
    main()
