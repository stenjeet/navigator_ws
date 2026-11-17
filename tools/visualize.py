#!/usr/bin/env python3
"""Record live simulation data for N seconds and render PNG figures.

Run alongside ``simulation.launch.py use_rviz:=false``. Produces:
  result_map.png   top view: polygon + trajectory + accumulated lidar points + robot
  result_scan.png  one scan: ideal vs noisy lidar points (report fig. 4.2 analogue)
  result_uwb.png   per-anchor ideal vs noisy UWB ranges over time

Collected data is also dumped to ``plot_data.npz``, so the figures can be
re-rendered without running the simulation again:

    python3 tools/visualize.py --replot

Both ``rclpy`` (ROS 2 environment) and ``matplotlib``/``numpy`` are required.
"""

import argparse
import math
import os
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
import numpy as np

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT_DIR = os.path.join(WORKSPACE_DIR, 'results')
DEFAULT_DURATION = 62.0

OUT_DIR = DEFAULT_OUT_DIR
DATA_PATH = os.path.join(OUT_DIR, 'plot_data.npz')

SCAN_FIELDS = ('ranges', 'angle_min', 'angle_increment', 'range_min', 'range_max')


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Viz:
    """Plain data container; filled by the recorder or loaded from plot_data.npz."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.traj = []
        self.map_points = []
        self.last_ideal = None
        self.last_noisy = None
        self.uwb = []  # (t, ideal[4], noisy[4])
        self.last_uwb_ideal = None

    # ---- (de)serialization ----
    def save(self, path=DATA_PATH):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        d = {
            'pose': np.array([self.x, self.y, self.yaw], dtype=float),
            'have_odom': np.array([self.have_odom]),
            'traj': np.array(self.traj, dtype=float).reshape(-1, 2),
            'map_points': np.array(self.map_points, dtype=float).reshape(-1, 2),
            'uwb_t': np.array([s[0] for s in self.uwb], dtype=float),
            'uwb_ideal': np.array([s[1] for s in self.uwb], dtype=float).reshape(-1, 4),
            'uwb_noisy': np.array([s[2] for s in self.uwb], dtype=float).reshape(-1, 4),
        }
        for tag, scan in (('ideal', self.last_ideal), ('noisy', self.last_noisy)):
            for field in SCAN_FIELDS:
                value = getattr(scan, field) if scan is not None else np.nan
                d['scan_%s_%s' % (tag, field)] = np.array(value)
        np.savez_compressed(path, **d)
        print('data saved: %s' % path)

    @classmethod
    def load(cls, path=DATA_PATH):
        z = np.load(path)
        obj = cls()
        obj.x, obj.y, obj.yaw = (float(v) for v in z['pose'])
        obj.have_odom = bool(z['have_odom'][0])
        obj.traj = [tuple(p) for p in z['traj']]
        obj.map_points = [tuple(p) for p in z['map_points']]
        obj.uwb = [(float(t), list(i), list(n))
                   for t, i, n in zip(z['uwb_t'], z['uwb_ideal'], z['uwb_noisy'])]

        class _Scan:
            pass

        for tag in ('ideal', 'noisy'):
            scan = _Scan()
            for field in SCAN_FIELDS:
                key = 'scan_%s_%s' % (tag, field)
                value = z[key]
                scan.__dict__[field] = list(value) if field == 'ranges' else float(value)
            if np.isfinite(scan.range_min):
                setattr(obj, 'last_' + tag, scan)
        return obj


class Recorder:
    """ROS subscriber wrapper that fills a Viz container."""

    def __init__(self, viz, node, world):
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32MultiArray

        self.viz = viz
        self.lidar_offset = world.LIDAR_OFFSET
        node.create_subscription(Odometry, '/ground_truth/odom', self.cb_odom, 20)
        node.create_subscription(LaserScan, '/scan_ideal', self.cb_ideal, 20)
        node.create_subscription(LaserScan, '/scan_noisy', self.cb_noisy, 20)
        node.create_subscription(Float32MultiArray, '/uwb/ideal_ranges', self.cb_uwb_ideal, 20)
        node.create_subscription(Float32MultiArray, '/uwb/noisy_ranges', self.cb_uwb_noisy, 20)

    def cb_odom(self, msg):
        v = self.viz
        v.x = msg.pose.pose.position.x
        v.y = msg.pose.pose.position.y
        v.yaw = quat_to_yaw(msg.pose.pose.orientation)
        v.have_odom = True
        if not v.traj or math.hypot(v.x - v.traj[-1][0], v.y - v.traj[-1][1]) > 0.02:
            v.traj.append((v.x, v.y))

    def lidar_pose(self):
        v = self.viz
        dx, dy, _ = self.lidar_offset
        c, s = math.cos(v.yaw), math.sin(v.yaw)
        return v.x + dx * c - dy * s, v.y + dx * s + dy * c

    def cb_ideal(self, msg):
        v = self.viz
        if not v.have_odom:
            return
        v.last_ideal = msg
        lx, ly = self.lidar_pose()
        for j in range(0, len(msg.ranges), 3):  # subsample to keep memory low
            r = msg.ranges[j]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            phi = v.yaw + msg.angle_min + j * msg.angle_increment
            v.map_points.append((lx + r * math.cos(phi), ly + r * math.sin(phi)))

    def cb_noisy(self, msg):
        self.viz.last_noisy = msg

    def cb_uwb_ideal(self, msg):
        self.viz.last_uwb_ideal = list(msg.data)

    def cb_uwb_noisy(self, msg):
        v = self.viz
        if v.last_uwb_ideal is not None:
            v.uwb.append((time.time(), list(v.last_uwb_ideal), list(msg.data)))


def render(v, world):
    os.makedirs(OUT_DIR, exist_ok=True)
    wall_c = '#33333f'
    obst_c = '#9440a0'

    # ---- result_map.png ----
    fig, ax = plt.subplots(figsize=(8, 8))
    for (xmin, xmax, ymin, ymax) in world.WALLS:
        ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                               facecolor=wall_c, edgecolor='none'))
    for (xmin, xmax, ymin, ymax, _h) in world.BOX_OBSTACLES:
        ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                               facecolor=obst_c, edgecolor='none'))
    for (cx, cy, r, _h) in world.CYLINDER_OBSTACLES:
        ax.add_patch(Circle((cx, cy), r, facecolor='#d92626', edgecolor='none'))
    tri = world.TRIANGLE_OBSTACLE['vertices']
    ax.add_patch(Polygon(tri, closed=True, facecolor='#26b340', edgecolor='none'))
    for (name, lx, ly, lr, _h, col) in world.LANDMARKS:
        ax.add_patch(Circle((lx, ly), lr, facecolor=col, edgecolor='k', linewidth=0.5))
    for (name, ax_, ay_, az_) in world.UWB_ANCHORS:
        ax.plot(ax_, ay_, marker='*', markersize=12, color='orange', markeredgecolor='k')
        ax.annotate(name, (ax_, ay_), textcoords='offset points', xytext=(6, 6), fontsize=9)

    if v.map_points:
        xs = [p[0] for p in v.map_points]
        ys = [p[1] for p in v.map_points]
        ax.scatter(xs, ys, s=0.5, c='black', alpha=0.6, label='точки лидара (идеал)')
    if v.traj:
        tx = [p[0] for p in v.traj]
        ty = [p[1] for p in v.traj]
        ax.plot(tx, ty, color='#19ff00', linewidth=2.0, label='истинная траектория')
    if v.have_odom:
        ax.plot(v.x, v.y, marker='o', color='blue', markersize=8, label='робот')
        ax.arrow(v.x, v.y, 0.4 * math.cos(v.yaw), 0.4 * math.sin(v.yaw),
                 head_width=0.15, color='blue')
    ax.set_xlim(-0.5, 8.5)
    ax.set_ylim(-0.5, 8.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x, м')
    ax.set_ylabel('y, м')
    ax.set_title('Тестовый полигон, истинная траектория и карта лидара (вид сверху)',
                 fontsize=12)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=3, fontsize=8)
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, 'result_map.png'), dpi=130)
    plt.close(fig)

    # ---- result_scan.png ----
    if v.last_ideal is not None and v.last_noisy is not None:
        fig, ax = plt.subplots(figsize=(7, 7))

        def to_points(scan, color, label):
            xs, ys = [], []
            for j, r in enumerate(scan.ranges):
                if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                    continue
                a = scan.angle_min + j * scan.angle_increment
                xs.append(r * math.cos(a))
                ys.append(r * math.sin(a))
            ax.scatter(xs, ys, s=6, c=color, label=label, alpha=0.85)

        to_points(v.last_ideal, '#ff5500', 'идеальные')
        to_points(v.last_noisy, '#00e5ff', 'зашумлённые')
        ax.plot(0, 0, marker='^', color='blue', markersize=10, label='лидар')
        ax.set_aspect('equal')
        ax.set_xlabel('x, м')
        ax.set_ylabel('y, м')
        ax.set_title('Скан лидара: идеальные (оранжевый) и зашумлённые (бирюзовый) точки\n'
                     'в системе координат лидара', fontsize=11)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.2)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, 'result_scan.png'), dpi=130)
        plt.close(fig)

    # ---- result_uwb.png ----
    if v.uwb:
        t0 = v.uwb[0][0]
        ts = [s[0] - t0 for s in v.uwb]
        fig, axes = plt.subplots(2, 2, figsize=(10, 7))
        for i, (name, _, _, _) in enumerate(world.UWB_ANCHORS):
            ax = axes[i // 2][i % 2]
            ideal = [s[1][i] if i < len(s[1]) else float('nan') for s in v.uwb]
            noisy = [s[2][i] if i < len(s[2]) else float('nan') for s in v.uwb]
            ax.plot(ts, ideal, color='#ff5500', label='идеальные')
            ax.plot(ts, noisy, '.', color='#00a0c0', markersize=4, label='зашумлённые')
            ax.set_title('Якорь %s' % name)
            ax.set_xlabel('t, с')
            ax.set_ylabel('дальность, м')
            ax.grid(True, alpha=0.2)
            ax.legend(fontsize=7)
        fig.suptitle('Дальности UWB: идеальные и зашумлённые измерения')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, 'result_uwb.png'), dpi=130)
        plt.close(fig)

    print('rendered: map_points=%d traj=%d uwb_samples=%d' %
          (len(v.map_points), len(v.traj), len(v.uwb)))
    print('output dir: %s' % OUT_DIR)


def record(duration):
    import rclpy
    from rclpy.node import Node
    from navigator_sim import world

    rclpy.init()
    node = Node('viz_recorder')
    viz = Viz()
    Recorder(viz, node, world)
    node.get_logger().info('recording for %.1f s...' % duration)
    end = time.time() + duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return viz


def main():
    global OUT_DIR, DATA_PATH

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION,
                        help='recording length in seconds (default: %(default)s)')
    parser.add_argument('--out', default=DEFAULT_OUT_DIR,
                        help='output directory (default: <workspace>/results)')
    parser.add_argument('--replot', action='store_true',
                        help='render from the saved plot_data.npz, no simulation needed')
    args = parser.parse_args()

    OUT_DIR = os.path.abspath(args.out)
    DATA_PATH = os.path.join(OUT_DIR, 'plot_data.npz')

    from navigator_sim import world

    if args.replot:
        viz = Viz.load()
    else:
        viz = record(args.duration)
        viz.save()
    render(viz, world)


if __name__ == '__main__':
    main()
