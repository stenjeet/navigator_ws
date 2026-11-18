#!/usr/bin/env python3
"""Record ground-truth and EKF poses and report localization errors (stage 5).

Run alongside the simulation with the EKF enabled::

    ros2 launch navigator_bringup localization.launch.py use_rviz:=false &
    python3 tools/localization_metrics.py --duration 62

Produces in ``results/``:

    result_localization_trajectory.png   true vs estimated trajectory
    result_localization_error.png        x, y and yaw errors over time
    localization_data.npz                raw samples for ``--replot``

Re-render without the simulation::

    python3 tools/localization_metrics.py --replot
"""

import argparse
import math
import os
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT_DIR = os.path.join(WORKSPACE_DIR, 'results')
DEFAULT_DURATION = 62.0
DATA_NAME = 'localization_data.npz'


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _stamp_seconds(msg):
    stamp = msg.header.stamp
    if stamp.sec == 0 and stamp.nanosec == 0:
        return time.time()
    return stamp.sec + stamp.nanosec * 1e-9


def record(duration):
    """Collect ``(t, x, y, yaw)`` samples from ground truth and the estimate."""
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = Node('localization_metrics')
    ground_truth = []
    estimate = []

    def gt_callback(msg):
        pose = msg.pose.pose
        ground_truth.append((
            _stamp_seconds(msg),
            float(pose.position.x),
            float(pose.position.y),
            quat_to_yaw(pose.orientation),
        ))

    def estimate_callback(msg):
        pose = msg.pose.pose
        estimate.append((
            _stamp_seconds(msg),
            float(pose.position.x),
            float(pose.position.y),
            quat_to_yaw(pose.orientation),
        ))

    node.create_subscription(Odometry, '/ground_truth/odom', gt_callback, 50)
    node.create_subscription(Odometry, '/estimate/odom', estimate_callback, 50)
    node.get_logger().info('recording %.1f s of /ground_truth/odom and /estimate/odom' % duration)

    started = time.monotonic()
    try:
        while time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return np.asarray(ground_truth, dtype=float), np.asarray(estimate, dtype=float)


def align(ground_truth, estimate):
    """Match every estimate sample to the nearest ground-truth sample."""
    if len(ground_truth) < 2 or len(estimate) < 2:
        raise SystemExit(
            'not enough samples (ground truth=%d, estimate=%d); is the EKF running?'
            % (len(ground_truth), len(estimate))
        )

    order = np.argsort(ground_truth[:, 0])
    ground_truth = ground_truth[order]
    gt_time = ground_truth[:, 0]

    index = np.searchsorted(gt_time, estimate[:, 0])
    index = np.clip(index, 1, len(gt_time) - 1)
    left = index - 1
    right = index
    choose = np.where(
        np.abs(gt_time[right] - estimate[:, 0]) < np.abs(gt_time[left] - estimate[:, 0]),
        right,
        left,
    )
    matched = ground_truth[choose]

    time_rel = estimate[:, 0] - estimate[0, 0]
    error_x = estimate[:, 1] - matched[:, 1]
    error_y = estimate[:, 2] - matched[:, 2]
    error_yaw = np.array(
        [wrap_angle(value) for value in estimate[:, 3] - matched[:, 3]]
    )
    return time_rel, error_x, error_y, error_yaw, matched


def compute_metrics(error_x, error_y, error_yaw):
    position = np.hypot(error_x, error_y)
    return {
        'rmse_x': float(np.sqrt(np.mean(error_x ** 2))),
        'rmse_y': float(np.sqrt(np.mean(error_y ** 2))),
        'rmse_pos': float(np.sqrt(np.mean(position ** 2))),
        'rmse_yaw': float(np.sqrt(np.mean(error_yaw ** 2))),
        'max_pos': float(np.max(position)),
        'max_yaw': float(np.max(np.abs(error_yaw))),
    }


def render(ground_truth, estimate, time_rel, error_x, error_y, error_yaw, metrics, out_dir):
    os.makedirs(out_dir, exist_ok=True)

    # Trajectory: truth vs estimate.
    figure, axes = plt.subplots(figsize=(7.0, 7.0))
    axes.plot(ground_truth[:, 1], ground_truth[:, 2], linewidth=2.0, label='ground truth')
    axes.plot(estimate[:, 1], estimate[:, 2], linewidth=1.4, label='EKF estimate')
    axes.scatter(
        [ground_truth[0, 1]], [ground_truth[0, 2]],
        marker='o', s=45, label='start',
    )
    axes.set_aspect('equal', adjustable='box')
    axes.set_xlabel('x [m]')
    axes.set_ylabel('y [m]')
    axes.set_title('Localization: true vs estimated trajectory')
    axes.grid(True)
    axes.legend()
    figure.tight_layout()
    trajectory_path = os.path.join(out_dir, 'result_localization_trajectory.png')
    figure.savefig(trajectory_path, dpi=140)
    plt.close(figure)

    # Errors over time.
    figure, (position_axes, yaw_axes) = plt.subplots(
        2, 1, figsize=(9.0, 6.5), sharex=True
    )
    position_axes.plot(time_rel, error_x, linewidth=1.0, label='$e_x$')
    position_axes.plot(time_rel, error_y, linewidth=1.0, label='$e_y$')
    position_axes.plot(
        time_rel, np.hypot(error_x, error_y), linewidth=1.6, label='$|e|$'
    )
    position_axes.set_ylabel('position error [m]')
    position_axes.grid(True)
    position_axes.legend()
    position_axes.set_title(
        'Localization error   RMSE x=%.3f m, y=%.3f m, |e|=%.3f m, yaw=%.4f rad'
        % (
            metrics['rmse_x'],
            metrics['rmse_y'],
            metrics['rmse_pos'],
            metrics['rmse_yaw'],
        )
    )

    yaw_axes.plot(time_rel, np.degrees(error_yaw), linewidth=1.0, label='$e_\\theta$')
    yaw_axes.set_xlabel('time [s]')
    yaw_axes.set_ylabel('yaw error [deg]')
    yaw_axes.grid(True)
    yaw_axes.legend()
    figure.tight_layout()
    error_path = os.path.join(out_dir, 'result_localization_error.png')
    figure.savefig(error_path, dpi=140)
    plt.close(figure)

    return trajectory_path, error_path


def main():
    parser = argparse.ArgumentParser(
        description='Record ground truth and EKF estimate, then report localization errors.'
    )
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION,
                        help='recording duration in seconds (default: %(default)s)')
    parser.add_argument('--out', default=DEFAULT_OUT_DIR,
                        help='output directory (default: %(default)s)')
    parser.add_argument('--replot', action='store_true',
                        help='re-render from the saved npz instead of recording')
    args = parser.parse_args()

    data_path = os.path.join(args.out, DATA_NAME)
    if args.replot:
        if not os.path.exists(data_path):
            raise SystemExit('no data at %s; run once without --replot' % data_path)
        with np.load(data_path) as data:
            ground_truth = data['ground_truth']
            estimate = data['estimate']
    else:
        ground_truth, estimate = record(args.duration)
        os.makedirs(args.out, exist_ok=True)
        np.savez(data_path, ground_truth=ground_truth, estimate=estimate)

    time_rel, error_x, error_y, error_yaw, _matched = align(ground_truth, estimate)
    metrics = compute_metrics(error_x, error_y, error_yaw)
    trajectory_path, error_path = render(
        ground_truth, estimate, time_rel, error_x, error_y, error_yaw,
        metrics, args.out,
    )

    print('samples: ground truth=%d, estimate=%d' % (len(ground_truth), len(estimate)))
    print(
        'RMSE: x=%.4f m, y=%.4f m, |e|=%.4f m, yaw=%.4f rad (%.2f deg)'
        % (
            metrics['rmse_x'],
            metrics['rmse_y'],
            metrics['rmse_pos'],
            metrics['rmse_yaw'],
            math.degrees(metrics['rmse_yaw']),
        )
    )
    print(
        'max:  |e|=%.4f m, |e_yaw|=%.4f rad (%.2f deg)'
        % (metrics['max_pos'], metrics['max_yaw'], math.degrees(metrics['max_yaw']))
    )
    print('figures: %s\n         %s' % (trajectory_path, error_path))


if __name__ == '__main__':
    main()
