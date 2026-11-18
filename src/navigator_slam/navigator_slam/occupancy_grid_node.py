#!/usr/bin/env python3
"""Occupancy-grid mapper (course project stage 6).

Builds a 2D occupancy grid from the noisy lidar scans and a pose estimate:

* free cells along every valid beam and occupied cells at the beam ends are
  accumulated in log-odds (``navigator_slam.occupancy_grid``);
* the pose comes from ``/estimate/odom`` (EKF, stage 5) by default; the same
  node with ``pose_topic:=/ground_truth/odom`` builds the reference map, so the
  two maps can be compared directly (see ``tools/map_compare.py``).

Inputs:
    /scan_noisy       sensor_msgs/LaserScan   (frame lidar_link, inf = dropout)
    pose_topic        nav_msgs/Odometry       (map frame)
Outputs:
    map_topic         nav_msgs/OccupancyGrid  (unknown = -1, free = 0, occupied = 100)

Poses are treated as already expressed in the map frame: the simulation keeps
``map -> odom`` as a fixed identity transform (``simulation.launch.py``), so no
TF lookup is required and the node does not publish any transforms.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from navigator_slam.occupancy_grid import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    LogOddsGrid,
    nearest_sample,
)


def _stamp_seconds(msg):
    stamp = msg.header.stamp
    if stamp.sec == 0 and stamp.nanosec == 0:
        return 0.0
    return stamp.sec + stamp.nanosec * 1e-9


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')

        self.declare_parameter('scan_topic', '/scan_noisy')
        self.declare_parameter('pose_topic', '/estimate/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('grid_min_x', -0.5)
        self.declare_parameter('grid_min_y', -0.5)
        self.declare_parameter('grid_max_x', 8.5)
        self.declare_parameter('grid_max_y', 8.5)

        self.declare_parameter('p_occupied', 0.70)
        self.declare_parameter('p_free', 0.30)
        self.declare_parameter('log_odds_min', -2.0)
        self.declare_parameter('log_odds_max', 3.5)
        self.declare_parameter('occupied_threshold', 0.60)
        self.declare_parameter('free_threshold', 0.40)

        self.declare_parameter('lidar_offset', [0.18, 0.0, 0.34])
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('pose_tolerance', 0.25)
        self.declare_parameter('pose_buffer_size', 100)
        self.declare_parameter('publish_rate_hz', 1.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.pose_tolerance = float(self.get_parameter('pose_tolerance').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.lidar_offset = tuple(
            float(v) for v in self.get_parameter('lidar_offset').value
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        pose_buffer_size = max(1, int(self.get_parameter('pose_buffer_size').value))
        map_topic = str(self.get_parameter('map_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)

        self.grid = LogOddsGrid.from_bounds(
            float(self.get_parameter('grid_min_x').value),
            float(self.get_parameter('grid_min_y').value),
            float(self.get_parameter('grid_max_x').value),
            float(self.get_parameter('grid_max_y').value),
            float(self.get_parameter('resolution').value),
            p_occupied=float(self.get_parameter('p_occupied').value),
            p_free=float(self.get_parameter('p_free').value),
            log_odds_min=float(self.get_parameter('log_odds_min').value),
            log_odds_max=float(self.get_parameter('log_odds_max').value),
            occupied_threshold=float(
                self.get_parameter('occupied_threshold').value
            ),
            free_threshold=float(self.get_parameter('free_threshold').value),
        )

        self.poses = deque(maxlen=pose_buffer_size)
        self.scans = 0

        self.create_subscription(Odometry, pose_topic, self.pose_callback, 50)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 20)
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, 1)
        self.create_timer(1.0 / max(publish_rate, 1e-3), self.publish_map)

        self.get_logger().info(
            'occupancy_grid_node: %dx%d cells (%.2f m), scan %s, pose %s, map %s'
            % (
                self.grid.width,
                self.grid.height,
                self.grid.resolution,
                self.scan_topic,
                pose_topic,
                map_topic,
            )
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.poses.append((
            _stamp_seconds(msg),
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
        ))

    def scan_callback(self, msg):
        if not self.poses:
            self.get_logger().warn(
                'scan dropped: no pose on %s yet'
                % self.get_parameter('pose_topic').value,
                throttle_duration_sec=5.0,
            )
            return

        stamp = _stamp_seconds(msg)
        index = nearest_sample([p[0] for p in self.poses], stamp)
        pose_time, x, y, yaw = self.poses[index]
        if abs(pose_time - stamp) > self.pose_tolerance:
            self.get_logger().warn(
                'scan dropped: nearest pose is %.3f s away'
                % abs(pose_time - stamp),
                throttle_duration_sec=5.0,
            )
            return

        free, occupied = self.grid.update_scan(
            (x, y, yaw),
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
            lidar_offset=self.lidar_offset,
            max_range=self.max_range,
        )
        self.scans += 1
        self.get_logger().debug(
            'scan %d: %d free, %d occupied cells'
            % (self.scans, free, occupied)
        )

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def publish_map(self):
        now = self.get_clock().now()
        occupancy = self.grid.occupancy_data()

        msg = OccupancyGrid()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.map_load_time = now.to_msg()
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.origin[0]
        msg.info.origin.position.y = self.grid.origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy.reshape(-1).tolist()
        self.map_pub.publish(msg)

        self.get_logger().info(
            'map: %d scans, occupied %d, free %d, unknown %d cells'
            % (
                self.scans,
                int(np.count_nonzero(occupancy == OCCUPIED)),
                int(np.count_nonzero(occupancy == FREE)),
                int(np.count_nonzero(occupancy == UNKNOWN)),
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
