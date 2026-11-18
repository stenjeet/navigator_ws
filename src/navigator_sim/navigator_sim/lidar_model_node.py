#!/usr/bin/env python3
"""Lidar scan model (report section 4.3).

Performs 2D ray casting from the lidar pose against the polygon geometry
(walls, box obstacles, cylinders, triangle, landmarks) and publishes ideal and
noisy sensor_msgs/LaserScan messages with frame_id = lidar_link.

Range model (report eq. 17-19):
    ideal  r_j = min distance to the nearest obstacle along the ray
    noisy  z_j = clip(r_j + b_L + nu_j, r_min, r_max), nu ~ N(0, sigma^2)
    dropout -> +inf (no reflection)

The UWB anchors are radio infrastructure and are intentionally NOT part of the
lidar occlusion geometry.
"""

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from navigator_sim import world

INF = float('inf')
EPS = 1e-12


def ray_aabb(ox, oy, dx, dy, xmin, xmax, ymin, ymax):
    """Ray vs axis-aligned box (slab method). Returns entry distance or None."""
    tmin = -INF
    tmax = INF

    if abs(dx) < EPS:
        if ox < xmin or ox > xmax:
            return None
    else:
        t1 = (xmin - ox) / dx
        t2 = (xmax - ox) / dx
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)

    if abs(dy) < EPS:
        if oy < ymin or oy > ymax:
            return None
    else:
        t1 = (ymin - oy) / dy
        t2 = (ymax - oy) / dy
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)

    if tmin <= tmax and tmax >= 0.0:
        return tmin if tmin >= 0.0 else 0.0
    return None


def ray_circle(ox, oy, dx, dy, cx, cy, r):
    """Ray vs circle. Returns entry distance or None (unit direction assumed)."""
    ex = ox - cx
    ey = oy - cy
    b = 2.0 * (dx * ex + dy * ey)
    c = ex * ex + ey * ey - r * r
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    t1 = (-b - sq) / 2.0
    t2 = (-b + sq) / 2.0
    if t1 < 0.0 and t2 < 0.0:
        return None
    return t1 if t1 >= 0.0 else t2


def ray_segment(ox, oy, dx, dy, x1, y1, x2, y2):
    """Ray vs 2D line segment. Returns distance or None."""
    rx = x2 - x1
    ry = y2 - y1
    denom = dx * ry - dy * rx
    if abs(denom) < EPS:
        return None
    ex = x1 - ox
    ey = y1 - oy
    t = (ex * ry - ey * rx) / denom
    if t < 0.0:
        return None
    s = (ex * dy - ey * dx) / denom
    if s < 0.0 or s > 1.0:
        return None
    return t


class WorldGeometry:
    """Precomputed obstacle geometry for fast ray casting."""

    def __init__(self):
        self.aabbs = [w for w in world.WALLS] + \
                     [(xmin, xmax, ymin, ymax) for (xmin, xmax, ymin, ymax, _) in world.BOX_OBSTACLES]
        self.circles = [(x, y, r) for (x, y, r, _) in world.CYLINDER_OBSTACLES]
        self.circles += [(x, y, r) for (_, x, y, r, _, _) in world.LANDMARKS]
        self.triangle = world.TRIANGLE_OBSTACLE['vertices']

    def raycast(self, ox, oy, phi):
        dx = math.cos(phi)
        dy = math.sin(phi)
        best = None

        for (xmin, xmax, ymin, ymax) in self.aabbs:
            t = ray_aabb(ox, oy, dx, dy, xmin, xmax, ymin, ymax)
            if t is not None and t >= 0.0 and (best is None or t < best):
                best = t

        for (cx, cy, r) in self.circles:
            t = ray_circle(ox, oy, dx, dy, cx, cy, r)
            if t is not None and t >= 0.0 and (best is None or t < best):
                best = t

        tri = self.triangle
        for i in range(3):
            x1, y1 = tri[i]
            x2, y2 = tri[(i + 1) % 3]
            t = ray_segment(ox, oy, dx, dy, x1, y1, x2, y2)
            if t is not None and (best is None or t < best):
                best = t

        return best


class LidarModelNode(Node):
    def __init__(self):
        super().__init__('lidar_model_node')

        self.declare_parameter('random_seed', 12025)
        self.declare_parameter('lidar_sigma', 0.02)
        self.declare_parameter('lidar_bias', 0.01)
        self.declare_parameter('lidar_dropout_probability', 0.01)
        self.declare_parameter('lidar_range_min', 0.12)
        self.declare_parameter('lidar_range_max', 8.0)
        self.declare_parameter('lidar_beams', 360)
        self.declare_parameter('lidar_rate_hz', 10.0)

        seed = int(self.get_parameter('random_seed').value)
        self.sigma = float(self.get_parameter('lidar_sigma').value)
        self.bias = float(self.get_parameter('lidar_bias').value)
        self.dropout_prob = float(self.get_parameter('lidar_dropout_probability').value)
        self.range_min = float(self.get_parameter('lidar_range_min').value)
        self.range_max = float(self.get_parameter('lidar_range_max').value)
        self.beams = int(self.get_parameter('lidar_beams').value)
        rate_hz = float(self.get_parameter('lidar_rate_hz').value)

        self.rng = random.Random(seed)
        self.geometry = WorldGeometry()
        self.lidar_offset = world.LIDAR_OFFSET

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_pose = False

        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10
        )
        self.ideal_pub = self.create_publisher(LaserScan, '/scan_ideal', 10)
        self.noisy_pub = self.create_publisher(LaserScan, '/scan_noisy', 10)

        self.angle_increment = 2.0 * math.pi / self.beams
        self.angle_min = 0.0
        self.angle_max = 2.0 * math.pi - self.angle_increment

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            'lidar_model_node: %d beams, rate %.1f Hz, seed %d'
            % (self.beams, rate_hz, seed)
        )

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.have_pose = True

    def lidar_position(self):
        dx, dy, _ = self.lidar_offset
        cos_t = math.cos(self.yaw)
        sin_t = math.sin(self.yaw)
        x_l = self.x + dx * cos_t - dy * sin_t
        y_l = self.y + dx * sin_t + dy * cos_t
        return x_l, y_l

    def timer_callback(self):
        if not self.have_pose:
            return

        x_l, y_l = self.lidar_position()
        ideal = self.make_scan()
        noisy = self.make_scan()

        for j in range(self.beams):
            phi = self.yaw + self.angle_min + j * self.angle_increment
            t = self.geometry.raycast(x_l, y_l, phi)

            if t is None or t < self.range_min or t > self.range_max:
                ideal.ranges[j] = INF
                noisy.ranges[j] = INF
                continue

            ideal.ranges[j] = float(t)

            if self.rng.random() < self.dropout_prob:
                noisy.ranges[j] = INF
            else:
                noise = self.rng.gauss(0.0, self.sigma)
                z = t + self.bias + noise
                z = min(max(z, self.range_min), self.range_max)
                noisy.ranges[j] = float(z)

        now = self.get_clock().now().to_msg()
        ideal.header.stamp = now
        noisy.header.stamp = now
        self.ideal_pub.publish(ideal)
        self.noisy_pub.publish(noisy)

    def make_scan(self):
        scan = LaserScan()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [0.0] * self.beams
        scan.intensities = []
        return scan


def main(args=None):
    rclpy.init(args=args)
    node = LidarModelNode()
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
