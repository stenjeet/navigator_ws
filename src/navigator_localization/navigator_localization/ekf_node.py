#!/usr/bin/env python3
"""ROS 2 wrapper around the pure EKF (course project stage 5).

Prediction uses the commanded velocities from ``/cmd_vel``; correction uses the
noisy UWB ranges from ``/uwb/noisy_ranges`` (NaN marks a dropout). The estimated
pose is published on ``/estimate/odom`` and ``/estimate/path``.

The node deliberately does NOT broadcast the ``odom -> base_footprint``
transform: that frame is produced by ``motion_node`` (ground truth) and a second
publisher would conflict. Visualization uses the estimate topics instead.

Inputs:
    /cmd_vel              geometry_msgs/Twist
    /uwb/noisy_ranges     std_msgs/Float32MultiArray
    /ground_truth/odom    nav_msgs/Odometry   (only for error reporting)
Outputs:
    /estimate/odom        nav_msgs/Odometry
    /estimate/path        nav_msgs/Path
    /estimate/error       geometry_msgs/Vector3Stamped   (if publish_error)
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray

from navigator_localization.ekf import EKFLocalization, wrap_angle


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class EkfNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        self.declare_parameter('initial_x', 1.5)
        self.declare_parameter('initial_y', 1.5)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('initial_std_x', 0.10)
        self.declare_parameter('initial_std_y', 0.10)
        self.declare_parameter('initial_std_yaw', 0.0873)
        self.declare_parameter('q_x', 1.0e-4)
        self.declare_parameter('q_y', 1.0e-4)
        self.declare_parameter('q_yaw', 1.0e-4)
        self.declare_parameter('uwb_sigma', 0.05)
        self.declare_parameter('tag_offset', [-0.15, 0.12, 0.46])
        self.declare_parameter('use_tag_offset', True)
        self.declare_parameter('anchor_names', ['A1', 'A2', 'A3', 'A4'])
        self.declare_parameter(
            'anchors',
            [0.50, 0.50, 1.20,
             7.40, 0.80, 1.20,
             7.10, 7.50, 1.20,
             0.70, 6.90, 1.20],
        )
        self.declare_parameter('predict_rate_hz', 20.0)
        self.declare_parameter('path_max_poses', 5000)
        self.declare_parameter('path_every_n', 4)
        self.declare_parameter('publish_error', True)

        x0 = float(self.get_parameter('initial_x').value)
        y0 = float(self.get_parameter('initial_y').value)
        yaw0 = float(self.get_parameter('initial_yaw').value)
        std_x = float(self.get_parameter('initial_std_x').value)
        std_y = float(self.get_parameter('initial_std_y').value)
        std_yaw = float(self.get_parameter('initial_std_yaw').value)
        q_x = float(self.get_parameter('q_x').value)
        q_y = float(self.get_parameter('q_y').value)
        q_yaw = float(self.get_parameter('q_yaw').value)
        self.uwb_sigma = float(self.get_parameter('uwb_sigma').value)
        tag_offset = tuple(
            float(v) for v in self.get_parameter('tag_offset').value
        )
        use_tag_offset = bool(self.get_parameter('use_tag_offset').value)

        self.anchor_names = [
            str(n) for n in self.get_parameter('anchor_names').value
        ]
        flat = [float(v) for v in self.get_parameter('anchors').value]
        if len(flat) % 3 != 0 or not flat:
            raise ValueError('anchors must be a non-empty flat list of x,y,z triples')
        self.anchors = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        if len(self.anchor_names) != len(self.anchors):
            self.get_logger().warn(
                'anchor_names length != anchors count; using generated names'
            )
            self.anchor_names = ['A%d' % (i + 1) for i in range(len(self.anchors))]

        self.rate = float(self.get_parameter('predict_rate_hz').value)
        self.path_max_poses = int(self.get_parameter('path_max_poses').value)
        self.path_every_n = max(1, int(self.get_parameter('path_every_n').value))
        self.publish_error = bool(self.get_parameter('publish_error').value)

        self.ekf = EKFLocalization(
            [x0, y0, yaw0],
            [
                [std_x * std_x, 0.0, 0.0],
                [0.0, std_y * std_y, 0.0],
                [0.0, 0.0, std_yaw * std_yaw],
            ],
            tag_offset=tag_offset,
            use_tag_offset=use_tag_offset,
        )
        self.Q = [
            [q_x, 0.0, 0.0],
            [0.0, q_y, 0.0],
            [0.0, 0.0, q_yaw],
        ]
        self.R = self.uwb_sigma ** 2

        self.v = 0.0
        self.omega = 0.0
        self.last_predict_time = None
        self.path_counter = 0

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.gt = None
        self.sum_sq = [0.0, 0.0, 0.0]
        self.error_count = 0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/uwb/noisy_ranges', self.uwb_callback, 10
        )

        self.odom_pub = self.create_publisher(Odometry, '/estimate/odom', 10)
        self.path_pub = self.create_publisher(Path, '/estimate/path', 10)

        if self.publish_error:
            self.error_pub = self.create_publisher(
                Vector3Stamped, '/estimate/error', 10
            )
            self.create_subscription(
                Odometry, '/ground_truth/odom', self.gt_callback, 10
            )

        self.create_timer(1.0 / self.rate, self.predict_callback)
        self.create_timer(1.0, self.report_metrics)
        self.get_logger().info(
            'ekf_node: %d anchors (%s), predict %.1f Hz, uwb sigma %.2f m'
            % (
                len(self.anchors),
                ', '.join(self.anchor_names),
                self.rate,
                self.uwb_sigma,
            )
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def cmd_callback(self, msg):
        self.v = float(msg.linear.x)
        self.omega = float(msg.angular.z)

    def gt_callback(self, msg):
        pose = msg.pose.pose
        self.gt = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
        )

    def predict_callback(self):
        now = self.get_clock().now()
        if self.last_predict_time is None:
            dt = 1.0 / self.rate
        else:
            dt = (now - self.last_predict_time).nanoseconds * 1e-9
        self.last_predict_time = now
        dt = min(max(dt, 1e-4), 0.2)

        self.ekf.predict(self.v, self.omega, dt, self.Q)
        self.publish_estimate(now)

    def uwb_callback(self, msg):
        ranges = [float(r) for r in msg.data]
        if len(ranges) != len(self.anchors):
            self.get_logger().warn(
                'uwb range count %d != anchors %d; message ignored'
                % (len(ranges), len(self.anchors))
            )
            return
        self.ekf.update(ranges, self.anchors, self.R)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def publish_estimate(self, now):
        stamp = now.to_msg()
        x, y, yaw = (float(v) for v in self.ekf.x)
        quat = yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega

        P = self.ekf.P
        covariance = [0.0] * 36
        covariance[0] = float(P[0, 0])
        covariance[1] = float(P[0, 1])
        covariance[5] = float(P[0, 2])
        covariance[6] = float(P[1, 0])
        covariance[7] = float(P[1, 1])
        covariance[11] = float(P[1, 2])
        covariance[30] = float(P[2, 0])
        covariance[31] = float(P[2, 1])
        covariance[35] = float(P[2, 2])
        odom.pose.covariance = covariance
        self.odom_pub.publish(odom)

        self.path_counter += 1
        if self.path_counter % self.path_every_n == 0:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'map'
            ps.pose = odom.pose.pose
            self.path.header.stamp = stamp
            self.path.poses.append(ps)
            if 0 < self.path_max_poses < len(self.path.poses):
                self.path.poses = self.path.poses[-self.path_max_poses:]
            self.path_pub.publish(self.path)

        if self.publish_error and self.gt is not None:
            ex = x - self.gt[0]
            ey = y - self.gt[1]
            eyaw = wrap_angle(yaw - self.gt[2])

            error = Vector3Stamped()
            error.header.stamp = stamp
            error.header.frame_id = 'map'
            error.vector.x = ex
            error.vector.y = ey
            error.vector.z = eyaw
            self.error_pub.publish(error)

            self.sum_sq[0] += ex * ex
            self.sum_sq[1] += ey * ey
            self.sum_sq[2] += eyaw * eyaw
            self.error_count += 1

    def report_metrics(self):
        if not self.publish_error or self.error_count == 0:
            return
        n = self.error_count
        rmse = [math.sqrt(s / n) for s in self.sum_sq]
        self.get_logger().info(
            'RMSE x=%.3f m y=%.3f m yaw=%.4f rad (samples=%d)'
            % (rmse[0], rmse[1], rmse[2], n)
        )


def main(args=None):
    rclpy.init(args=args)
    node = EkfNode()
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
