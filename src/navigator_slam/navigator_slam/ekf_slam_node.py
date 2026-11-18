#!/usr/bin/env python3
"""Joint EKF-SLAM node: robot pose and landmark map (course project stage 8).

The filter itself is the pure module ``navigator_slam.ekf_slam``; this wrapper
only converts messages:

Inputs:
    /cmd_vel                    geometry_msgs/Twist        (prediction control)
    /landmarks/observations     navigator_interfaces/LandmarkObservationArray
                                                           (frame base_footprint)
    /uwb/noisy_ranges           std_msgs/Float32MultiArray (NaN = dropout)
    /ground_truth/odom          nav_msgs/Odometry          (only for /slam/error)
Outputs:
    /slam/odom                  nav_msgs/Odometry          (map -> base_footprint)
    /slam/path                  nav_msgs/Path
    /slam/landmarks             navigator_interfaces/LandmarkMap
    /slam/markers               visualization_msgs/MarkerArray
    /slam/error                 geometry_msgs/Vector3Stamped

Like the stage-5 EKF the node does NOT broadcast TF: ``odom ->
base_footprint`` is owned by the ground-truth ``motion_node``, and
``map -> odom`` is the fixed identity transform of the simulation. The pose is
therefore published on topics only.

The landmark observations are pose-free (``base_footprint``) and stamped with
the scan time; they are applied when they arrive, which at simulation speeds
(<= 0.4 m/s, 10 Hz) is within a couple of centimetres of the stamped pose.
"""

import math
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from navigator_interfaces.msg import Landmark, LandmarkMap, LandmarkObservationArray
from navigator_slam.ekf_slam import EKFSLAM, Observation, wrap_angle
from navigator_slam.pose_utils import quaternion_to_yaw

#: Warn when a landmark observation reaches the filter later than this [s].
OBSERVATION_AGE_WARN = 0.5

MARKER_COLORS = [
    (0.90, 0.15, 0.15),
    (0.15, 0.80, 0.20),
    (0.15, 0.30, 0.90),
    (0.95, 0.70, 0.05),
    (0.70, 0.15, 0.85),
    (0.10, 0.80, 0.80),
]


def _stamp_seconds(msg):
    stamp = msg.header.stamp
    if stamp.sec == 0 and stamp.nanosec == 0:
        return 0.0
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class EkfSlamNode(Node):
    def __init__(self):
        super().__init__('ekf_slam_node')

        # Topics and frames.
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('observation_topic', '/landmarks/observations')
        self.declare_parameter('uwb_topic', '/uwb/noisy_ranges')
        self.declare_parameter('ground_truth_topic', '/ground_truth/odom')
        self.declare_parameter('odom_topic', '/slam/odom')
        self.declare_parameter('path_topic', '/slam/path')
        self.declare_parameter('landmarks_topic', '/slam/landmarks')
        self.declare_parameter('markers_topic', '/slam/markers')
        self.declare_parameter('error_topic', '/slam/error')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_footprint')

        # Filter setup.
        self.declare_parameter('initial_x', 1.5)
        self.declare_parameter('initial_y', 1.5)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('initial_std_x', 0.10)
        self.declare_parameter('initial_std_y', 0.10)
        self.declare_parameter('initial_std_yaw', 0.0873)
        self.declare_parameter('q_x', 1.0e-4)
        self.declare_parameter('q_y', 1.0e-4)
        self.declare_parameter('q_yaw', 1.0e-4)
        self.declare_parameter('predict_rate_hz', 20.0)

        # UWB (stage 5 range model, same anchors and tag offset as the sim).
        self.declare_parameter('use_uwb', True)
        self.declare_parameter('uwb_sigma', 0.05)
        self.declare_parameter('tag_offset', [-0.15, 0.12, 0.46])
        self.declare_parameter(
            'anchors',
            [0.50, 0.50, 1.20,
             7.40, 0.80, 1.20,
             7.10, 7.50, 1.20,
             0.70, 6.90, 1.20],
        )

        # Landmark model and association.
        self.declare_parameter('gate', 9.21)
        self.declare_parameter('sigma_range', 0.05)
        self.declare_parameter('sigma_bearing', 0.05)
        self.declare_parameter('initial_landmark_std', 0.10)
        self.declare_parameter('max_landmarks', 50)
        # A candidate landmark enters the state only after this many
        # consistent sightings within tentative_gate metres (wall corners
        # otherwise become phantom landmarks).
        self.declare_parameter('min_landmark_hits', 8)
        self.declare_parameter('tentative_gate', 0.35)
        self.declare_parameter('max_tentative_tracks', 200)

        # Publishing.
        self.declare_parameter('path_max_poses', 5000)
        self.declare_parameter('path_every_n', 4)
        self.declare_parameter('publish_error', True)
        self.declare_parameter('publish_rate_hz', 5.0)

        x0 = float(self.get_parameter('initial_x').value)
        y0 = float(self.get_parameter('initial_y').value)
        yaw0 = float(self.get_parameter('initial_yaw').value)
        std_x = float(self.get_parameter('initial_std_x').value)
        std_y = float(self.get_parameter('initial_std_y').value)
        std_yaw = float(self.get_parameter('initial_std_yaw').value)
        q_x = float(self.get_parameter('q_x').value)
        q_y = float(self.get_parameter('q_y').value)
        q_yaw = float(self.get_parameter('q_yaw').value)

        self.use_uwb = bool(self.get_parameter('use_uwb').value)
        self.uwb_sigma = float(self.get_parameter('uwb_sigma').value)
        tag_offset = tuple(
            float(v) for v in self.get_parameter('tag_offset').value
        )
        flat = [float(v) for v in self.get_parameter('anchors').value]
        if len(flat) % 3 != 0 or not flat:
            raise ValueError('anchors must be a non-empty flat list of x,y,z triples')
        self.anchors = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        self.gate = float(self.get_parameter('gate').value)

        self.slam = EKFSLAM(
            [x0, y0, yaw0],
            [
                [std_x * std_x, 0.0, 0.0],
                [0.0, std_y * std_y, 0.0],
                [0.0, 0.0, std_yaw * std_yaw],
            ],
            tag_offset=tag_offset,
            sigma_range=float(self.get_parameter('sigma_range').value),
            sigma_bearing=float(self.get_parameter('sigma_bearing').value),
            initial_landmark_std=float(
                self.get_parameter('initial_landmark_std').value
            ),
            max_landmarks=int(self.get_parameter('max_landmarks').value),
            gate=self.gate,
            min_landmark_hits=int(
                self.get_parameter('min_landmark_hits').value
            ),
            tentative_gate=float(self.get_parameter('tentative_gate').value),
            max_tentative_tracks=int(
                self.get_parameter('max_tentative_tracks').value
            ),
        )
        self.Q = [
            [q_x, 0.0, 0.0],
            [0.0, q_y, 0.0],
            [0.0, 0.0, q_yaw],
        ]
        self.R_uwb = self.uwb_sigma ** 2

        self.rate = float(self.get_parameter('predict_rate_hz').value)
        self.path_max_poses = int(self.get_parameter('path_max_poses').value)
        self.path_every_n = max(1, int(self.get_parameter('path_every_n').value))
        self.publish_error = bool(
            self.get_parameter('publish_error').value
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.v = 0.0
        self.omega = 0.0
        self.last_predict_time = None
        self.path_counter = 0
        self.path = Path()
        self.path.header.frame_id = self.frame_id

        self.gt = None
        self.sum_sq = [0.0, 0.0, 0.0]
        self.error_count = 0

        self.observation_count = 0
        self.matched_count = 0
        self.added_count = 0
        self.dropped_count = 0
        self.cap_warned = False

        self.create_subscription(
            Twist, str(self.get_parameter('cmd_topic').value),
            self.cmd_callback, 10,
        )
        self.create_subscription(
            LandmarkObservationArray,
            str(self.get_parameter('observation_topic').value),
            self.observation_callback, 10,
        )
        if self.use_uwb:
            self.create_subscription(
                Float32MultiArray,
                str(self.get_parameter('uwb_topic').value),
                self.uwb_callback, 10,
            )

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 10
        )
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter('path_topic').value), 10
        )
        self.landmarks_pub = self.create_publisher(
            LandmarkMap, str(self.get_parameter('landmarks_topic').value), 10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('markers_topic').value), 10
        )

        if self.publish_error:
            self.error_pub = self.create_publisher(
                Vector3Stamped, str(self.get_parameter('error_topic').value), 10
            )
            self.create_subscription(
                Odometry,
                str(self.get_parameter('ground_truth_topic').value),
                self.gt_callback, 10,
            )

        self.create_timer(1.0 / max(self.rate, 1e-3), self.predict_callback)
        self.create_timer(1.0 / max(publish_rate, 1e-3), self.publish_map)
        self.create_timer(1.0, self.report_metrics)

        self.get_logger().info(
            'ekf_slam_node: predict %.1f Hz, %d UWB anchors (%s), gate %.2f, '
            'max %d landmarks'
            % (
                self.rate,
                len(self.anchors),
                'enabled' if self.use_uwb else 'disabled',
                self.gate,
                self.slam.max_landmarks,
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

    def uwb_callback(self, msg):
        ranges = [float(r) for r in msg.data]
        if len(ranges) != len(self.anchors):
            self.get_logger().warn(
                'uwb range count %d != anchors %d; message ignored'
                % (len(ranges), len(self.anchors)),
                throttle_duration_sec=5.0,
            )
            return
        self.slam.update_uwb(ranges, self.anchors, self.R_uwb)

    def observation_callback(self, msg):
        stamp = _stamp_seconds(msg)
        if stamp > 0.0:
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if age > OBSERVATION_AGE_WARN:
                self.get_logger().warn(
                    'landmark observation is %.2f s old; applied at the '
                    'current estimate' % age,
                    throttle_duration_sec=5.0,
                )

        observations = []
        for item in msg.observations:
            observation = Observation(
                range=float(item.range),
                bearing=float(item.bearing),
                radius=float(item.radius),
                range_variance=float(item.range_variance),
                bearing_variance=float(item.bearing_variance),
            )
            if observation.is_finite():
                observations.append(observation)
        if not observations:
            return

        self.observation_count += len(observations)
        associations, added = self.slam.process_observations(observations)
        self.matched_count += sum(
            1 for association in associations if association is not None
        )
        self.added_count += len(added)
        for new_id in added:
            self.get_logger().debug(
                'new landmark %d confirmed (map: %d landmarks, %d candidates)'
                % (new_id, self.slam.landmark_count, len(self.slam.tentative))
            )
        if self.slam.landmark_count >= self.slam.max_landmarks:
            # The state is full: candidates are dropped until a slot frees up.
            self.dropped_count += len(observations) - sum(
                1 for association in associations if association is not None
            ) - len(added)
            if not self.cap_warned:
                self.cap_warned = True
                self.get_logger().warn(
                    'landmark cap reached (%d): new landmarks are ignored'
                    % self.slam.max_landmarks
                )

    # ------------------------------------------------------------------
    # Prediction and publishing
    # ------------------------------------------------------------------
    def predict_callback(self):
        now = self.get_clock().now()
        if self.last_predict_time is None:
            dt = 1.0 / max(self.rate, 1e-3)
        else:
            dt = (now - self.last_predict_time).nanoseconds * 1e-9
        self.last_predict_time = now
        dt = min(max(dt, 1e-4), 0.2)

        self.slam.predict(self.v, self.omega, dt, self.Q)
        self.publish_pose(now)

    def publish_pose(self, now):
        stamp = now.to_msg()
        x, y, yaw = (float(v) for v in self.slam.mu[:3])
        quat = yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega

        P = self.slam.Sigma
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
            ps.header.frame_id = self.frame_id
            ps.pose = odom.pose.pose
            self.path.poses.append(ps)
            if 0 < self.path_max_poses < len(self.path.poses):
                self.path.poses = self.path.poses[-self.path_max_poses:]
            self.path.header.stamp = stamp
            self.path_pub.publish(self.path)

        if self.publish_error and self.gt is not None:
            ex = x - self.gt[0]
            ey = y - self.gt[1]
            eyaw = wrap_angle(yaw - self.gt[2])

            error = Vector3Stamped()
            error.header.stamp = stamp
            error.header.frame_id = self.frame_id
            error.vector.x = ex
            error.vector.y = ey
            error.vector.z = eyaw
            self.error_pub.publish(error)

            self.sum_sq[0] += ex * ex
            self.sum_sq[1] += ey * ey
            self.sum_sq[2] += eyaw * eyaw
            self.error_count += 1

    def publish_map(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        landmarks = self.slam.landmarks()

        message = LandmarkMap()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        array = MarkerArray()
        for index, landmark in enumerate(landmarks):
            entry = Landmark()
            entry.id = int(landmark.id)
            entry.x = float(landmark.x)
            entry.y = float(landmark.y)
            entry.covariance = [
                float(landmark.covariance[0, 0]),
                float(landmark.covariance[0, 1]),
                float(landmark.covariance[1, 0]),
                float(landmark.covariance[1, 1]),
            ]
            message.landmarks.append(entry)

            r, g, b = MARKER_COLORS[landmark.id % len(MARKER_COLORS)]
            sphere = self._marker(index, Marker.SPHERE, stamp)
            sphere.pose.position.x = entry.x
            sphere.pose.position.y = entry.y
            sphere.pose.position.z = 0.12
            sphere.scale.x = 0.18
            sphere.scale.y = 0.18
            sphere.scale.z = 0.24
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = 1.0
            array.markers.append(sphere)

            label = self._marker(1000 + index, Marker.TEXT_VIEW_FACING, stamp)
            label.pose.position.x = entry.x
            label.pose.position.y = entry.y
            label.pose.position.z = 0.50
            label.scale.z = 0.22
            label.color.r = 0.1
            label.color.g = 0.1
            label.color.b = 0.1
            label.color.a = 1.0
            label.text = '%d' % entry.id
            array.markers.append(label)

        self.landmarks_pub.publish(message)
        self.markers_pub.publish(array)

    def _marker(self, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'slam_landmarks'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    # ------------------------------------------------------------------
    # Metrics
    # ------------------------------------------------------------------
    def report_metrics(self):
        n = self.error_count
        if self.publish_error and n:
            rmse = [math.sqrt(s / n) for s in self.sum_sq]
            self.get_logger().info(
                'slam RMSE x=%.3f m y=%.3f m yaw=%.4f rad (samples=%d), '
                '%d landmarks (%d matched, %d new, %d dropped of %d '
                'observations)'
                % (
                    rmse[0], rmse[1], rmse[2], n,
                    self.slam.landmark_count,
                    self.matched_count,
                    self.added_count,
                    self.dropped_count,
                    self.observation_count,
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = EkfSlamNode()
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
