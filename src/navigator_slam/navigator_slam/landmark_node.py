#!/usr/bin/env python3
"""Landmark detector and landmark map (course project stage 7).

Wraps the pure detector/associator in ``navigator_slam.landmark_detection``:

* every scan is turned into pose-free observations in ``base_footprint`` and
  published on ``/landmarks/observations``;
* when a pose is available (``pose_topic``, ``/estimate/odom`` by default) the
  observations are associated with the tracked map, new ids are created on
  first sight and the position of known ids is updated with an exponential
  moving average; the map is published periodically on ``/landmarks/map`` and
  drawn on ``/landmarks/markers``;
* an empty ``pose_topic`` disables map tracking, association and markers, so
  the node can feed the joint EKF-SLAM (stage 8) with raw observations only.

Inputs:
    /scan_noisy                  sensor_msgs/LaserScan  (frame lidar_link)
    pose_topic                   nav_msgs/Odometry     (optional, map frame)
Outputs:
    /landmarks/observations      navigator_interfaces/LandmarkObservationArray
    /landmarks/map               navigator_interfaces/LandmarkMap
    /landmarks/markers           visualization_msgs/MarkerArray

Poses are already expressed in the ``map`` frame (``map -> odom`` is a fixed
identity transform in the simulation), so no TF lookup is needed and the node
does not publish any transforms.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from navigator_interfaces.msg import (
    Landmark as LandmarkMsg,
    LandmarkMap,
    LandmarkObservation as LandmarkObservationMsg,
    LandmarkObservationArray,
)

from navigator_slam.landmark_detection import (
    DetectionParams,
    LandmarkEstimate,
    associate_observations,
    extract_landmarks,
)

#: Indices of the 3x3 pose block inside a 6x6 row-major pose covariance.
POSE_COVARIANCE_INDICES = (0, 1, 2, 6, 7, 8, 30, 31, 35)

#: Height of the landmark cylinders in the visualization [m].
MARKER_HEIGHT = 0.45


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_covariance(covariance):
    """3x3 ``[x, y, yaw]`` block of a row-major 6x6 pose covariance."""
    values = [float(covariance[index]) for index in POSE_COVARIANCE_INDICES]
    return np.asarray(values, dtype=float).reshape(3, 3)


def rotate(angle, x, y):
    """Rotate a planar vector by ``angle``."""
    cos_angle, sin_angle = math.cos(angle), math.sin(angle)
    return cos_angle * x - sin_angle * y, sin_angle * x + cos_angle * y


class LandmarkNode(Node):
    def __init__(self):
        super().__init__('landmark_node')

        self.declare_parameter('scan_topic', '/scan_noisy')
        self.declare_parameter('pose_topic', '/estimate/odom')
        self.declare_parameter('observation_topic', '/landmarks/observations')
        self.declare_parameter('map_topic', '/landmarks/map')
        self.declare_parameter('marker_topic', '/landmarks/markers')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('map_frame_id', 'map')

        self.declare_parameter('lidar_offset', [0.18, 0.0, 0.34])

        self.declare_parameter('jump_threshold', 0.15)
        self.declare_parameter('min_points', 3)
        self.declare_parameter('max_points', 25)
        self.declare_parameter('max_cluster_width', 0.20)
        self.declare_parameter('radius_min', 0.04)
        self.declare_parameter('radius_max', 0.25)
        self.declare_parameter('max_residual', 0.05)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('sigma_range', 0.02)
        self.declare_parameter('sigma_bearing', 0.02)

        self.declare_parameter('gate', 9.21)
        self.declare_parameter('map_alpha', 0.30)
        self.declare_parameter('max_landmarks', 100)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.observation_topic = str(
            self.get_parameter('observation_topic').value
        )
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.map_frame_id = str(self.get_parameter('map_frame_id').value)
        self.lidar_offset = tuple(
            float(v) for v in self.get_parameter('lidar_offset').value
        )

        self.params = DetectionParams(
            jump_threshold=float(self.get_parameter('jump_threshold').value),
            min_points=int(self.get_parameter('min_points').value),
            max_points=int(self.get_parameter('max_points').value),
            max_cluster_width=float(
                self.get_parameter('max_cluster_width').value
            ),
            radius_min=float(self.get_parameter('radius_min').value),
            radius_max=float(self.get_parameter('radius_max').value),
            max_residual=float(self.get_parameter('max_residual').value),
            max_range=float(self.get_parameter('max_range').value),
            min_range=float(self.get_parameter('min_range').value),
            sigma_range=float(self.get_parameter('sigma_range').value),
            sigma_bearing=float(self.get_parameter('sigma_bearing').value),
        )
        self.gate = float(self.get_parameter('gate').value)
        self.map_alpha = float(self.get_parameter('map_alpha').value)
        self.max_landmarks = max(
            1, int(self.get_parameter('max_landmarks').value)
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        # Map: id -> LandmarkEstimate. Ids start at 1 and are never reused.
        self.landmarks = {}
        self.next_id = 1
        self.scans = 0

        # Latest pose estimate (None until the first message arrives).
        self.pose = None
        self.pose_covariance = np.eye(3, dtype=float)

        self.observation_pub = self.create_publisher(
            LandmarkObservationArray, self.observation_topic, 10
        )
        self.map_pub = self.create_publisher(LandmarkMap, self.map_topic, 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.marker_topic, 1
        )

        if self.pose_topic:
            self.create_subscription(
                Odometry, self.pose_topic, self.pose_callback, 50
            )
            self.create_timer(
                1.0 / max(publish_rate, 1e-3), self.publish_map
            )
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 20
        )

        self.get_logger().info(
            'landmark_node: scan %s, pose %s, observations %s, map %s'
            % (
                self.scan_topic,
                self.pose_topic if self.pose_topic else '(disabled)',
                self.observation_topic,
                self.map_topic,
            )
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.pose = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
        )
        self.pose_covariance = pose_covariance(msg.pose.covariance)

    def scan_callback(self, msg):
        observations = extract_landmarks(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
            lidar_offset=self.lidar_offset,
            params=self.params,
        )

        array = LandmarkObservationArray()
        array.header.stamp = msg.header.stamp
        array.header.frame_id = self.frame_id
        for observation in observations:
            item = LandmarkObservationMsg()
            item.range = observation.range
            item.bearing = observation.bearing
            item.radius = observation.radius
            item.range_variance = observation.range_variance
            item.bearing_variance = observation.bearing_variance
            array.observations.append(item)
        self.observation_pub.publish(array)

        self.scans += 1
        if self.pose is not None and observations:
            self.update_map(observations)

        self.get_logger().debug(
            '%d observations, %d landmarks in the map (scan %d)'
            % (len(observations), len(self.landmarks), self.scans)
        )

    # ------------------------------------------------------------------
    # Landmark map
    # ------------------------------------------------------------------
    def observation_to_map(self, observation):
        """Map-frame position and 2x2 covariance of one observation."""
        x, y, yaw = self.pose
        robot_x = observation.range * math.cos(observation.bearing)
        robot_y = observation.range * math.sin(observation.bearing)
        offset_x, offset_y = rotate(yaw, robot_x, robot_y)

        # d(x, y) / d(range, bearing) in the robot frame, then in the map frame.
        cos_bearing, sin_bearing = (
            math.cos(observation.bearing),
            math.sin(observation.bearing),
        )
        polar_jacobian = np.array(
            [
                [cos_bearing, -observation.range * sin_bearing],
                [sin_bearing, observation.range * cos_bearing],
            ],
            dtype=float,
        )
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        rotation = np.array(
            [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=float
        )
        jacobian = rotation @ polar_jacobian
        measurement_noise = np.diag(
            [observation.range_variance, observation.bearing_variance]
        )
        covariance = jacobian @ measurement_noise @ jacobian.T

        # The pose itself is uncertain too: rotate its xy block into the map.
        pose_xy = self.pose_covariance[:2, :2]
        covariance = covariance + rotation @ pose_xy @ rotation.T
        covariance = 0.5 * (covariance + covariance.T)
        return float(x + offset_x), float(y + offset_y), covariance

    def update_map(self, observations):
        """Associate observations and update/create the landmark map entries."""
        estimates = [self.landmarks[key] for key in sorted(self.landmarks)]
        associations = associate_observations(
            observations,
            self.pose,
            estimates,
            self.pose_covariance,
            gate=self.gate,
            max_range=self.params.max_range,
        )

        for observation, landmark_id in zip(observations, associations):
            map_x, map_y, covariance = self.observation_to_map(observation)
            if landmark_id is None:
                if len(self.landmarks) >= self.max_landmarks:
                    continue
                landmark_id = self.next_id
                self.next_id += 1
                self.landmarks[landmark_id] = LandmarkEstimate(
                    id=landmark_id,
                    x=map_x,
                    y=map_y,
                    covariance=covariance,
                )
                self.get_logger().info(
                    'new landmark %d at (%.2f, %.2f)'
                    % (landmark_id, map_x, map_y)
                )
            else:
                self.landmarks[landmark_id] = self.merge(
                    self.landmarks[landmark_id], map_x, map_y, covariance
                )

    def merge(self, landmark, map_x, map_y, covariance):
        """Exponential moving average of a landmark position and covariance."""
        alpha = min(max(self.map_alpha, 1e-3), 1.0)
        merged_covariance = (
            (1.0 - alpha) * landmark.covariance + alpha * covariance
        )
        merged_covariance = 0.5 * (merged_covariance + merged_covariance.T)
        return LandmarkEstimate(
            id=landmark.id,
            x=(1.0 - alpha) * landmark.x + alpha * map_x,
            y=(1.0 - alpha) * landmark.y + alpha * map_y,
            covariance=merged_covariance,
        )

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def publish_map(self):
        if self.pose is None:
            return

        stamp = self.get_clock().now().to_msg()
        message = LandmarkMap()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame_id

        markers = MarkerArray()
        for landmark_id in sorted(self.landmarks):
            landmark = self.landmarks[landmark_id]

            item = LandmarkMsg()
            item.id = landmark.id
            item.x = landmark.x
            item.y = landmark.y
            item.covariance = [
                float(landmark.covariance[0, 0]),
                float(landmark.covariance[0, 1]),
                float(landmark.covariance[1, 0]),
                float(landmark.covariance[1, 1]),
            ]
            message.landmarks.append(item)

            markers.markers.append(
                self.make_cylinder_marker(landmark, stamp)
            )
            markers.markers.append(self.make_label_marker(landmark, stamp))

        self.map_pub.publish(message)
        self.marker_pub.publish(markers)
        self.get_logger().debug(
            'published %d landmarks, %d markers'
            % (len(message.landmarks), len(markers.markers))
        )

    def make_cylinder_marker(self, landmark, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = stamp
        marker.ns = 'landmarks'
        marker.id = int(landmark.id)
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(landmark.x)
        marker.pose.position.y = float(landmark.y)
        marker.pose.position.z = 0.5 * MARKER_HEIGHT
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = MARKER_HEIGHT
        marker.color.r = 0.10
        marker.color.g = 0.75
        marker.color.b = 0.25
        marker.color.a = 0.9
        return marker

    def make_label_marker(self, landmark, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = stamp
        marker.ns = 'landmark_labels'
        marker.id = int(landmark.id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(landmark.x)
        marker.pose.position.y = float(landmark.y)
        marker.pose.position.z = MARKER_HEIGHT + 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.text = str(landmark.id)
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkNode()
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
