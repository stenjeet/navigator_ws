#!/usr/bin/env python3
"""UWB range model (report section 4.2).

Computes the true range from the UWB tag to each anchor, then applies per-anchor
bias, Gaussian noise and independent Bernoulli dropouts. Ideal and noisy arrays
are published separately so that the noisy stream can be compared against the
reference.

Scenario hooks (stage 9, defaults reproduce the nominal model exactly):

    anchors          flat x, y, z triples; an empty list means world.UWB_ANCHORS
    outage_start_s   start of a full outage window on the node clock [s]
    outage_end_s     end of the window; every range is NaN inside it and
                     ``0.0 / 0.0`` disables the outage

Outputs:
    /uwb/ideal_ranges   std_msgs/Float32MultiArray   [A1, A2, A3, A4]
    /uwb/noisy_ranges   std_msgs/Float32MultiArray   NaN marks a dropout
    /uwb/anchors        visualization_msgs/MarkerArray
"""

import math
import random

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

from navigator_sim import world

NAN = float('nan')

#: Default anchor layout of the polygon as a flat x, y, z list. It is the code
#: default because ROS cannot infer the type of an empty YAML sequence: writing
#: ``anchors: []`` to a parameter file makes the parameter NOT_SET.
DEFAULT_ANCHORS = [
    float(value) for anchor in world.UWB_ANCHORS for value in anchor[1:4]
]


class UwbModelNode(Node):
    def __init__(self):
        super().__init__('uwb_model_node')

        self.declare_parameter('random_seed', 12025)
        self.declare_parameter('uwb_sigma', 0.05)
        self.declare_parameter('uwb_dropout_probability', 0.03)
        self.declare_parameter('uwb_biases', [0.02, -0.03, 0.04, 0.01])
        self.declare_parameter('uwb_rate_hz', 10.0)
        # Flat x, y, z triples; the default is the report polygon
        # (world.UWB_ANCHORS). Do not set an empty list in YAML: ROS stores it
        # as an uninitialized parameter.
        self.declare_parameter('anchors', DEFAULT_ANCHORS)
        # UWB outage window [s] on the node clock, relative to the first timer
        # callback; 0.0 / 0.0 disables it.
        self.declare_parameter('outage_start_s', 0.0)
        self.declare_parameter('outage_end_s', 0.0)

        seed = int(self.get_parameter('random_seed').value)
        self.sigma = float(self.get_parameter('uwb_sigma').value)
        self.dropout_prob = float(self.get_parameter('uwb_dropout_probability').value)
        self.biases = [float(b) for b in self.get_parameter('uwb_biases').value]
        rate_hz = float(self.get_parameter('uwb_rate_hz').value)

        # Reproducible RNG (report section 4.4).
        self.rng = random.Random(seed)

        self.anchors = self.parse_anchors()
        self.tag_offset = world.UWB_TAG_OFFSET

        self.outage_start_s = float(self.get_parameter('outage_start_s').value)
        self.outage_end_s = float(self.get_parameter('outage_end_s').value)
        self.first_callback_time = None

        if len(self.biases) != len(self.anchors):
            self.get_logger().warn(
                'uwb_biases length (%d) != number of anchors (%d)'
                % (len(self.biases), len(self.anchors))
            )

        # Latest ground-truth pose.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_pose = False

        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10
        )
        self.ideal_pub = self.create_publisher(Float32MultiArray, '/uwb/ideal_ranges', 10)
        self.noisy_pub = self.create_publisher(Float32MultiArray, '/uwb/noisy_ranges', 10)
        self.anchors_pub = self.create_publisher(MarkerArray, '/uwb/anchors', 10)

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.timer_callback)
        self.publish_anchors()
        self.get_logger().info(
            'uwb_model_node: rate %.1f Hz, seed %d, %d anchors, outage %s'
            % (
                rate_hz,
                seed,
                len(self.anchors),
                'off' if not self.outage_enabled()
                else '%.1f..%.1f s' % (self.outage_start_s, self.outage_end_s),
            )
        )

    def parse_anchors(self):
        """Anchors from the ``anchors`` parameter, or the polygon defaults.

        An empty or uninitialized parameter falls back to ``world.UWB_ANCHORS``;
        the latter happens when a parameter file sets ``anchors: []`` because
        ROS cannot infer the type of an empty YAML sequence.
        """
        try:
            flat = [
                float(value) for value in self.get_parameter('anchors').value
            ]
        except ParameterUninitializedException:
            flat = []
        if not flat:
            return world.UWB_ANCHORS
        if len(flat) % 3 != 0:
            raise ValueError(
                'anchors must be a flat list of x, y, z triples, got %d values'
                % len(flat)
            )
        return [
            ('A%d' % (index + 1), flat[index], flat[index + 1], flat[index + 2])
            for index in range(0, len(flat), 3)
        ]

    def outage_enabled(self):
        """True when the configured window is a valid, non-empty interval."""
        return (
            math.isfinite(self.outage_start_s)
            and math.isfinite(self.outage_end_s)
            and self.outage_end_s > self.outage_start_s
        )

    def in_outage(self):
        """True while the node clock is inside the configured outage window."""
        if not self.outage_enabled() or self.first_callback_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self.first_callback_time
        ).nanoseconds * 1e-9
        return self.outage_start_s <= elapsed < self.outage_end_s

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.have_pose = True

    def tag_position(self):
        dx, dy, _ = self.tag_offset
        cos_t = math.cos(self.yaw)
        sin_t = math.sin(self.yaw)
        x_tag = self.x + dx * cos_t - dy * sin_t
        y_tag = self.y + dx * sin_t + dy * cos_t
        return x_tag, y_tag, self.tag_offset[2]

    def timer_callback(self):
        # The outage window is relative to the first callback, so it does not
        # depend on how long the node took to receive the first pose.
        if self.first_callback_time is None:
            self.first_callback_time = self.get_clock().now()

        if not self.have_pose:
            return

        x_tag, y_tag, z_tag = self.tag_position()
        ideal = Float32MultiArray()
        noisy = Float32MultiArray()
        # Keep the RNG call order identical whether or not the outage is
        # active, so only the NaN pattern differs between the scenarios.
        outage = self.in_outage()

        for i, (name, ax, ay, az) in enumerate(self.anchors):
            d = math.sqrt(
                (x_tag - ax) ** 2 + (y_tag - ay) ** 2 + (z_tag - az) ** 2
            )
            ideal.data.append(float(d))

            if self.rng.random() < self.dropout_prob:
                noisy.data.append(NAN)
            else:
                bias = self.biases[i] if i < len(self.biases) else 0.0
                noise = self.rng.gauss(0.0, self.sigma)
                noisy.data.append(NAN if outage else float(d + bias + noise))

        ideal.layout.dim = []
        noisy.layout.dim = []
        self.ideal_pub.publish(ideal)
        self.noisy_pub.publish(noisy)

    def publish_anchors(self):
        array = MarkerArray()
        marker_id = 0
        for (name, x, y, z) in self.anchors:
            anchor = Marker()
            anchor.header.frame_id = 'map'
            anchor.header.stamp = self.get_clock().now().to_msg()
            anchor.ns = 'uwb_anchors'
            anchor.id = marker_id
            anchor.type = Marker.CYLINDER
            anchor.action = Marker.ADD
            anchor.pose.position.x = float(x)
            anchor.pose.position.y = float(y)
            anchor.pose.position.z = float(z / 2.0)
            anchor.pose.orientation.w = 1.0
            anchor.scale.x = 0.24
            anchor.scale.y = 0.24
            anchor.scale.z = float(z)
            anchor.color.r = 1.0
            anchor.color.g = 0.55
            anchor.color.b = 0.0
            anchor.color.a = 1.0
            array.markers.append(anchor)
            marker_id += 1

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = 'uwb_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x)
            label.pose.position.y = float(y)
            label.pose.position.z = float(z + 0.25)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.28
            label.color.r = 0.1
            label.color.g = 0.1
            label.color.b = 0.1
            label.color.a = 1.0
            label.text = name
            array.markers.append(label)
            marker_id += 1

        self.anchors_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = UwbModelNode()
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
