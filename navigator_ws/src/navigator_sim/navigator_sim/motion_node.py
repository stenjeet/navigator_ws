#!/usr/bin/env python3
"""Ground-truth motion model (report section 3).

Simulates a differential-drive robot along a reproducible square trajectory
and publishes the true state: pose, heading, velocities and wheel angles, plus
the odom -> base_footprint transform.

Outputs:
    /ground_truth/odom    nav_msgs/Odometry        true pose + twist
    /ground_truth/path    nav_msgs/Path            accumulated true trajectory
    /cmd_vel              geometry_msgs/Twist      commanded v, omega
    /joint_states         sensor_msgs/JointState   wheel angles
    /tf                   odom -> base_footprint
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')

        self.declare_parameter('start_x', 1.5)
        self.declare_parameter('start_y', 1.5)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('side_length', 4.0)
        self.declare_parameter('linear_velocity', 0.35)
        self.declare_parameter('angular_velocity', 0.60)
        self.declare_parameter('time_step', 0.05)
        self.declare_parameter('wheel_radius', 0.09)
        self.declare_parameter('wheel_base', 0.42)
        self.declare_parameter('loops', 1)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_yaw = float(self.get_parameter('start_yaw').value)
        self.side_length = float(self.get_parameter('side_length').value)
        self.linear_velocity = float(self.get_parameter('linear_velocity').value)
        self.angular_velocity = float(self.get_parameter('angular_velocity').value)
        self.dt = float(self.get_parameter('time_step').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.loops = int(self.get_parameter('loops').value)

        # State
        self.x = self.start_x
        self.y = self.start_y
        self.yaw = self.start_yaw
        self.v = 0.0
        self.w = 0.0
        self.wheel_left_angle = 0.0
        self.wheel_right_angle = 0.0

        # Segment list: 4 straights + 4 in-place turns of +pi/2 per loop.
        self.segments = []
        for _ in range(max(1, self.loops)):
            for _ in range(4):
                self.segments.append(('straight', self.side_length))
                self.segments.append(('turn', math.pi / 2.0))
        self.seg_index = 0
        self.seg_remaining = self.segments[0][1] if self.segments else 0.0

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.path_pub = self.create_publisher(Path, '/ground_truth/path', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            'motion_node: square trajectory, %d segment(s)' % len(self.segments)
        )

    def timer_callback(self):
        self.step()
        self.publish()

    def step(self):
        """Advance the state by one integration step (clipping the last step
        of each segment to land exactly on the target)."""
        if self.seg_index >= len(self.segments):
            self.v = 0.0
            self.w = 0.0
            return

        kind, total = self.segments[self.seg_index]

        if kind == 'straight':
            v, w = self.linear_velocity, 0.0
            rate = self.linear_velocity
        else:
            v, w = 0.0, self.angular_velocity
            rate = self.angular_velocity

        step = rate * self.dt
        if step >= self.seg_remaining:
            step = self.seg_remaining
            dt_used = step / rate if rate > 0.0 else 0.0
        else:
            dt_used = self.dt

        self.x += v * math.cos(self.yaw) * dt_used
        self.y += v * math.sin(self.yaw) * dt_used
        self.yaw += w * dt_used

        # Differential-drive inverse kinematics -> wheel angular speeds.
        v_left = v - 0.5 * w * self.wheel_base
        v_right = v + 0.5 * w * self.wheel_base
        self.wheel_left_angle += (v_left / self.wheel_radius) * dt_used
        self.wheel_right_angle += (v_right / self.wheel_radius) * dt_used

        self.v = v
        self.w = w

        self.seg_remaining -= step
        if self.seg_remaining <= 1e-9:
            self.seg_index += 1
            if self.seg_index < len(self.segments):
                self.seg_remaining = self.segments[self.seg_index][1]
            else:
                # Snap back to the exact start pose (loop closure).
                self.x = self.start_x
                self.y = self.start_y
                self.yaw = self.start_yaw
                self.v = 0.0
                self.w = 0.0

    def publish(self):
        now = self.get_clock().now().to_msg()
        quat = yaw_to_quaternion(self.yaw)

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        # Path
        pose = odom.pose.pose
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = 'map'
        ps.pose = pose
        self.path.header.stamp = now
        self.path.poses.append(ps)
        self.path_pub.publish(self.path)

        # Commanded velocity
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = self.w
        self.cmd_pub.publish(cmd)

        # Joint states
        js = JointState()
        js.header.stamp = now
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.wheel_left_angle, self.wheel_right_angle]
        self.joint_pub.publish(js)

        # TF odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
