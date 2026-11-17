#!/usr/bin/env python3
"""Publishes the test polygon (walls, obstacles, UWB anchors, SLAM landmarks)
as a visualization_msgs/MarkerArray on /polygon_markers (report section 2).

Geometry is imported from world.py so the rendered map and the lidar
ray-casting model stay consistent.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from navigator_sim import world


class PolygonMarkersNode(Node):
    def __init__(self):
        super().__init__('polygon_markers_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/polygon_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_polygon)
        self.get_logger().info('Publishing polygon markers on /polygon_markers')

    def _base_marker(self, marker_id, ns, marker_type):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def make_box(self, marker_id, cx, cy, sx, sy, sz, r, g, b, ns='polygon'):
        marker = self._base_marker(marker_id, ns, Marker.CUBE)
        marker.pose.position.x = float(cx)
        marker.pose.position.y = float(cy)
        marker.pose.position.z = float(sz / 2.0)
        marker.scale.x = float(sx)
        marker.scale.y = float(sy)
        marker.scale.z = float(sz)
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0
        return marker

    def make_cylinder(self, marker_id, x, y, radius, height, r, g, b, ns='obstacles'):
        marker = self._base_marker(marker_id, ns, Marker.CYLINDER)
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(height / 2.0)
        marker.scale.x = float(2.0 * radius)
        marker.scale.y = float(2.0 * radius)
        marker.scale.z = float(height)
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0
        return marker

    def make_triangle_prism(self, marker_id, vertices, height, r, g, b):
        """Build a closed triangular prism as a Marker.TRIANGLE_LIST."""
        marker = self._base_marker(marker_id, 'obstacles', Marker.TRIANGLE_LIST)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0

        (x0, y0), (x1, y1), (x2, y2) = vertices
        bottom = [Point(x=xi, y=yi, z=0.0) for xi, yi in vertices]
        top = [Point(x=xi, y=yi, z=float(height)) for xi, yi in vertices]

        pts = []
        # bottom face
        pts += [bottom[0], bottom[1], bottom[2]]
        # top face
        pts += [top[0], top[2], top[1]]
        # three side faces (each split into two triangles)
        for i in range(3):
            j = (i + 1) % 3
            pts += [bottom[i], bottom[j], top[j]]
            pts += [bottom[i], top[j], top[i]]
        marker.points = pts
        return marker

    def make_anchor(self, marker_id, label_id, name, x, y, z):
        anchor = self.make_cylinder(
            marker_id, x, y, radius=0.12, height=1.20,
            r=1.0, g=0.55, b=0.0, ns='uwb_anchors'
        )
        label = self._base_marker(label_id, 'labels', Marker.TEXT_VIEW_FACING)
        label.pose.position.x = float(x)
        label.pose.position.y = float(y)
        label.pose.position.z = float(z + 0.25)
        label.scale.z = 0.28
        label.color.r = 0.1
        label.color.g = 0.1
        label.color.b = 0.1
        label.color.a = 1.0
        label.text = name
        return anchor, label

    def publish_polygon(self):
        array = MarkerArray()
        marker_id = 0

        # Floor
        array.markers.append(
            self.make_box(marker_id, 4.0, 4.0, 8.0, 8.0, 0.05,
                          0.88, 0.88, 0.88, ns='floor')
        )
        marker_id += 1

        # Outer walls (from world.WALLS footprints)
        wall_color = (0.20, 0.20, 0.25)
        for (xmin, xmax, ymin, ymax) in world.WALLS:
            cx = 0.5 * (xmin + xmax)
            cy = 0.5 * (ymin + ymax)
            sx = xmax - xmin
            sy = ymax - ymin
            array.markers.append(
                self.make_box(marker_id, cx, cy, sx, sy, world.WALL_HEIGHT,
                              *wall_color, ns='walls')
            )
            marker_id += 1

        # L-shaped wall (box obstacles)
        obstacle_color = (0.58, 0.22, 0.58)
        for (xmin, xmax, ymin, ymax, height) in world.BOX_OBSTACLES:
            cx = 0.5 * (xmin + xmax)
            cy = 0.5 * (ymin + ymax)
            sx = xmax - xmin
            sy = ymax - ymin
            array.markers.append(
                self.make_box(marker_id, cx, cy, sx, sy, height,
                              *obstacle_color, ns='obstacles')
            )
            marker_id += 1

        # Cylindrical obstacles
        for (x, y, radius, height) in world.CYLINDER_OBSTACLES:
            array.markers.append(
                self.make_cylinder(marker_id, x, y, radius, height,
                                   0.85, 0.15, 0.15)
            )
            marker_id += 1

        # Triangular obstacle
        tri = world.TRIANGLE_OBSTACLE
        array.markers.append(
            self.make_triangle_prism(marker_id, tri['vertices'], tri['height'],
                                     0.15, 0.70, 0.25)
        )
        marker_id += 1

        # UWB anchors
        for index, (name, x, y, z) in enumerate(world.UWB_ANCHORS):
            anchor, label = self.make_anchor(marker_id, 100 + index, name, x, y, z)
            array.markers.append(anchor)
            array.markers.append(label)
            marker_id += 1

        # SLAM landmarks
        for (name, x, y, radius, height, color) in world.LANDMARKS:
            array.markers.append(
                self.make_cylinder(marker_id, x, y, radius, height,
                                   *color, ns='landmarks')
            )
            marker_id += 1

        self.publisher_.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = PolygonMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
