#!/usr/bin/env python3
import os
import math
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

class FieldVisualizer(Node):
    def __init__(self):
        super().__init__('field_visualizer')

        # --- which arena to load ---
        self.declare_parameter('arena', 'KSC')
        arena = self.get_parameter('arena').value
        self.get_logger().info(f"Loading arena '{arena}'")

        # --- load YAML ---
        pkg_share = get_package_share_directory('field_description')
        yml = os.path.join(pkg_share, 'config', 'arenas.yaml')
        try:
            with open(yml, 'r') as f:
                cfg = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Could not open {yml}: {e}")
            rclpy.shutdown()
            return

        arenas = cfg.get('arenas', {})
        if arena not in arenas:
            self.get_logger().error(f"Arena '{arena}' not found in {yml}")
            rclpy.shutdown()
            return

        arena_cfg = arenas[arena]
        self.frames = arena_cfg.get('frames', [])
        self.shapes = arena_cfg.get('shapes', {})

        # --- static TF broadcaster ---
        self.broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        # 1) broadcast all your pre‑defined frames
        for f in self.frames:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = f['parent']
            t.child_frame_id  = f['child']
            t.transform.translation.x = float(f.get('x', 0.0))
            t.transform.translation.y = float(f.get('y', 0.0))
            t.transform.translation.z = float(f.get('z', 0.0))
            yaw = float(f.get('yaw', 0.0))
            qz, qw = math.sin(yaw/2.0), math.cos(yaw/2.0)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.broadcaster.sendTransform(t)
        self.get_logger().info(f"Published {len(self.frames)} static transforms for named frames")

        # 2) broadcast one extra frame per shape, at its center
        shape_tf_count = 0
        for name, s in self.shapes.items():
            cx, cy = s.get('center', [0.0,0.0])
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'field_origin'       # shapes are defined relative to field_origin
            t.child_frame_id  = f"{name}_center"     # e.g. "starting_zone_center"
            t.transform.translation.x = float(cx)
            t.transform.translation.y = float(cy)
            t.transform.translation.z = 0.0
            # no rotation needed for a point
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(t)
            shape_tf_count += 1
        self.get_logger().info(f"Published {shape_tf_count} static transforms for shape centers")

        # --- marker publisher ---
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        # publish once immediately, then every second
        self.publish_markers()
        self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        marray = MarkerArray()
        mid = 0

        for name, s in self.shapes.items():
            m = Marker()
            m.header.frame_id = 'field_origin'
            m.header.stamp    = now
            m.ns             = name
            m.id             = mid; mid += 1
            m.action         = Marker.ADD

            shape_type = s.get('type', 'rectangle')
            if shape_type == 'rectangle':
                m.type = Marker.CUBE
                cx, cy    = s['center']
                w, h      = s['size']
                m.pose.position.x = cx
                m.pose.position.y = cy
                m.pose.position.z = 0.01
                m.scale.x = w
                m.scale.y = h
                m.scale.z = 0.02

            elif shape_type == 'circle':
                m.type = Marker.CYLINDER
                cx, cy    = s['center']
                r         = s['radius']
                m.pose.position.x = cx
                m.pose.position.y = cy
                m.pose.position.z = 0.01
                m.scale.x = r*2.0
                m.scale.y = r*2.0
                m.scale.z = 0.02

            else:
                self.get_logger().warn(f"Unknown shape type '{shape_type}' for '{name}'")
                continue

            # simple green translucent
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.4

            marray.markers.append(m)

        self.marker_pub.publish(marray)
        self.get_logger().debug(f"Published {len(marray.markers)} markers")

def main(args=None):
    rclpy.init(args=args)
    node = FieldVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
