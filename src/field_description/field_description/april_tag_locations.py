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
        self.KSCx = 0
        self.KSCy = 0
        self.KSCz = 0
        self.UCF_lx = 0
        self.UCF_ly = 0
        self.UCF_lz = 0
        self.UCF_hx = 0
        self.UCF_hy = 0
        self.UCF_hz = 0
        self.IOWAx = 0
        self.IOWAy = 0
        self.IOWAz = 0


        # --- which arena to load ---
        self.declare_parameter('arena', 'KSC')
        self.arena = self.get_parameter('arena').value
        self.get_logger().info(f"Loading arena '{self.arena}'")
        self.positions = {
            "KSC": [self.KSCx, self.KSCy, self.KSCz],
            "UCF_low":[self.UCF_lx, self.UCF_ly, self.UCF_lz,],
            "UCF_high":[self.UCF_hx, self.UCF_hy, self.UCF_hz,],
            "IOWA":[self.IOWAx, self.IOWAy, self.IOWAz,],
            
        }
        super().__init__('static_april_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.pub_april_tag_transform()    

    def pub_april_tag_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'field_origin'
        t.child_frame_id = "tag"

        t.transform.translation.x = self.positions[self.arena][0]
        t.transform.translation.y = self.positions[self.arena][1]
        t.transform.translation.z = self.positions[self.arena][2]
        quat = quaternion_from_euler(
            0,0,0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)



            

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def main(args=None):
    rclpy.init(args=args)
    node = FieldVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()