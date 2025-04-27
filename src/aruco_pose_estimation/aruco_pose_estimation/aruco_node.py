import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import cv2
import os
import imutils
import numpy as np
import math
import argparse
class Quaternion:
    w: float
    x: float
    y: float
    z: float

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q 
class PosePublisher(Node):
    def __init__(self, camera):
        super().__init__(f'pose_publisher_{camera}')
        

        self.pose_publisher = self.create_publisher(Pose, f'pose_{camera}', 10)
        self.stop_subscription = self.create_subscription(
            Bool,
            'stop_aruco',
            self.stop_callback,
            10)
        self.image_subscription = self.create_subscription(
            Image,
            f'usbcam_image_{camera}',
            self.image_callback,
            10)
            
        self.bridge = CvBridge()
        self.stop = False


        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # Load camera calibration data
        datapath = "/workspace/src/aruco_pose_estimation/data"
        paramPath = os.path.join(datapath, "matrixanddist.npz")
        if not os.path.exists(paramPath):
            self.get_logger().error(".npz path does not exist")
            return
        data = np.load(paramPath)
        self.cameraMatrix = data['matrix']
        self.distCoeffs = data['distortion']
        
        self.markerLength = 0.1  # 10 cm tag size
        self.objectPoints = np.array([
            [-self.markerLength / 2, self.markerLength / 2, 0],
            [self.markerLength / 2, self.markerLength / 2, 0],
            [self.markerLength / 2, -self.markerLength / 2, 0],
            [-self.markerLength / 2, -self.markerLength / 2, 0]
        ], dtype=np.float32)
    def stop_callback(self, msg):
        self.stop = msg.data
        if(self.stop):
            self.get_logger().info("Stopped")
    
        else:
            self.get_logger().info("Resumed")

    def image_callback(self, msg):
        self.get_logger().info("Received image")


        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgra8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgra8')

        if self.stop:
            return
        try:
            frame = imutils.resize(frame, width=600)
            corners, ids, _ = self.arucoDetector.detectMarkers(frame)

            if ids is not None and len(corners) > 0:
                for markerCorner, markerID in zip(corners, ids.flatten()):
                    imagePoints = markerCorner.reshape((4, 2))
                    success, rvec, tvec = cv2.solvePnP(
                        self.objectPoints,
                        imagePoints,
                        self.cameraMatrix,
                        self.distCoeffs
                    )
                    
                    if success:

                        pose_msg = Pose()
                        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = tvec.flatten()
                        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = rvec.flatten()[:3]
                        pose_msg.orientation.w = 0.0 
                        quaternion = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
                        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = quaternion.x, quaternion.y, quaternion.z 
                        pose_msg.orientation.w = quaternion.w

                        self.pose_publisher.publish(pose_msg)
                        self.get_logger().info(str(pose_msg))
                        self.get_logger().info(f"Published pose")

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 USB Camera Node')
    parser.add_argument('--cam', type=int, default=0, help='Index of the camera (default is 0)')
    cli_args = parser.parse_args()
    rclpy.init(args=args)
    pose_publisher = PosePublisher(camera=cli_args.cam)

    rclpy.spin(pose_publisher)

    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
