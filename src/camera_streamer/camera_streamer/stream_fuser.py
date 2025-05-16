import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from sensor_msgs.msg import Image
from action_interfaces.action import Fuser

import os
from glob import glob
import cv2
from cv_bridge import CvBridge
import time


def get_serial(device_path: str) -> str:
    base = os.path.basename(device_path)
    last = base.split('_')[-1]
    return last.replace('-video-index0', '')


class ImageSelectorNode(Node):
    def __init__(self):
        super().__init__('usb_cam_selector')
        self.get_logger().info('Starting USB Camera Selector Node')

        self.bridge = CvBridge()
        self.current_index = 0
        self.available_cameras = []
        self.capture = None
        self.current_cam = None

        self.publisher = self.create_publisher(Image, '/driver/selected_image', 10)
        self.timer = self.create_timer(0.02, self.publish_frame)  # Increased timer frequency

        self._action_server = ActionServer(
            self,
            Fuser,
            'change_image',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )

        self.scan_cameras()
        self.start_camera(self.current_index)

    def scan_cameras(self):
        pattern = '/dev/v4l/by-id/usb-046d*_Brio_101_*-video-index0'
        self.available_cameras = sorted(glob(pattern))
        for path in self.available_cameras:
            self.get_logger().info(f'Detected camera: {path}')

    def start_camera(self, index: int):
        if not self.available_cameras:
            self.get_logger().warn('No cameras found')
            return

        cam_path = os.path.realpath(self.available_cameras[index])

        # Only start the camera if it's not already the one being used
        if self.capture and self.current_cam == cam_path:
            self.get_logger().info(f'Camera already selected: {cam_path}')
            return

        # Release the existing capture before opening a new camera
        if self.capture:
            self.capture.release()

        self.capture = cv2.VideoCapture(cam_path)
        self.current_cam = cam_path

        # Set camera properties for better performance (FPS and resolution)
        self.capture.set(cv2.CAP_PROP_FPS, 30)  # Try setting FPS to 30
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set resolution to 640x480
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.capture.isOpened():
            self.get_logger().error(f'Failed to open camera at {cam_path}')
        else:
            self.get_logger().info(f'Opened camera: {cam_path}')

    def publish_frame(self):
        if not self.capture or not self.capture.isOpened():
            return
        ret, frame = self.capture.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)

    def goal_callback(self, goal_request):
        cmd = goal_request.command
        if cmd not in (-1, 1):
            self.get_logger().warn(f'Rejecting invalid command: {cmd}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        cmd = goal_handle.request.command
        num = len(self.available_cameras)
        result = Fuser.Result()

        if num == 0:
            result.success = False
            result.message = 'No cameras available'
            goal_handle.succeed()
            return result

        if cmd == 1:
            self.current_index = (self.current_index + 1) % num
        elif cmd == -1:
            self.current_index = (self.current_index - 1) % num
        # cmd == 0 means no change

        # Only switch camera if it's different from the current one
        if self.available_cameras[self.current_index] != self.current_cam:
            self.start_camera(self.current_index)

        # If we switched or are already on the correct camera, publish the frame
        self.publish_frame()

        result.success = True
        result.message = f"Switched to camera: {get_serial(self.available_cameras[self.current_index])}"
        goal_handle.succeed()
        return result

    def destroy(self):
        if self.capture:
            self.capture.release()
            time.sleep(0.6)  # Give some time for cleanup
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info('Shutting down USB Camera Selector Node')
    node.destroy()
    rclpy.shutdown()
