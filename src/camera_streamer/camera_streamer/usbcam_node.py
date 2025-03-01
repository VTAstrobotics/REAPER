import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from ROS 2
import cv2  # Import OpenCV for computer vision tasks
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS and OpenCV images
from sensor_msgs.msg import Image  # Import the Image message type from sensor_msgs
import argparse  # Import argparse for command-line arguments
import linuxpy
from linuxpy.video.device import Device
import numpy as np
class CameraNode(Node):
    def __init__(self, cam):
        self.node_name = "usbcam_node_" + str(cam)
        super().__init__(self.node_name)  # Initialize the Node with the name 'webcam_node'
        self.frame_rate = 30
        self.camera = Device.from_id(cam)
        self.publisher_ = self.create_publisher(Image, f'usbcam_image_{cam}' , 3)  # Create a publisher for the camera so that the topic name has the camera number
        self.timer = self.create_timer(self.frame_rate**-1, self.timer_callback)  # Create a timer to call camera such that the framerate is 30
        # self.cap = cv2.VideoCapture(cam ,cv2.CAP_V4L)  # Open the specified webcam // V4L gives 6x the framerate compared to the default
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Set the buffer size to 1 to minimize latency
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)  # Set the frame width to 640 pixels
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)  # Set the frame height to 480 pixels
        # self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)  # Set the frames per second to 60
        self.bridge = CvBridge()  # Initialize the CvBridge to convert between ROS and OpenCV images
        self.camera.open()
    def close(self):
        self.camera.close()
    def timer_callback(self):
        for frame in self.camera:  # Capture a frame from the webcam
            if frame is not None:  # Check if the frame was captured successfully
                image_data = np.frombuffer(bytes(frame), dtype=np.uint8)
                image = image_data.reshape((frame.height,frame.width, 2))
                image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
                msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")  # Convert the OpenCV image to a ROS Image message
                self.publisher_.publish(msg)  # Publish the Image message
                # cv2.imshow('Webcam', frame)  # Display the frame in an OpenCV window
            else:
                self.get_logger().error('Failed to capture image')  # Log an error message if the frame was not captured
            break
def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 USB Camera Node')
    parser.add_argument('--cam', type=int, default=0, help='the number from /dev/video* of the device')

    cli_args = parser.parse_args()

    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = CameraNode(cli_args.cam)  # Create an instance of the CameraNode with the specified camera index
    try:
        rclpy.spin(node)  # Spin the node to keep it alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Allow the user to exit with Ctrl+C
    finally:
        node.camera.close()
        node.destroy_node()  # Destroy the ROS 2 node
        rclpy.shutdown()  # Shut down the ROS 2 Python client library

if __name__ == '__main__':
    main()  # Run the main function if this script is executed
