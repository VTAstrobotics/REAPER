
import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from ROS 2
import cv2  # Import OpenCV for computer vision tasks
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS and OpenCV images
from sensor_msgs.msg import Image  # Import the Image message type from sensor_msgs
import argparse  # Import argparse for command-line arguments

def filter_usbcam_images(topics: list[str]) -> list[str]:
    return [topic for topic in topics if topic.startswith('/usbcam_image')]

class stream_fuser(Node):
    def __init__(self):
        super().__init__(f'fuser')  # Initialize the Node with the name 'webcam_node'
        self.subscribers = []
        self.selected_camera = ""
        self.timer = self.create_timer(1, self.get_all_cameras)
        self.pub_timer = self.create_timer(50**-1, self.publish_image)
        self.camera_publisher = self.create_publisher(str, "/driver/image")
        self.selected_cam_sub = self.create_subscription(str, "selected_camera",self.change_selected_camera, 1)
    def change_selected_camera(self, msg):
        self.selected_cam = msg.data
    def get_all_cameras(self):
        topics_and_types = self.get_topic_names_and_types()
        self.get_logger().info(f"camera topics = {topics_and_types}")
        all_names = [name for name, _ in topics_and_types]
        cameras  = filter_usbcam_images(all_names)
        self.get_logger().info(f"camera topics = {cameras}")
        for i in cameras:
            if i not in self.subscribers:
                self.subscribers.append(i)
                
    def publish_image(self):
        self

            
        
        
        
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = stream_fuser()  # Create an instance of the CameraNode with the specified camera index
    try:
        rclpy.spin(node)  # Spin the node to keep it alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Allow the user to exit with Ctrl+C
    
    


if __name__ == '__main__':
    main()  # Run the main function if this script is executed
