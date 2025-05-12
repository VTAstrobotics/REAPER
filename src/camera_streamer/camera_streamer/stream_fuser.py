import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from action_interfaces.action import Fuser
import re
class ImageSelectorNode(Node):
    def __init__(self):
        super().__init__('image_selector')
        self.get_logger().info('ImageSelector node starting up')
        self.topic_pattern = re.compile(r'^/usbcam_image_.+')
        self.topic_prefix = "/usbcam_image_"
        self.topics = []
        self.index = 0
        # Publisher for the selected image
        self.publisher = self.create_publisher(Image, '/driver/selected_image', 10)
        # Discover and subscribe to image topics
        self.update_topics()
        # Timer to periodically refresh topic list
        self.create_timer(1.0, self.update_topics)
        # Action server for increment/decrement
        self._action_server = ActionServer(
            self,
            Fuser,
            'change_image',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )
        self.get_logger().info('ChangeImage action server initialized')

    def update_topics(self):
        """Discover image topics and (un)subscribe as needed."""
        names_types = self.get_topic_names_and_types()
        found = [name for name, types in names_types
                 if name.startswith(self.topic_prefix) and 'sensor_msgs/msg/Image' in types]
        # Add new subscriptions
        for name in found:
            if name not in self.topics:
                self.get_logger().info(f'New image topic discovered: {name}')
                # subscribe with a callback capturing the topic name
                sub = self.create_subscription(
                    Image, name,
                    lambda msg, t=name: self.image_callback(msg, t), 10)
                self.topics.append(name)
        # Remove lost subscriptions (optional)
        for name in list(self.topics):
            if name not in found:
                self.get_logger().info(f'Image topic lost: {name}')
                self.destroy_subscription(self.subscriptions[name])
                self.topics.remove(name)
                # Adjust index if necessary
                self.index %= max(1, len(self.topics))
        # Sort topics for deterministic order (optional)
        self.topics.sort()

    def image_callback(self, msg, topic):
        """Re-publish messages from the selected topic."""
        if topic == self.topics[self.index]:
            # Forward the image
            self.publisher.publish(msg)

    def goal_callback(self, goal_request):
        """Accept only valid goals."""
        cmd = goal_request.command
        if cmd not in (-1, 0, 1):
            self.get_logger().warn(f'Rejecting invalid command: {cmd}')
            return GoalResponse.REJECT
        self.get_logger().info(f'Goal received: {cmd}')
        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute an increment/decrement command."""
        cmd = goal_handle.request.command
        num = len(self.topics)
        if num == 0:
            # No topics to cycle through
            goal_handle.succeed()
            result = Fuser.Result()
            result.success = False
            result.message = 'No image topics available'
            return result
        # Compute new index with wrap-around
        if cmd == 1:
            self.index = (self.index + 1) % num
        elif cmd == -1:
            self.index = (self.index - 1) % num
        new_topic = self.topics[self.index]
        self.get_logger().info(f'Image selection changed to {new_topic}')
        goal_handle.succeed()
        result = Fuser.Result()
        result.success = True
        result.message = f'Selected image: {new_topic}'
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSelectorNode()
    # Use a MultiThreadedExecutor to handle subscriptions and action concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.get_logger().info('Shutting down ImageSelector')
    node.destroy()
    rclpy.shutdown()
