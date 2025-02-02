import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class EncoderSubscriber(Node):
    def __init__(self):
        super().__init__('encoder_subscriber')

        # Create a subscriber to the 'encoder_value' topic
        self.subscription = self.create_subscription(
            Float64,                 # Message type
            'encoder_value',         # Topic name
            self.listener_callback,  # Callback function when data is received
            10                        # QoS history depth
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # This function is called every time a new message is received
        self.get_logger().info(f"Received encoder value: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    # Create the subscriber node
    encoder_subscriber = EncoderSubscriber()

    try:
        # Spin to keep the subscriber node running
        rclpy.spin(encoder_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        encoder_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
