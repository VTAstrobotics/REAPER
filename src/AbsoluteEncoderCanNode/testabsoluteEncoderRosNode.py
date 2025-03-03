import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')
        
        # Publishers for the two different encoder values
        self.encoder_publisher_1 = self.create_publisher(Float64, '/dig/link', 10)
        self.encoder_publisher_2 = self.create_publisher(Float64, '/dig/bckt', 10)
        self.get_logger().info("Encoder Reader Node has been started.")
        
        # Timers for reading the two encoder values at 1 Hz
        self.create_timer(1.0, lambda: self.read_encoder_value(0x01, self.encoder_publisher_1))
        self.create_timer(1.0, lambda: self.read_encoder_value(0x02, self.encoder_publisher_2))

    def read_encoder_value(self, encoder_id, publisher):
        # Create a CAN bus instance
        bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # CAN message to request encoder value
        message = can.Message(arbitration_id=encoder_id, is_extended_id=False, data=[0x04, encoder_id, 0x01, 0x00])

        try:
            # Send the message
            bus.send(message)
            self.get_logger().info(f"Message sent to Encoder {encoder_id}: {message}")

            # Wait for the response
            response = bus.recv(1.0)  # 1 second timeout

            if response and response.arbitration_id == encoder_id:
                self.get_logger().info(f"Response received from Encoder {encoder_id}: {response}")

                # Decode the encoder value
                encoder_value = (response.data[6] << 24) | (response.data[5] << 16) | (response.data[4] << 8) | response.data[3]
                encoder_value = (encoder_value * 360) / 1024  # Convert to degrees
                encoder_value = (encoder_value * 2 * 3.14159) / 360  # Convert to radians

                self.get_logger().info(f"Encoder {encoder_id} Value: {encoder_value}")

                # Publish the encoder value
                publisher.publish(Float64(data=encoder_value))
            else:
                self.get_logger().warn(f"No response or wrong ID for Encoder {encoder_id}.")
        
        except Exception as e:
            self.get_logger().error(f"Error in reading encoder value for Encoder {encoder_id}: {e}")
        
        finally:
            bus.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # Create the encoder reader node
    encoder_reader = EncoderReader()

    try:
        # Spin to keep the node running
        rclpy.spin(encoder_reader)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        encoder_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
