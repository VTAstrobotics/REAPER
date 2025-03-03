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
        self.timer_1 = self.create_timer(1.0, self.read_encoder_value_1)
        self.timer_2 = self.create_timer(1.0, self.read_encoder_value_2)

    def read_encoder_value_1(self):
        # Create a CAN bus instance
        bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # CAN message for the first encoder value
        message = can.Message(arbitration_id=0x01, is_extended_id=False, data=[0x04, 0x01, 0x01, 0x00])

        try:
            # Send the message
            bus.send(message)
            self.get_logger().info(f"Message 1 sent: {message}")

            # Wait for the response
            response = bus.recv(1.0)  # 1 second timeout

            if response:
                self.get_logger().info(f"Response 1 received: {response}")
                # Decode the encoder value
                encoder_value = (response.data[6] << 24) | (response.data[5] << 16) | (response.data[4] << 8) | response.data[3]
                encoder_value = (encoder_value * 360) / 1024
                encoder_value = ((encoder_value * 2 * 3.14159) / 360) /(2*3.14159)

                self.get_logger().info(f"Encoder 1 Value: {encoder_value}")

                # Publish the encoder value
                self.encoder_publisher_1.publish(Float64(data=encoder_value))
            else:
                self.get_logger().warn("No response for Message 1.")
        
        except Exception as e:
            self.get_logger().error(f"Error in reading encoder value 1: {e}")
        
        finally:
            bus.shutdown()

    def read_encoder_value_2(self):
        # Create a CAN bus instance
        bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # CAN message for the second encoder value
        message2 = can.Message(arbitration_id=0x01, is_extended_id=False, data=[0x04, 0x01, 0x01, 0x00])

        try:
            # Send the second message
            bus.send(message2)
            self.get_logger().info(f"Message 2 sent: {message2}")

            # Wait for the response
            response2 = bus.recv(1.0)  # 1 second timeout

            if response2:
                self.get_logger().info(f"Response 2 received: {response2}")
                # Decode the second encoder value
                encoder_value = (response2.data[6] << 24) | (response2.data[5] << 16) | (response2.data[4] << 8) | response2.data[3]
                encoder_value = (encoder_value * 360) / 1024
                encoder_value = ((encoder_value * 2 * 3.14159) / 360) / (2 * 3.14159)

                self.get_logger().info(f"Encoder 2 Value: {encoder_value}")

                # Publish the second encoder value
                self.encoder_publisher_2.publish(Float64(data=encoder_value))
            else:
                self.get_logger().warn("No response for Message 2.")
        
        except Exception as e:
            self.get_logger().error(f"Error in reading encoder value 2: {e}")
        
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
