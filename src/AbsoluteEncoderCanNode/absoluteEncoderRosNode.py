import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')
        # Publisher that will send encoder values as a Float64 message
        self.encoder_publisher = self.create_publisher(Float64, 'encoder_value', 10)
        self.get_logger().info("Encoder Reader Node has been started.")
        
        # Timer to read the encoder value periodically (e.g., every second)
        self.timer = self.create_timer(1.0, self.read_encoder_value)  # 1 Hz rate

    def read_encoder_value(self):
        # Create a CAN bus instance (use 'can0' or your actual CAN interface)
        bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # Create the CAN message to read encoder value
        message = can.Message(arbitration_id=0x01, is_extended_id=False, data=[0x04, 0x01, 0x01, 0x00])
        message2 = can.Message(arbitration_id=0x01, is_extended_id=False, data=[0x04, 0x01, 0x01, 0x00])

        try:
            # Send the message to the bus
            bus.send(message)
            self.get_logger().info(f"Message sent: {message}")

            # Wait for the response from the encoder
            response = bus.recv(1.0)  # 1 second timeout

            if response:
                self.get_logger().info(f"Response received: {response}")
                # Assuming the encoder value is in the last 4 bytes of the response
                encoder_value = (response.data[6] << 24) | (response.data[5] << 16) | (response.data[4] << 8) | response.data[3]
                encoder_value = (encoder_value * 360) / 1024
                encoder_value = (encoder_value * 2*3.14159)/ 360

                self.get_logger().info(f"Encoder Value: {encoder_value}")

                # Publish the encoder value
                self.encoder_publisher.publish(Float64(data=encoder_value))

            else:
                self.get_logger().warn(f"No response from encoder.")
        try:
            # Send the message to the bus
            bus.send(message2)
            self.get_logger().info(f"Message sent: {message}")

            # Wait for the response from the encoder
            response = bus.recv(1.0)  # 1 second timeout

            if response:
                self.get_logger().info(f"Response 2 received: {response}")
                # Assuming the encoder value is in the last 4 bytes of the response
                encoder_value = (response.data[6] << 24) | (response.data[5] << 16) | (response.data[4] << 8) | response.data[3]
                encoder_value = (encoder_value * 360) / 1024
                encoder_value = (encoder_value * 2*3.14159)/ 360

                self.get_logger().info(f"Encoder 2 Value: {encoder_value}")

                # Publish the encoder value
                self.encoder_publisher.publish(Float64(data=encoder_value))

            else:
                self.get_logger().warn(f"No response from encoder 2.")
        except Exception as e:
            self.get_logger().error(f"Error in reading encoder 2 value: {e}")

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
