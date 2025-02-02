import can
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


def read_encoder_value():
    # Create a CAN bus instance (use 'can0' or your actual CAN interface)
    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    # Create the CAN message to read encoder value
    message = can.Message(arbitration_id=0x01, is_extended_id=False, data=[0x04, 0x01, 0x01, 0x00])

    # Send the message to the bus
    bus.send(message)
    print("Message sent:", message)

    # Wait for the response from the encoder
    response = bus.recv(1.0)  # 5 second timeout

    if response:
        print("Response received:", response)
        #Assuming the encoder value is in the last 4 bytes of the response
        encoder_value = (response.data[6] << 24) | (response.data[5] << 16) | (response.data[4] << 8) | response.data[3]
        encoder_value = (encoder_value *360)/1024
        print(f"Encoder Value: {encoder_value}")
    else:
        print("No response from encoder.")

    bus.shutdown()

if __name__ == '__main__':
    read_encoder_value()

