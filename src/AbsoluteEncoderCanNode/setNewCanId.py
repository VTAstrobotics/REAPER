import can

def set_encoder_id(old_id, new_id, channel='can0'):
    # Create a CAN bus instance
    bus = can.interface.Bus(channel=channel, bustype='socketcan')

    # CAN message to set the encoder ID
    # This is a generic example, modify the data bytes as per your encoder's manual
    message = can.Message(
        arbitration_id=old_id, 
        is_extended_id=False, 
        data=[0x04, 0x01, 0x02, new_id]  # Example: Command to set new ID
    )

    try:
        # Send the message
        bus.send(message)
        print(f"Set ID message sent: {message}")

        # Wait for acknowledgment (optional, depends on encoder behavior)
        response = bus.recv(1.0)  # 1 second timeout

        if response:
            print(f"Response received: {response}")
            print("Encoder ID set successfully.")
        else:
            print("No response received. Check if the ID was set correctly.")
    
    except Exception as e:
        print(f"Error in setting encoder ID: {e}")
    
    finally:
        bus.shutdown()

if __name__ == '__main__':
    old_id = 0x01  # Current ID of the encoder
    new_id = 0x02  # New ID you want to set
    set_encoder_id(old_id, new_id)
