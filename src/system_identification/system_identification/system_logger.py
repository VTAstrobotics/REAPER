import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import pandas as pd



from state_messages.msg import MotorState
from action_interfaces.action import Dig


class system_identifier(Node):

    def __init__(self):
        super().__init__('system_identifier')
        self.action_client = ActionClient(self, Dig, 'dig')
        self.period = .010
        self.time = 0
        self.timer = self.create_timer(self.period, self.timer_callback)        
        self.left_data = {"VoltageApplied": [], "VoltageInput":[], "velocity":[], "time":[]}
        self.right_data = {"VoltageApplied": [], "VoltageInput":[], "velocity":[], "time": []}
        
        self.currentOutput = 0
        self.left_logger_sub = self.create_subscription(
            MotorState,
            'left_linkage/state',
            self.left_logger,
            2)
        
        self.right_logger_sub = self.create_subscription(
            MotorState,
            "right_linkage/state",
            self.right_logger,
            2
        )

    def left_logger(self, msg):
        self.left_data["VoltageApplied"].append(msg.current_applied_voltage)
        self.left_data["VoltageInput"].append(msg.input_voltage)
        self.left_data["Velocity"].append(msg.current_position)
        self.left_data["time"].append(self.time)
    def right_logger(self, msg):
        self.right_data["VoltageApplied"].append(msg.current_applied_voltage)
        self.right_data["VoltageInput"].append(msg.input_voltage)
        self.right_data["Velocity"].append(msg.current_position)
        self.right_data["time"].append(self.time)
    def timer_callback(self):
        self.time += self.period
        print("wrote file")
        self.currentOutput += self.period * .25
        self.send_goal(self.currentOutput)
        if((int(self.time) % 1) == 0):
            self.write_data() 
        
    def write_data(self):
        (pd.DataFrame.from_dict(data=self.left_data, orient='index').to_csv('left_data.csv', header=True))
        (pd.DataFrame.from_dict(data=self.right_data, orient='index').to_csv('right_data.csv', header=True))
        
    def send_goal(self, power_out):
        goal_msg = Dig.Goal()
        goal_msg.dig_link_pwr_goal = power_out
        
        self.action_client.wait_for_server()
        
        return self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = system_identifier()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()