import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import pandas as pd
from time import sleep
import datetime


from state_messages.msg import MotorState
from action_interfaces.action import Dig


class system_identifier(Node):

    def __init__(self):
        super().__init__('system_identifier')
        self.action_client = ActionClient(self, Dig, 'dig')
        self.period = .005
        self.time = 0
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.left_data = {"VoltageApplied": [], "VoltageInput":[], "Velocity":[], "time":[]}
        self.right_data = {"VoltageApplied": [], "VoltageInput":[], "Velocity":[], "time": []}

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
        self.left_data["VoltageApplied"].append(self.currentOutput)
        self.left_data["VoltageInput"].append(msg.input_voltage)
        self.left_data["Velocity"].append(msg.current_speed)
        self.left_data["time"].append(self.time)
    def right_logger(self, msg):
        self.right_data["VoltageApplied"].append(self.currentOutput)
        self.right_data["VoltageInput"].append(msg.input_voltage)
        self.right_data["Velocity"].append(msg.current_speed)
        self.right_data["time"].append(self.time)
    def timer_callback(self):
        self.time += self.period
        print("wrote file")
        self.currentOutput += self.period * .1
        self.send_goal(self.currentOutput)


    def write_data(self):
        (pd.DataFrame.from_dict(data=self.left_data, orient='columns').to_csv('left_data.csv', header=True, mode='x'))
        (pd.DataFrame.from_dict(data=self.right_data, orient='columns').to_csv('right_data.csv', header=True, mode='x'))

    def send_goal(self, power_out):
        goal_msg = Dig.Goal()
        goal_msg.link_pwr_goal = power_out

        self.action_client.wait_for_server()

        return self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    time = datetime.datetime.now()
    minimal_subscriber = system_identifier()

    while((datetime.datetime.now() - time).seconds < 20):


        rclpy.spin_once(minimal_subscriber)
    minimal_subscriber.write_data()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()