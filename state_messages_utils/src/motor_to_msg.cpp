#include "state_messages_utils/motor_to_msg.hpp"

using namespace state_messages;

motor_to_msg::motor_to_msg(rclcpp::Node::SharedPtr node, std::string motor_name)
{
    this->node_to_publish_under = node;
    this->state_publisher = node->create_publisher<msg::MotorState>("motor_name/state", 2);
}

kraken_to_msg::kraken_to_msg(rclcpp::Node::SharedPtr node, std::string motor_name, hardware::TalonFX& motor, float frequency)
:motor_to_msg(node , motor_name){
    auto period = std::chrono::duration<double>(1/frequency);
    this->timer = node->create_wall_timer(period, std::bind(&kraken_to_msg::publish_state, this));
    this->motor = motor;


    
}


