#include "state_messages_utils/motor_to_msg.hpp"

using namespace state_messages;

motor_to_msg::motor_to_msg(rclcpp::Node::SharedPtr node, std::string motor_name)
{
    this->node_to_publish_under = node;
    this->msg = state_messages::msg::MotorState();
    this->state_publisher = node->create_publisher<msg::MotorState>("motor_name/state", 2);
}
/**
 * creates a publisher attached to the passing node of topic name motor_name/state also publishes a motor state message at frequncy Hz
 */
kraken_to_msg::kraken_to_msg(rclcpp::Node::SharedPtr node, std::string motor_name, hardware::TalonFX* motor, float frequency)
:motor_to_msg(node , motor_name){
    auto period = std::chrono::duration<double>(1/frequency);
    this->timer = node->create_wall_timer(period, std::bind(&kraken_to_msg::publish_state, this));
    this->motor = motor;
    
}

void kraken_to_msg::publish_state(){
    float position = this->motor->GetPosition().GetValueAsDouble();
    float current = this->motor->GetTorqueCurrent().GetValueAsDouble();
    float output_voltage = this->motor->GetMotorVoltage().GetValueAsDouble();
    float input_voltage = this->motor->GetSupplyVoltage().GetValueAsDouble();
    float velocity = this->motor->GetVelocity().GetValueAsDouble();

    this->msg.current_applied_voltage = output_voltage;
    this->msg.input_voltage = input_voltage;
    this->msg.current_current = current;
    this->current_speed = velocity;
    this->current_position = position;

    this->state_publisher->publish(this->msg);
}


