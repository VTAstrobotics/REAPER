#include <utility>

#include "state_messages_utils/motor_to_msg.hpp"

using namespace state_messages_utils;
using namespace state_messages;
namespace state_messages_utils{

motor_to_msg::motor_to_msg(const rclcpp::Node::SharedPtr& node, const std::string& motor_name) : node_to_publish_under(node)
{
    
    this->msg = state_messages::msg::MotorState();
    std::string const name = motor_name + "/state";
    this->state_publisher = node->create_publisher<msg::MotorState>(name, 2);
}

void motor_to_msg::publish_state(){} // this prevents build errors

motor_to_msg::~motor_to_msg()= default;
/**
 * creates a publisher attached to the passing node of topic name motor_name/state also publishes a motor state message at frequncy Hz
 */
kraken_to_msg::kraken_to_msg(const rclcpp::Node::SharedPtr& node, std::string motor_name, hardware::TalonFX* motor, float frequency)
: motor_to_msg(node , std::move(motor_name)), motor(motor) {
    auto period = std::chrono::duration<double>(1/frequency);
    this->timer = node->create_wall_timer(period, std::bind(&kraken_to_msg::publish_state, this));
    

}

state_messages_utils::kraken_to_msg::~kraken_to_msg()
= default;
void kraken_to_msg::publish_state()
{
    float const position = this->motor->GetPosition().GetValueAsDouble();
    float const current = this->motor->GetTorqueCurrent().GetValueAsDouble();
    float const output_voltage = this->motor->GetMotorVoltage().GetValueAsDouble();
    float const input_voltage = this->motor->GetSupplyVoltage().GetValueAsDouble();
    float const velocity = this->motor->GetVelocity().GetValueAsDouble();

    this->msg.current_applied_voltage = output_voltage;
    this->msg.input_voltage = input_voltage;
    this->msg.current_current = current;
    this->msg.current_speed = velocity;
    this->msg.current_position = position;

    this->state_publisher->publish(this->msg);
}
}