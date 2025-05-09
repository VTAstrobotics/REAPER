#include <utility>

#include "state_messages_utils/motor_to_msg.hpp"

using namespace state_messages_utils;
using namespace state_messages;

namespace state_messages_utils
{

motor_to_msg::motor_to_msg(const rclcpp::Node::SharedPtr& node,
                           const std::string& motor_name) :
  node_to_publish_under(node)
{
  this->msg = state_messages::msg::MotorState();
  std::string const NAME = motor_name + "/state";
  this->state_publisher = node->create_publisher<msg::MotorState>(NAME, 2);
}

void motor_to_msg::publish_state() {} // this prevents build errors

motor_to_msg::~motor_to_msg() = default;

/**
 * creates a publisher attached to the passing node of topic name
 * motor_name/state also publishes a motor state message at frequncy Hz
 */
kraken_to_msg::kraken_to_msg(const rclcpp::Node::SharedPtr& node,
                             const std::string& motor_name,
                             hardware::TalonFX* motor, float frequency) :
  motor_to_msg(node, motor_name), motor(motor)
{
  auto period = std::chrono::duration<double>(1 / frequency);
  this->timer = node->create_wall_timer(
    period, std::bind(&kraken_to_msg::publish_state, this));
}

state_messages_utils::kraken_to_msg::~kraken_to_msg() = default;

void kraken_to_msg::publish_state()
{
  float const POSITION = this->motor->GetPosition().GetValueAsDouble();
  float const CURRENT = this->motor->GetTorqueCurrent().GetValueAsDouble();
  float const OUTPUT_VOLTAGE =
    this->motor->GetMotorVoltage().GetValueAsDouble();
  float const INPUT_VOLTAGE =
    this->motor->GetSupplyVoltage().GetValueAsDouble();
  float const VELOCITY = this->motor->GetVelocity().GetValueAsDouble();

  this->msg.current_applied_voltage = OUTPUT_VOLTAGE;
  this->msg.input_voltage = INPUT_VOLTAGE;
  this->msg.current_current = CURRENT;
  this->msg.current_speed = VELOCITY;
  this->msg.current_position = POSITION;

  this->state_publisher->publish(this->msg);
}
} // namespace state_messages_utils
