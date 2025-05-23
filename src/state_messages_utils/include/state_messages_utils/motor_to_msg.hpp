#include "ctre/phoenix6/TalonFX.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "state_messages/msg/motor_state.hpp"

using namespace ctre::phoenix6;

namespace state_messages_utils
{
class motor_to_msg
{
 protected:
  rclcpp::Node::SharedPtr node_to_publish_under;
  state_messages::msg::MotorState msg;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<state_messages::msg::MotorState>::SharedPtr state_publisher;

 public:
  motor_to_msg(const rclcpp::Node::SharedPtr& node,
               const std::string& motor_name);
  virtual ~motor_to_msg();
  virtual void publish_state();
};

class kraken_to_msg : public motor_to_msg
{
 private:
  hardware::TalonFX* motor;
  double input_voltage_{0}; // BUS VOLTAGE
  double current_speed_{
    0}; // in units/S either m/s or rot/s or rad/s; clarify which in your code
  double current_current_{0}; // in Amps
  double current_applied_voltage_{
    0}; // in volts, this is the duty cycle * input voltage
  double current_position_{
    0}; // the current position in m, rots, or radians, clarify which in your
        // code         Float64 current_current
 public:
  kraken_to_msg(const rclcpp::Node::SharedPtr& node,
                const std::string& motor_name, hardware::TalonFX* motor,
                float frequency);
  ~kraken_to_msg() override;
  void publish_state() override;
};

} // namespace state_messages_utils
