
#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#include "SparkBase.hpp"
#include "SparkFlex.hpp"
#include "SparkMax.hpp"
#include "action_interfaces/action/drive.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float32.hpp"

namespace drive_server
{
class DriveActionServer : public rclcpp::Node
{
 public:
  using Drive = action_interfaces::action::Drive;
  using GoalHandleDrive = rclcpp_action::ServerGoalHandle<Drive>;

  explicit DriveActionServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
    Node("drive_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Drive>(
      this, "drive", std::bind(&DriveActionServer::handle_goal, this, _1, _2),
      std::bind(&DriveActionServer::handle_cancel, this, _1),
      std::bind(&DriveActionServer::handle_accepted, this, _1));

    left_motor_.SetIdleMode(IdleMode::kBrake);
    left_motor_.SetMotorType(MotorType::kBrushless);
    // left_motor.SetSmartCurrentFreeLimit(50.0);
    left_motor_.SetSmartCurrentStallLimit(80.0); // 0.8 Nm
    left_motor_.BurnFlash();

    right_motor_.SetIdleMode(IdleMode::kBrake);
    right_motor_.SetMotorType(MotorType::kBrushless);
    right_motor_.SetInverted(true);
    // left_motor.SetSmartCurrentFreeLimit(50.0);
    left_motor_.SetSmartCurrentStallLimit(80.0); // 0.8 Nm
    RCLCPP_INFO(this->get_logger(), "Drive action server is ready");
  }

 private:
  rclcpp_action::Server<Drive>::SharedPtr action_server_;
  // hardware::TalonFX drive_left{20, "can0"};
  // hardware::TalonFX drive_right{21, "can0"};
  // controls::DutyCycleOut drive_left_duty{0.0};
  // controls::DutyCycleOut drive_right_duty{0.0};
  double drive_left_duty_ = 0;
  double drive_right_duty_ = 0;
  SparkMax left_motor_{"can1", 10};
  SparkMax right_motor_{"can1", 11};
  // Motor 1
  bool has_goal_{false};
  int loop_rate_hz_{120};
  double track_width_{1.0};
  double normalization_constant_ = 1; // change this during testing
  std::shared_ptr<GoalHandleDrive> drive_goal_handle_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    const std::shared_ptr<const Drive::Goal>& goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %f",
                goal->velocity_goal.linear.x); // change to drive specific
    (void)uuid;
    if (!has_goal_) {
      RCLCPP_INFO(this->get_logger(), "Accepted Goal and Will soon Execute it");
      has_goal_ = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    RCLCPP_INFO(this->get_logger(),
                "rejected goal, there must be one still executing");
    return rclcpp_action::GoalResponse::REJECT;
  }

  private:
    rclcpp_action::Server<Drive>::SharedPtr action_server_;
    // hardware::TalonFX drive_left{20, "can0"};
    // hardware::TalonFX drive_right{21, "can0"};
    // controls::DutyCycleOut drive_left_duty{0.0};
    // controls::DutyCycleOut drive_right_duty{0.0};
        double drive_left_duty = 0;
        double drive_right_duty = 0;
        SparkMax left_motor{"can1", 10};
        SparkMax right_motor{"can1", 11};
        // Motor 1
    bool has_goal{false};
    int loop_rate_hz{120};
    double track_width{1.0};
    double normalization_constant = 1; //change this during testing
    std::shared_ptr<GoalHandleDrive> Drive_Goal_Handle;
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDrive>& GOAL_HANDLE)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // Stop motors immediately
    left_motor_.SetDutyCycle(0.0);
    right_motor_.SetDutyCycle(0.0);
    RCLCPP_INFO(this->get_logger(), "MOTORS STOPPED");

    has_goal_ = false;
    (void)GOAL_HANDLE;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDrive>& GOAL_HANDLE)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&DriveActionServer::execute, this, _1), GOAL_HANDLE}
      .detach();
  }

  std::vector<double> curvature_drive(double linear_speed, double z_rotation)
  {
    linear_speed = std::clamp(linear_speed, -1.0, 1.0);
    z_rotation = std::clamp(z_rotation, -1.0, 1.0);

    double left_speed = linear_speed - z_rotation;
    double right_speed = linear_speed + z_rotation;

    // this desaturates
    //
    double const MAX_MAGNITUDE =
      std::max(std::abs(left_speed), std::abs(right_speed));
    if (MAX_MAGNITUDE > 1) {
      left_speed /= MAX_MAGNITUDE;
      right_speed /= MAX_MAGNITUDE;
    }
    std::vector<double> speeds = {left_speed, right_speed};
    return speeds;
  }

  void execute(const std::shared_ptr<GoalHandleDrive>& GOAL_HANDLE)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(
      loop_rate_hz_); // this should be 20 hz which I can't imagine not being
                      // enough for the dump

    const auto GOAL = GOAL_HANDLE->get_goal();

    auto feedback = std::make_shared<Drive::Feedback>();
    auto result = std::make_shared<Drive::Result>();
    double const LINEAR = GOAL->velocity_goal.linear.x;
    double const ANGULAR = GOAL->velocity_goal.angular.z;

    double const V_LEFT = LINEAR - ANGULAR;
    double const V_RIGHT = LINEAR + ANGULAR;

    auto start_time = this->now();
    auto end_time = start_time + rclcpp::Duration::from_seconds(0.1);

    while (rclcpp::ok() && this->now() < end_time) {
      if (GOAL_HANDLE->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal is canceling");
        GOAL_HANDLE->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        drive_goal_handle_ = nullptr;
        has_goal_ = false;
        return;
      }
      left_motor_.Heartbeat();
      right_motor_.Heartbeat();
      left_motor_.SetDutyCycle(std::min(std::max(V_LEFT, -1.), 1.));
      right_motor_.SetDutyCycle(std::min(std::max(V_RIGHT, -1.), 1.));
      feedback->inst_velocity.linear.x = V_LEFT;
      feedback->inst_velocity.angular.z = V_RIGHT; // placeholders
      GOAL_HANDLE->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // drive_left_duty.Output = 0.0;
    // drive_right_duty.Output = 0.0;
    // drive_left.SetControl(drive_left_duty);
    // drive_right.SetControl(drive_right_duty);

    if (rclcpp::ok()) {
      result->curr_velocity = GOAL->velocity_goal;
      GOAL_HANDLE->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      drive_goal_handle_ = nullptr;
      has_goal_ = false;
    }
  }
}; // class DriveActionServer

} // namespace drive_server

RCLCPP_COMPONENTS_REGISTER_NODE(drive_server::DriveActionServer)
