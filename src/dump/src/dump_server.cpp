
#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#include "action_interfaces/action/dump.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace action_interfaces::action;
using namespace ctre::phoenix6;

namespace dump_server
{
class DumpActionServer : public rclcpp::Node
{
 public:
  using Dump = action_interfaces::action::Dump;
  using GoalHandleDump = rclcpp_action::ServerGoalHandle<Dump>;

  explicit DumpActionServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
    Node("dump_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Dump>(
      this, "dump", std::bind(&DumpActionServer::handle_goal, this, _1, _2),
      std::bind(&DumpActionServer::handle_cancel, this, _1),
      std::bind(&DumpActionServer::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Action server is ready");

    configs::CurrentLimitsConfigs lim_config{};
    lim_config.SupplyCurrentLimit = 30;
    lim_config.SupplyCurrentLimitEnable = true;
    conveyor_motor_.GetConfigurator().Apply(lim_config);
  }

 private:
  rclcpp_action::Server<Dump>::SharedPtr action_server_;
  hardware::TalonFX conveyor_motor_{30, "can0"};
  controls::DutyCycleOut conveyor_duty_cycle_{0};
  float volume_deposited_{0};
  bool has_goal_{false};
  int loop_rate_hz_{20};
  std::shared_ptr<GoalHandleDump> dump_goal_handle_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr volume_description_ =
    this->create_subscription<std_msgs::msg::Float32>(
      "/dump/volume", 2,
      std::bind(&DumpActionServer::dump_volume_callback, this,
                std::placeholders::_1));
  float starting_volume_{
    -802000}; // if this is negative 8020 then it means that we have not
              // reseeded the starting volume for the run. Note that even
              // the absolute value is an entirely unrealistic volume

  void dump_volume_callback(const std_msgs::msg::Float32 MSG)
  {
    RCLCPP_INFO(this->get_logger(), "I have recived %f", MSG.data);
    if (abs(abs(starting_volume_) - 802000) < 2) {
      starting_volume_ = MSG.data;
      RCLCPP_INFO(this->get_logger(), "starting volume is now %f", MSG.data);
    } else {
      volume_deposited_ = starting_volume_ - MSG.data;
      RCLCPP_INFO(this->get_logger(), "I have deposited %f", volume_deposited_);
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    const std::shared_ptr<const Dump::Goal>& goal)
  {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with order %f m^3 or %f power",
                goal->deposition_goal, goal->pwr_goal);
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

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDump>& GOAL_HANDLE)
  {
    (void)GOAL_HANDLE; // for unused warning
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    conveyor_duty_cycle_.Output = 0;
    dump_goal_handle_ = nullptr;
    has_goal_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDump>& GOAL_HANDLE)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin
    // up a new thread
    std::thread{std::bind(&DumpActionServer::execute, this, _1), GOAL_HANDLE}
      .detach();
  }

  void execute(const std::shared_ptr<GoalHandleDump>& GOAL_HANDLE)
  {
    const auto GOAL = GOAL_HANDLE->get_goal();

    if (GOAL->auton) {
      RCLCPP_DEBUG(this->get_logger(), "execute: control using force sensor");
      execute_with_force(GOAL_HANDLE);
    }

    else {
      RCLCPP_DEBUG(this->get_logger(), "execute: manual power control");
      execute_pwr_dump(GOAL_HANDLE);
    }
  }

  void execute_with_force(const std::shared_ptr<GoalHandleDump>& GOAL_HANDLE)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(
      loop_rate_hz_); // this should be 20 hz which I can't imagine not
                      // being enough for the dump
    const auto GOAL = GOAL_HANDLE->get_goal();
    auto feedback = std::make_shared<Dump::Feedback>();
    auto result = std::make_shared<Dump::Result>();
    auto& amount_done = feedback->percent_done;
    while (volume_deposited_ <= GOAL->deposition_goal) {
      if (GOAL_HANDLE->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal is canceling");
        GOAL_HANDLE->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        dump_goal_handle_ = nullptr; // Reset the active goal
        has_goal_ = false;
        return;
      }

      double const SPEED = GOAL->deposition_goal;
      auto result = std::make_shared<Dump::Result>();
      ctre::phoenix::unmanaged::FeedEnable(
        pow(static_cast<float>(loop_rate_hz_), -1));
      conveyor_duty_cycle_.Output = SPEED;
      conveyor_motor_.SetControl(conveyor_duty_cycle_);
      RCLCPP_INFO(this->get_logger(), "The motor should be running");
      amount_done = volume_deposited_ / GOAL->deposition_goal * 100;

      GOAL_HANDLE->publish_feedback(feedback);
      loop_rate.sleep();
    }
    has_goal_ = false;
    if (rclcpp::ok()) {
      result->est_deposit_goal = volume_deposited_;
      GOAL_HANDLE->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      volume_deposited_ = 0;
      dump_goal_handle_ = nullptr;
      has_goal_ = false;
    }
  }

  void execute_pwr_dump(const std::shared_ptr<GoalHandleDump>& GOAL_HANDLE)
  {
    RCLCPP_DEBUG(this->get_logger(), "execute_pwr: executing...");

    const auto GOAL = GOAL_HANDLE->get_goal();
    double const POWER_GOAL = GOAL->pwr_goal;

    auto feedback = std::make_shared<Dump::Feedback>();
    auto result = std::make_shared<Dump::Result>();
    auto& amount_done = feedback->percent_done;

    // check that goal is allowable (duty cycle takes [-1, 1])
    if (POWER_GOAL < -1 || POWER_GOAL > 1) {
      RCLCPP_ERROR(this->get_logger(),
                   "execute_pwr_dump: Power was out of bounds. Power "
                   "goals should always be in [-1, 1]");
    }

    if (GOAL_HANDLE->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal is canceling");
      GOAL_HANDLE->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      dump_goal_handle_ = nullptr; // Reset the active goal
      has_goal_ = false;
      return;
    }

    ctre::phoenix::unmanaged::FeedEnable(1000 *
                                         (1.0 / (double)(loop_rate_hz_)));

    conveyor_duty_cycle_.Output = POWER_GOAL;
    conveyor_motor_.SetControl(conveyor_duty_cycle_);
    amount_done = 100;

    GOAL_HANDLE->publish_feedback(feedback);

    if (rclcpp::ok()) {
      result->est_deposit_goal = POWER_GOAL;

      GOAL_HANDLE->succeed(result);
      RCLCPP_INFO(this->get_logger(), "execute_pwr: Goal succeeded");
    } else {
      RCLCPP_ERROR(this->get_logger(), "execute_pwr: Goal failed");
    }

    dump_goal_handle_ = nullptr;
    has_goal_ = false;
  }
}; // class DumpActionServer

} // namespace dump_server

RCLCPP_COMPONENTS_REGISTER_NODE(dump_server::DumpActionServer)
