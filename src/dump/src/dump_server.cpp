
#include "action_interfaces/action/dump.hpp"
#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANBus.hpp"
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

    explicit DumpActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("dump_action_server", options)
    {
      using namespace std::placeholders;

      this->action_server_ = rclcpp_action::create_server<Dump>(
          this,
          "dump",
          std::bind(&DumpActionServer::handle_goal, this, _1, _2),
          std::bind(&DumpActionServer::handle_cancel, this, _1),
          std::bind(&DumpActionServer::handle_accepted, this, _1));
      RCLCPP_INFO(this->get_logger(), "Action server is ready");

      configs::CurrentLimitsConfigs limConfig{};
      limConfig.SupplyCurrentLimit = 30;
      limConfig.SupplyCurrentLimitEnable = true;
      conveyorMotor.GetConfigurator().Apply(limConfig);

    }

  private:
    rclcpp_action::Server<Dump>::SharedPtr action_server_;
    hardware::TalonFX conveyorMotor{20, "can0"};
    controls::DutyCycleOut conveyorDutyCycle{0};
    float volume_deposited{0};
    bool has_goal{false};
    int loop_rate_hz{20};
    std::shared_ptr<GoalHandleDump> Dump_Goal_Handle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr volume_description = this->create_subscription<std_msgs::msg::Float32>(
      "/dump/volume", 2, std::bind(&DumpActionServer::dump_volume_callback, this, std::placeholders::_1));
    float starting_volume{-802000};// if this is negative 8020 then it means that we have not reseeded the starting volume for the run. Note that even the absolute value is an entirely unrealistic volume

    void dump_volume_callback(const std_msgs::msg::Float32 msg){
      RCLCPP_INFO(this->get_logger(), "I have recived %f", msg.data);
      if(abs(abs(starting_volume) - 802000) < 2){
        starting_volume = msg.data;
        RCLCPP_INFO(this->get_logger(), "starting volume is now %f", msg.data);

      }
      else{
        volume_deposited = starting_volume - msg.data;
        RCLCPP_INFO(this->get_logger(), "I have deposited %f", volume_deposited);

      }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Dump::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->deposition_goal);
      (void)uuid;
      if(!has_goal){
        RCLCPP_INFO(this->get_logger(),"Accepted Goal and Will soon Execute it");
        has_goal = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      else{
        RCLCPP_INFO(this->get_logger(),"rejected goal, there must be one still executing");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDump> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        conveyorDutyCycle.Output = 0;
        Dump_Goal_Handle = nullptr;
        has_goal = false;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDump> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&DumpActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleDump> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(loop_rate_hz); // this should be 20 hz which I can't imagine not being enough for the dump
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<Dump::Feedback>();
      auto result = std::make_shared<Dump::Result>();
      auto &amountDone = feedback->percent_done;
      while (volume_deposited <= goal->deposition_goal)
      {
        if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal is canceling");
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                Dump_Goal_Handle = nullptr;  // Reset the active goal
                has_goal = false;
                return;
            }

        double speed = goal->deposition_goal;
        auto result = std::make_shared<Dump::Result>();
        ctre::phoenix::unmanaged::FeedEnable(pow(static_cast<float>(loop_rate_hz), -1));
        conveyorDutyCycle.Output = speed;
        conveyorMotor.SetControl(conveyorDutyCycle);
        RCLCPP_INFO(this->get_logger(), "The motor should be running");
        amountDone = volume_deposited/goal->deposition_goal * 100;

        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();

      }
        has_goal = false;
     if (rclcpp::ok())
        {
         result->est_deposit_goal = volume_deposited;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
          volume_deposited = 0;
          Dump_Goal_Handle = nullptr;
          has_goal = false;

        }
    }
  }; // class DumpActionServer

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(dump_server::DumpActionServer)
