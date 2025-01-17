
#include "action_interfaces/action/dump.hpp"
#include <functional>
#include <memory>
#include <thread>

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
    }

  private:
    rclcpp_action::Server<Dump>::SharedPtr action_server_;
    hardware::TalonFX conveyorMotor{20, "can0"};
    controls::DutyCycleOut conveyorDutyCycle{0};
    float volume_deposited{0};
    explicit bool has_goal{false};

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
      (void)goal_handle;
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
      rclcpp::Rate loop_rate(20); // this should be 20 hz which I can't imagine not being enough for the dump
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<Dump::Feedback>();
      while (volume_deposited <= goal->deposition_goal)
      {
        auto &amountDone = feedback->percent_done;
        double speed = goal->deposition_goal;
        auto result = std::make_shared<Dump::Result>();
        ctre::phoenix::unmanaged::FeedEnable(50);
        conveyorDutyCycle.Output = speed;
        conveyorMotor.SetControl(conveyorDutyCycle);
        RCLCPP_INFO(this->get_logger(), "The motor should be running");

        loop_rate.sleep();
      } 
        has_goal = false;
     if (rclcpp::ok())
        {
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
          volume_deposited = 0;
        }
    }
  }; // class DumpActionServer

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(dump_server::DumpActionServer)