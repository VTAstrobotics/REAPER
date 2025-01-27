
#include "action_interfaces/action/dump.hpp"
#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace action_interfaces::action;
using namespace ctre::phoenix6;
namespace drive_server
{
  class DriveActionServer : public rclcpp::Node
  {
  public:
    using Drive = action_interfaces::action::Drive;
    using GoalHandleDrive = rclcpp_action::ServerGoalHandle<Drive>;

    explicit DriveActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("drive_action_server", options)
    {
      using namespace std::placeholders;

      this->action_server_ = rclcpp_action::create_server<Drive>(
          this,
          "drive",
          std::bind(&DriveActionServer::handle_goal, this, _1, _2),
          std::bind(&DriveActionServer::handle_cancel, this, _1),
          std::bind(&DriveActionServer::handle_accepted, this, _1));
      RCLCPP_INFO(this->get_logger(), "Drive action server is ready");
    }

  private:
    rclcpp_action::Server<Drive>::SharedPtr action_server_;
    hardware::TalonFX drive_left_{20, "can0"};
    hardware::TalonFX drive_right_{21, "can0"};
    controls::DutyCycleOut drive_left_duty_{0.0};
    controls::DutyCycleOut drive_right_duty_{0.0};

    bool has_goal{false};
    int loop_rate_hz{20};
    double track_width_{1.0};
    std::shared_ptr<GoalHandleDrive> Drive_Goal_Handle;
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Drive::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->velocity_goal.linear.x); // change to drive specific
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
        const std::shared_ptr<GoalHandleDrive> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
         // Stop motors immediately
        drive_left_duty_.Output = 0.0;
        drive_right_duty_.Output = 0.0;
        drive_left_.SetControl(drive_left_duty_);
        drive_right_.SetControl(drive_right_duty_);

        has_goal = false;
        (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDrive> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&DriveActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleDrive> goal_handle)
    {

      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(loop_rate_hz); // this should be 20 hz which I can't imagine not being enough for the dump

      const auto goal = goal_handle->get_goal();

      auto feedback = std::make_shared<Drive::Feedback>();
      auto result = std::make_shared<Drive::Result>();
      double linear  = goal->velocity_goal.linear.x;
      double angular = goal->velocity_goal.angular.z;
      
      double v_left  = linear  - 0.5 * angular * track_width_;
      double v_right = linear  + 0.5 * angular * track_width_;

      auto &amountDone = feedback->inst_velocity;
      while (goal->curr_velocity <= goal->velocity_goal)
      {
        if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal is canceling");
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                Drive_Goal_Handle = nullptr;  // Reset the active goal
                has_goal = false;
                return;
            }

        double speed = goal->velocity_goal;
        auto result = std::make_shared<Drive::Result>();
        ctre::phoenix::unmanaged::FeedEnable(pow(static_cast<float>(loop_rate_hz), -1));
        driveLeftDutyCycle.Output = leftPower;
        driveRightDutyCycle.Output = rightPower;
        //compare current velocity to goal velocity with a PID controller
        driveLeft.SetControl(driveLeftDutyCycle);
        driveRight.SetControl(driveRightDutyCycle); //duty cycle is between 0-1. 
        RCLCPP_INFO(this->get_logger(), "The motors should be running");
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();

      } 
        has_goal = false;
     if (rclcpp::ok())
        {
         result->curr_velocity = volume_deposited;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
          volume_deposited = 0;
          Drive_Goal_Handle = nullptr;
          has_goal = false;

        }
    }
  }; // class DumpActionServer

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(drive_server::DriveActionServer)