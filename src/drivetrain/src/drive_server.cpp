
#include "action_interfaces/action/drive.hpp"
#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"
#include "SparkMax.hpp"
#include "SparkFlex.hpp"
#include "SparkBase.hpp"
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

      left_motor.SetIdleMode(IdleMode::kBrake);
      left_motor.SetMotorType(MotorType::kBrushless);
      // left_motor.SetSmartCurrentFreeLimit(50.0);
      left_motor.SetSmartCurrentStallLimit(80.0); // 0.8 Nm
      left_motor.BurnFlash();

      right_motor.SetIdleMode(IdleMode::kBrake);
      right_motor.SetMotorType(MotorType::kBrushless);
      right_motor.SetInverted(true);
      // left_motor.SetSmartCurrentFreeLimit(50.0);
      left_motor.SetSmartCurrentStallLimit(80.0); // 0.8 Nm
      RCLCPP_INFO(this->get_logger(), "Drive action server is ready");
    }

  private:
    rclcpp_action::Server<Drive>::SharedPtr action_server_;
    // hardware::TalonFX drive_left{20, "can0"};
    // hardware::TalonFX drive_right{21, "can0"};
    // controls::DutyCycleOut drive_left_duty{0.0};
    // controls::DutyCycleOut drive_right_duty{0.0};
    double drive_left_duty = 0;
    double drive_right_duty = 0;
    double driven_dist = 0;
    double drive_factor = 0; //TODO: set this constant
    SparkMax left_motor{"can1", 10};
    SparkMax right_motor{"can1", 11};
    // Motor 1
    bool has_goal{false};
    bool driving_dist{false};
    int loop_rate_hz{120};
    double track_width{1.0};
    double normalization_constant = 1; // change this during testing
    std::shared_ptr<GoalHandleDrive> Drive_Goal_Handle;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Drive::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->velocity_goal.linear.x); // change to drive specific
      (void)uuid;
      if (!has_goal)
      {
        RCLCPP_INFO(this->get_logger(), "Accepted Goal and Will soon Execute it");
        has_goal = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "rejected goal, there must be one still executing");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDrive> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      // Stop motors immediately
      left_motor.SetDutyCycle(0.0);
      right_motor.SetDutyCycle(0.0);
      RCLCPP_INFO(this->get_logger(), "MOTORS STOPPED");

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


    void drive_dist(double goal_dist){
      driven_dist += drive_factor * (left_motor.GetPosition() + right_motor.GetPosition())/2;
      
        left_motor.SetDutyCycle(0.3);
        right_motor.SetDutyCycle(0.3);
      
      left_motor.SetSetpoint(goal_dist);
      right_motor.SetSetpoint(goal_dist);
      if(left_motor.GetPosition() > goal_dist && right_motor.GetPosition() > goal_dist){
        left_motor.SetDutyCycle(0.0);
        right_motor.SetDutyCycle(0.0);
        driving_dist = false;
      RCLCPP_INFO(this->get_logger(), "I'VE GONE FAR ENOUGH %f", driven_dist);
        return;
      }
      driving_dist = true;
      RCLCPP_INFO(this->get_logger(), "driving dist %f", driven_dist);

    }

    std::vector<double> curvatureDrive(double linear_speed, double z_rotation)
    {
      linear_speed = std::clamp(linear_speed, -1.0, 1.0);
      z_rotation = std::clamp(z_rotation, -1.0, 1.0);

      double left_speed = linear_speed - z_rotation;
      double right_speed = linear_speed + z_rotation;

      // this desaturates
      //
      double max_magnitude = std::max(std::abs(left_speed), std::abs(right_speed));
      if (max_magnitude > 1)
      {
        left_speed /= max_magnitude;
        right_speed /= max_magnitude;
      }
      std::vector<double> speeds = {left_speed, right_speed};
      return speeds;
    }
    void execute(const std::shared_ptr<GoalHandleDrive> goal_handle)
    {

      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(loop_rate_hz); // this should be 20 hz which I can't imagine not being enough for the dump

      const auto goal = goal_handle->get_goal();

      auto feedback = std::make_shared<Drive::Feedback>();
      auto result = std::make_shared<Drive::Result>();
      double linear = goal->velocity_goal.linear.x;
      double angular = goal->velocity_goal.angular.z;

      double v_left = linear - angular;
      double v_right = linear + angular;

      auto start_time = this->now();
      auto end_time = start_time + rclcpp::Duration::from_seconds(0.1);

      while (rclcpp::ok() && this->now() < end_time)
      {
        if (goal_handle->is_canceling())
        {
          RCLCPP_INFO(this->get_logger(), "Goal is canceling");
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          Drive_Goal_Handle = nullptr;
          has_goal = false;
          return;
        }
        left_motor.Heartbeat();
        right_motor.Heartbeat();
        if (!goal->drive_auto)
        {
          left_motor.SetDutyCycle(std::min(std::max(v_left, -1.), 1.));
          right_motor.SetDutyCycle(std::min(std::max(v_right, -1.), 1.));
          feedback->inst_velocity.linear.x = v_left;
          feedback->inst_velocity.angular.z = v_right; // placeholders
        }

        if(abs(goal->distance_meters) > 0 ){
          drive_dist(goal->distance_meters);
          driving_dist = true;
          if(){

          }
        }

        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
      }

      // drive_left_duty.Output = 0.0;
      // drive_right_duty.Output = 0.0;
      // drive_left.SetControl(drive_left_duty);
      // drive_right.SetControl(drive_right_duty);

      if (rclcpp::ok())
      {
        result->curr_velocity = goal->velocity_goal;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        Drive_Goal_Handle = nullptr;
        has_goal = false;
      }
    }
  }; // class DriveActionServer

}; // namespace drive_server

RCLCPP_COMPONENTS_REGISTER_NODE(drive_server::DriveActionServer)
