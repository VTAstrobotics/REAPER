#include "action_interfaces/action/dig.hpp"

#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#include "std_msgs/msg/float32.hpp"

#include "SparkMax.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "../include/utils.h"

using namespace action_interfaces::action;
using namespace ctre::phoenix6;
namespace dig_server
{
  class DigActionServer : public rclcpp::Node
  {
  public:
    using Dig = action_interfaces::action::Dig;
    using GoalHandleDig = rclcpp_action::ServerGoalHandle<Dig>;

    explicit DigActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("dig_action_server", options)
    {
      using namespace std::placeholders;

      this->action_server_ = rclcpp_action::create_server<Dig>(
          this,
          "dig_action",
          std::bind(&DigActionServer::handle_goal, this, _1, _2),
          std::bind(&DigActionServer::handle_cancel, this, _1),
          std::bind(&DigActionServer::handle_accepted, this, _1));

      configs::Slot0Configs linkConfigs{};
      float K_u = 3.9, T_u = 0.04;
      linkConfigs.kP = 0.8 * K_u;
      linkConfigs.kI = 0; // 0; PD controller
      linkConfigs.kD = 0.1 * K_u * T_u;
      l_link_mtr_.GetConfigurator().Apply(linkConfigs);

      configs::Slot0Configs bcktConfigs{};
      K_u = 3.9, T_u = 0.04; // TODO: tune these values for the bucket.
      bcktConfigs.kP = 0.8 * K_u;
      bcktConfigs.kI = 0; // 0; PD controller
      bcktConfigs.kD = 0.1 * K_u * T_u;
      l_bckt_mtr_.GetConfigurator().Apply(bcktConfigs);

      r_link_mtr_.SetControl(controls::Follower{l_link_mtr_.GetDeviceID(), true}); // true because they are mounted inverted
      r_bckt_mtr_.SetControl(controls::Follower{l_bckt_mtr_.GetDeviceID(), false});

      l_vib_mtr_.SetIdleMode(IdleMode::kCoast);
      l_vib_mtr_.SetMotorType(MotorType::kBrushless);
      l_vib_mtr_.BurnFlash();

      r_vib_mtr_.SetIdleMode(IdleMode::kCoast);
      r_vib_mtr_.SetMotorType(MotorType::kBrushless);
      r_vib_mtr_.BurnFlash();

      RCLCPP_DEBUG(this->get_logger(), "Ready for action");
    }

  private:
    rclcpp_action::Server<Dig>::SharedPtr action_server_;

    // linkage actuators
    hardware::TalonFX l_link_mtr_{20, "can0"}; // canid (each motor), can interface (same for all)
    controls::DutyCycleOut l_link_pwr_duty_cycle_{0}; // [-1, 1]
    controls::PositionDutyCycle l_link_pos_duty_cycle_{0 * 0_tr}; // absolute position to reach (in rotations)
    hardware::TalonFX r_link_mtr_{21, "can0"};

    // bucket rotators
    hardware::TalonFX l_bckt_mtr_{30, "can0"};
    controls::DutyCycleOut l_bckt_pwr_duty_cycle_{0};
    controls::PositionDutyCycle l_bckt_pos_duty_cycle_{0 * 0_tr}; // absolute position to reach (in rotations)
    hardware::TalonFX r_bckt_mtr_{40, "can0"};

    // vibration motors
    SparkMax l_vib_mtr_{"can0", 50};
    SparkMax r_vib_mtr_{"can0", 51};

    bool has_goal_{false};
    const int LOOP_RATE_HZ_{15};
    std::shared_ptr<GoalHandleDig> dig_goal_handle_;

    // subs to actuator position topics
    // should always be aligned so only 1 per pair of acts
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr link_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/link", 2, std::bind(&DigActionServer::dig_link_cb, this, std::placeholders::_1));

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bckt_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/bckt", 2, std::bind(&DigActionServer::dig_bckt_cb, this, std::placeholders::_1));

    float starting_link_pos_{-987654}; // if this is negative 987654 then it means that we have not reseeded the starting volume for the run. Note that even the absolute value is an entirely unrealistic volume
    float current_link_pos_{-987654};

    float starting_bckt_pos_{-987654}; // if this is negative 987654 then it means that we have not reseeded the starting volume for the run. Note that even the absolute value is an entirely unrealistic volume
    float current_bckt_pos_{-987654};

    /**
     * this gets us the sensor data for where our linkage actuators are at
     */
    void dig_link_cb(const std_msgs::msg::Float32 msg){
      RCLCPP_INFO(this->get_logger(), "/dig/link: %f", msg.data);
      if(abs(abs(starting_link_pos_) - 987654) < 2){ // first pos
        starting_link_pos_ = msg.data;
        RCLCPP_INFO(this->get_logger(), "starting linkage actuator positions are %f", starting_link_pos_);

      }
      else{
        current_link_pos_ = msg.data;
        RCLCPP_INFO(this->get_logger(), "current linkage actuator positions are %f", current_link_pos_);

      }
    }

    /**
     * this gets us the sensor data for where our rotation motors are at
     */
    void dig_bckt_cb(const std_msgs::msg::Float32 msg){
      RCLCPP_INFO(this->get_logger(), "/dig/bckt: %f", msg.data);
      if(abs(abs(starting_bckt_pos_) - 987654) < 2){ // first pos
        starting_bckt_pos_ = msg.data;
        RCLCPP_INFO(this->get_logger(), "starting rotation motor positions are %f", starting_bckt_pos_);

      }
      else{
        current_bckt_pos_ = msg.data;
        RCLCPP_INFO(this->get_logger(), "current rotation motor positions are %f", current_bckt_pos_);

      }
    }


    /**************************************************************************
     * General action server handling                                         *
     *                                                                        *
     *                                                                        *
     *************************************************************************/

    /**
     *
     */
     rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Dig::Goal> goal)
    {
      // RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->dig_goal);
      (void)uuid, (void)goal; // for unused warning

      if(!has_goal_){
        RCLCPP_INFO(this->get_logger(),"Accepted goal");
        has_goal_ = true;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      else{
        RCLCPP_INFO(this->get_logger(),"Rejected goal because one is still executing");

        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    /**
     *
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle; // for unused warning

      // stop motion
      l_link_pwr_duty_cycle_.Output = 0;
      l_bckt_pwr_duty_cycle_.Output = 0;

      // set class vars
      dig_goal_handle_ = nullptr;
      has_goal_ = false;

      return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     *
     */
    void handle_accepted(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&DigActionServer::execute, this, _1), goal_handle}.detach();
    }

    /**
     * Parses the parameters and calls the appropriate helper function
     */
    void execute(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      const auto goal = goal_handle->get_goal();

      if (goal->auton) {
        RCLCPP_DEBUG(this->get_logger(), "execute: autonomous control");
        execute_auton(goal_handle);

      } else if (goal->pos) {
        RCLCPP_DEBUG(this->get_logger(), "execute: position control");
        execute_pos(goal_handle);

      } else {
        RCLCPP_DEBUG(this->get_logger(), "execute: power control");
        execute_pwr(goal_handle);

      }
    }

    /**************************************************************************
     * Power control handling                                                 *
     *                                                                        *
     *                                                                        *
     *************************************************************************/

    /**
     *
     */
    void execute_pwr(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_DEBUG(this->get_logger(), "execute_pwr: executing...");

      rclcpp::Rate loop_rate(LOOP_RATE_HZ_);
      const auto goal = goal_handle->get_goal();
      double linkage_goal = goal->dig_link_pwr_goal;
      double bucket_goal  = goal->dig_bckt_pwr_goal;

      // check that goal is allowable (duty cycle takes [-1, 1])
      if (linkage_goal < -1 || linkage_goal > 1 ||
          bucket_goal  < -1 || bucket_goal  > 1)
      {
        RCLCPP_ERROR(this->get_logger(), "execute_pwr: Linkage and/or Bucket goal was out of bounds. Power goals should always be in [-1, 1]");
      }

      auto feedback = std::make_shared<Dig::Feedback>();
      auto result = std::make_shared<Dig::Result>();

      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Goal is canceling");
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        dig_goal_handle_ = nullptr;  // Reset the active goal
        has_goal_ = false;
        return;
      }

      auto &linkPercentDone = feedback->percent_link_done;
      auto &bcktPercentDone = feedback->percent_bckt_done;
      RCLCPP_DEBUG(this->get_logger(), "Running for %f ms", 1000 * (1.0/(double)(LOOP_RATE_HZ_))); //this is the correct math with correct units :)
      ctre::phoenix::unmanaged::FeedEnable(1000 * (1.0/(double)(LOOP_RATE_HZ_)));

      l_link_pwr_duty_cycle_.Output = linkage_goal;
      l_bckt_pwr_duty_cycle_.Output = bucket_goal ;

      l_link_mtr_.SetControl(l_link_pwr_duty_cycle_);
      l_bckt_mtr_.SetControl(l_bckt_pwr_duty_cycle_);

      linkPercentDone = 100;
      bcktPercentDone = 100;
      goal_handle->publish_feedback(feedback);

      if (rclcpp::ok())
      {
        result->est_dig_link_goal = linkage_goal;
        result->est_dig_bckt_goal = bucket_goal ;

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "execute_pwr: Goal succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "execute_pwr: Goal failed");
      }

      dig_goal_handle_ = nullptr;
      has_goal_ = false;
      loop_rate.sleep();
    }

    /**************************************************************************
     * Position control handling                                              *
     *                                                                        *
     *                                                                        *
     *************************************************************************/

    /**
     *
     */
    void execute_pos(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_DEBUG(this->get_logger(), "execute_pos: executing...");

      rclcpp::Rate loop_rate(LOOP_RATE_HZ_);
      const auto goal = goal_handle->get_goal();
      double linkage_goal = goal->dig_link_pos_goal;
      double bucket_goal  = goal->dig_bckt_pos_goal;

      auto feedback = std::make_shared<Dig::Feedback>();
      auto result = std::make_shared<Dig::Result>();

      // TODO: need to factor in the absolute encoders (and any other sensor data) from the callback above
      current_link_pos_ = (double)l_link_mtr_.GetPosition().GetValue();
      current_bckt_pos_ = (double)l_bckt_mtr_.GetPosition().GetValue();

      while (!APPROX(current_link_pos_, linkage_goal) ||
             !APPROX(current_bckt_pos_, bucket_goal))
      { // keep sending the request because CTRE's watchdog
        // TODO: need to factor in the absolute encoders (and any other sensor data) from the callback above
        current_link_pos_ = (double)l_link_mtr_.GetPosition().GetValue();
        current_bckt_pos_ = (double)l_bckt_mtr_.GetPosition().GetValue();

        if (goal_handle->is_canceling())
        {
          RCLCPP_INFO(this->get_logger(), "Goal is canceling");
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          dig_goal_handle_ = nullptr;  // Reset the active goal
          has_goal_ = false;
          return;
        }

        auto &linkPercentDone = feedback->percent_link_done;
        auto &bcktPercentDone = feedback->percent_bckt_done;

        RCLCPP_DEBUG(this->get_logger(), "Running for %f ms", 1000 * (1.0/(double)(LOOP_RATE_HZ_))); //this is the correct math with correct units :)
        ctre::phoenix::unmanaged::FeedEnable(1000 * (1.0/(double)(LOOP_RATE_HZ_)));
        units::angle::turn_t linkage_angl{linkage_goal * 1_tr};
        units::angle::turn_t bucket_angl{bucket_goal * 1_tr};

        l_link_pos_duty_cycle_.Position = linkage_angl;
        l_bckt_pos_duty_cycle_.Position = bucket_angl ;

        units::angular_velocity::turns_per_second_t linkage_speed{1};
        units::angular_velocity::turns_per_second_t bucket_speed{1};
        l_link_pos_duty_cycle_.Velocity = linkage_speed; // rotations per sec
        l_bckt_pos_duty_cycle_.Velocity = bucket_speed ; // rotations per sec

        l_link_mtr_.SetControl(l_link_pos_duty_cycle_);
        l_bckt_mtr_.SetControl(l_bckt_pos_duty_cycle_);

        linkPercentDone = (abs(linkage_goal) - abs(current_link_pos_))/abs(linkage_goal) * 100;
        bcktPercentDone = (abs(bucket_goal ) - abs(current_bckt_pos_))/abs(bucket_goal ) * 100;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
      }

      if (rclcpp::ok())
      {
        result->est_dig_link_goal = current_link_pos_;
        result->est_dig_bckt_goal = current_bckt_pos_;

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "execute_pos: Goal succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "execute_pos: Goal failed");
      }

      dig_goal_handle_ = nullptr;
      has_goal_ = false;
    }

    /**************************************************************************
     * Autonomous dig handling                                                *
     *                                                                        *
     *                                                                        *
     *************************************************************************/

    /**
     *
     */
    void execute_auton(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      (void)goal_handle; // for unused warning
      dig_goal_handle_ = nullptr;
      has_goal_ = false;

      // l_vib_mtr.Heartbeat();
      // r_vib_mtr.Heartbeat();

      // l_vib_mtr.SetDutyCycle(0.1);
      // r_vib_mtr.SetDutyCycle(0.1);
    }

  }; // class DigActionServer

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(dig_server::DigActionServer)
