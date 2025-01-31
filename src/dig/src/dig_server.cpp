#include "action_interfaces/action/dig.hpp"

#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#include "std_msgs/msg/float32.hpp"

// TODO: add SparkMAX CAN stuff

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
        : Node("dig_action_node", options)
    {
      using namespace std::placeholders;

      this->action_server_ = rclcpp_action::create_server<Dig>(
          this,
          "dig_action_server",
          std::bind(&DigActionServer::handle_goal, this, _1, _2),
          std::bind(&DigActionServer::handle_cancel, this, _1),
          std::bind(&DigActionServer::handle_accepted, this, _1));

      RCLCPP_INFO(this->get_logger(), "DIG: Action server is ready");

    }

  private:
    rclcpp_action::Server<Dig>::SharedPtr action_server_;

    // linkage actuators
    hardware::TalonFX lLinkMotor{10, "can0"}; // canid (each motor), can interface (same for all)
    controls::DutyCycleOut lLinkPwrDutyCycle{0}; // [-1, 1]
    controls::PositionDutyCycle lLinkPosDutyCycle{0 * 0_tr}; // absolute position to reach (in rotations)
    hardware::TalonFX rLinkMotor{20, "can0"};
    // m_follower.Follow(m_leader);
    // m_follower.SetInverted(TalonFXInvertType::FollowMaster);
    // m_strictFollower.Follow(m_leader);
    // m_strictFollower.SetInverted(TalonFXInvertType::CounterClockwise);

    // bucket rotators
    hardware::TalonFX lBcktMotor{30, "can0"};
    controls::DutyCycleOut lBcktPwrDutyCycle{0};
    controls::PositionDutyCycle lBcktPosDutyCycle{0 * 0_tr}; // absolute position to reach (in rotations)
    hardware::TalonFX rBcktMotor{40, "can0"};

    // vibration motors
    // TODO: 2 NEO 550s via SparkMAX

    bool has_goal{false};
    const int LOOP_RATE_HZ{60};
    std::shared_ptr<GoalHandleDig> dig_goal_handle;

    // subs to actuator position topics
    // should always be aligned so only 1 per pair of acts
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr linear  = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/link", 2, std::bind(&DigActionServer::dig_link_callback, this, std::placeholders::_1));

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rotational = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/bckt", 2, std::bind(&DigActionServer::dig_bckt_callback, this, std::placeholders::_1));

    float starting_act_pos{-987654}; // if this is negative 987654 then it means that we have not reseeded the starting volume for the run. Note that even the absolute value is an entirely unrealistic volume
    float current_link_pos{-987654};

    float starting_bckt_pos{-987654}; // if this is negative 987654 then it means that we have not reseeded the starting volume for the run. Note that even the absolute value is an entirely unrealistic volume
    float current_bckt_pos{-987654};

    /**
     * this gets us the sensor data for where our linkage actuators are at
     */
    void dig_link_callback(const std_msgs::msg::Float32 msg){
      RCLCPP_INFO(this->get_logger(), "DIG: /dig/link: %f", msg.data);
      if(abs(abs(starting_act_pos) - 987654) < 2){ // first pos
        starting_act_pos = msg.data;
        RCLCPP_INFO(this->get_logger(), "DIG: starting linkage actuator positions are %f", starting_act_pos);

      }
      else{
        current_link_pos = msg.data;
        RCLCPP_INFO(this->get_logger(), "DIG: current linkage actuator positions are %f", current_link_pos);

      }
    }

    /**
     * this gets us the sensor data for where our rotation motors are at
     */
    void dig_bckt_callback(const std_msgs::msg::Float32 msg){
      RCLCPP_INFO(this->get_logger(), "DIG: /dig/bckt: %f", msg.data);
      if(abs(abs(starting_bckt_pos) - 987654) < 2){ // first pos
        starting_bckt_pos = msg.data;
        RCLCPP_INFO(this->get_logger(), "DIG: starting rotation motor positions are %f", starting_bckt_pos);

      }
      else{
        current_bckt_pos = msg.data;
        RCLCPP_INFO(this->get_logger(), "DIG: current rotation motor positions are %f", current_bckt_pos);

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
      // RCLCPP_INFO(this->get_logger(), "DIG: Received goal request with order %f", goal->dig_goal);

      if(!has_goal){
        RCLCPP_INFO(this->get_logger(),"DIG: Accepted goal");
        has_goal = true;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      else{
        RCLCPP_INFO(this->get_logger(),"DIG: Rejected goal because one is still executing");

        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    /**
     *
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "DIG: Received request to cancel goal");

      // stop motion
      lLinkPwrDutyCycle.Output = 0;
      lBcktPwrDutyCycle.Output = 0;

      // set class vars
      dig_goal_handle = nullptr;
      has_goal = false;

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
        RCLCPP_INFO(this->get_logger(), "DIG: execute: autonomous control");
        execute_auton(goal_handle);

      } else if (!std::isnan(goal->dig_link_pos_goal) &&
                 !std::isnan(goal->dig_bckt_pos_goal)) {
        RCLCPP_INFO(this->get_logger(), "DIG: execute: position control");
        execute_pos(goal_handle);

      } else if (!std::isnan(goal->dig_link_pwr_goal) &&
                 !std::isnan(goal->dig_bckt_pwr_goal)) {
        RCLCPP_INFO(this->get_logger(), "DIG: execute: power control");
        execute_pwr(goal_handle);

      } else {
        RCLCPP_ERROR(this->get_logger(), "DIG: execute: Malformed request. Were request values populated?");
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
      RCLCPP_INFO(this->get_logger(), "DIG: execute_pwr: executing...");

      const auto goal = goal_handle->get_goal();
      float linkage_goal = goal->dig_link_pwr_goal;
      float bucket_goal  = goal->dig_bckt_pwr_goal;

      // check that goal is allowable (duty cycle takes [-1, 1])
      if (linkage_goal < -1 || linkage_goal > 1 ||
          bucket_goal  < -1 || bucket_goal  > 1)
      {
        RCLCPP_ERROR(this->get_logger(), "DIG: execute_pwr: Linkage and/or Bucket goal was out of bounds. Power goals should always be in [-1, 1]");
      }

      auto feedback = std::make_shared<Dig::Feedback>();
      auto result = std::make_shared<Dig::Result>();

      auto &linkPercentDone = feedback->percent_link_done;
      auto &bcktPercentDone = feedback->percent_bckt_done;
      ctre::phoenix::unmanaged::FeedEnable(pow(static_cast<float>(LOOP_RATE_HZ), -1));

      lLinkPwrDutyCycle.Output = linkage_goal;
      lBcktPwrDutyCycle.Output = bucket_goal ;

      lLinkMotor.SetControl(lLinkPwrDutyCycle);
      lBcktMotor.SetControl(lBcktPwrDutyCycle);

      linkPercentDone = 100;
      bcktPercentDone = 100;
      goal_handle->publish_feedback(feedback);

      if (rclcpp::ok())
      {
        result->est_dig_link_goal = linkage_goal;
        result->est_dig_bckt_goal = bucket_goal ;

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "DIG: execute_pwr: Goal succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "DIG: execute_pwr: Goal failed");
      }

      has_goal = false;
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
      RCLCPP_INFO(this->get_logger(), "DIG: execute_pos: executing...");

      rclcpp::Rate loop_rate(LOOP_RATE_HZ);
      const auto goal = goal_handle->get_goal();
      float linkage_goal = goal->dig_link_pos_goal;
      float bucket_goal  = goal->dig_bckt_pos_goal;

      // check that goal is allowable (duty cycle takes [-1, 1])
      if (linkage_goal < -1 || linkage_goal > 1 ||
          bucket_goal  < -1 || bucket_goal  > 1)
      {
        RCLCPP_ERROR(this->get_logger(), "DIG: execute_pos: Linkage and/or Bucket goal was out of bounds. Power goals should always be in [-1, 1]");
      }

      auto feedback = std::make_shared<Dig::Feedback>();
      auto result = std::make_shared<Dig::Result>();

      while (!APPROX(current_link_pos, linkage_goal) &&
             !APPROX(current_bckt_pos, bucket_goal))
      { // keep sending the request because CTRE's watchdog
        auto &linkPercentDone = feedback->percent_link_done;
        auto &bcktPercentDone = feedback->percent_bckt_done;
        ctre::phoenix::unmanaged::FeedEnable(pow(static_cast<float>(LOOP_RATE_HZ), -1));
        units::angle::turn_t linkage_rots {linkage_goal * 1_tr};
        units::angle::turn_t bucket_rots{bucket_goal * 1_tr};
        lLinkPosDutyCycle.Position = linkage_rots;
        lBcktPosDutyCycle.Position = bucket_rots ;

        lLinkMotor.SetControl(lLinkPosDutyCycle);
        lBcktMotor.SetControl(lBcktPosDutyCycle);

        linkPercentDone = (abs(linkage_goal) - abs(current_link_pos))/abs(linkage_goal) * 100;
        bcktPercentDone = (abs(bucket_goal ) - abs(current_bckt_pos))/abs(bucket_goal ) * 100;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
      }

      if (rclcpp::ok())
      {
        result->est_dig_link_goal = current_link_pos;
        result->est_dig_bckt_goal = current_bckt_pos;

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "DIG: execute_pos: Goal succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "DIG: execute_pos: Goal failed");
      }

      has_goal = false;
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

    }

  }; // class DigActionServer

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(dig_server::DigActionServer)
