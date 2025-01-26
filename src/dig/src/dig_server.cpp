#include "action_interfaces/action/dig.hpp"
#include <trigger_action/action/trigger_action.hpp>

#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"

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
    using DigPos = action_interfaces::action::Dig;
    using DigAuto = trigger_action::action::TriggerAction;
    using GoalHandleDig = rclcpp_action::ServerGoalHandle<Dig>;

    explicit DigActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("dig_action_server", options)
    {
      using namespace std::placeholders;

      this->pos_action_server_ = rclcpp_action::create_server<Dig>(
          this,
          "dig_pos_action",
          std::bind(&DigActionServer::handle_pos_goal, this, _1, _2),
          std::bind(&DigActionServer::handle_pos_cancel, this, _1),
          std::bind(&DigActionServer::handle_pos_accepted, this, _1));

      RCLCPP_INFO(this->get_logger(), "DIG: Position (manual) action server is ready");

      this->auto_action_server_ = rclcpp_action::create_server<Dig>(
          this,
          "dig_auto_action",
          std::bind(&DigActionServer::handle_auto_goal, this, _1, _2),
          std::bind(&DigActionServer::handle_auto_cancel, this, _1),
          std::bind(&DigActionServer::handle_auto_accepted, this, _1));

      RCLCPP_INFO(this->get_logger(), "DIG: Position (manual) action server is ready");

    }

  private:
    rclcpp_action::Server<Dig>::SharedPtr pos_action_server_;
    rclcpp_action::Server<Dig>::SharedPtr auto_action_server_;

    // linkage actuators
    hardware::TalonFX lActMotor{10, "can0"}; // canid (each motor), can interface (same for all)
    controls::DutyCycleOut lActDutyCycle{0};
    hardware::TalonFX rActMotor{20, "can0"}; // canid (each motor), can interface (same for all)
    rActMotor.Follow(lActMotor);
    // m_follower.Follow(m_leader);
    // m_follower.SetInverted(TalonFXInvertType::FollowMaster);
    // m_strictFollower.Follow(m_leader);
    // m_strictFollower.SetInverted(TalonFXInvertType::CounterClockwise);

    // bucket rotators
    hardware::TalonFX lRotMotor{30, "can0"}; // canid (each motor), can interface (same for all)
    controls::DutyCycleOut lRotDutyCycle{0};
    hardware::TalonFX rRotMotor{40, "can0"}; // canid (each motor), can interface (same for all)
    rRotMotor.Follow(rActMotor);

    // vibration motors
    // TODO: 2 NEO 550s via SparkMAX

    bool has_goal{false};
    int LOOP_RATE_HZ{60};
    std::shared_ptr<GoalHandleDig> dig_goal_handle;

    // subs to actuator position servers
    // rclcpp::Subscription<controls_msgs::msg::Dig>::SharedPtr pos_description = this->create_subscription<controls_msgs::msg::Dig>(
    //   "/dig", 2, std::bind(&DigActionServer::dig_pos_callback, this, std::placeholders::_1));

    float starting_act_pos{-987654}; // if this is negative 987654 then it means that we have not reseeded the starting volume for the run. Note that even the absolute value is an entirely unrealistic volume
    float current_act_pos{-987654};

    float starting_rot_pos{-987654}; // if this is negative 987654 then it means that we have not reseeded the starting volume for the run. Note that even the absolute value is an entirely unrealistic volume
    float current_rot_pos{-987654};

    // void dig_pos_callback(const controls_msgs::msg::Dig msg){
    //   RCLCPP_INFO(this->get_logger(), "DIG: I have recived %f", msg.data);
    //   if(abs(abs(starting_act_pos) - 987654) < 2){ // first pos
    //     starting_act_pos = msg.lins;
    //     starting_rot_pos = msg.rots;

    //     RCLCPP_INFO(this->get_logger(), "DIG: starting actuator position is now %f", starting_act_pos);
    //     RCLCPP_INFO(this->get_logger(), "DIG: starting rotation position is now %f", starting_rot_pos);

    //   }
    //   else{
    //     current_act_pos = msg.lins;
    //     current_rot_pos = msg.rots;

    //     RCLCPP_INFO(this->get_logger(), "DIG: current actuator position is now %f", starting_act_pos);
    //     RCLCPP_INFO(this->get_logger(), "DIG: current rotation position is now %f", starting_rot_pos);

    //   }
    // }

    /**************************************************************************
     * Pos dig handling                                                       *
     *                                                                        *
     *                                                                        *
     *************************************************************************/

    rclcpp_action::GoalResponse handle_pos_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Dig::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "DIG: Received goal request with order %f", goal->dig_goal);

      if(!has_goal){
        RCLCPP_INFO(this->get_logger(),"DIG: Accepted goal");
        has_goal = true;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      else{
        RCLCPP_INFO(this->get_logger(),"DIG: Rejected goal; one is still executing");

        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    rclcpp_action::CancelResponse handle_pos_cancel(
        const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "DIG: Received request to cancel goal");

      // stop motion
      lActDutyCycle.Output = 0;
      lRotDutyCycle.Output = 0;

      // set class vars
      dig_goal_handle = nullptr;
      has_goal = false;

      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_pos_accepted(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&DigActionServer::execute_pos, this, _1), goal_handle}.detach();
    }

    void execute_pos(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "DIG: Executing goal");

      rclcpp::Rate loop_rate(LOOP_RATE_HZ);
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<Dig::Feedback>();
      auto result = std::make_shared<Dig::Result>();

      while (!APPROX(current_act_pos, goal->dig_act_goal) &&
             !APPROX(current_rot_pos, goal->dig_rot_goal))
      {
        auto &actPercentDone = feedback->percent_act_done;
        auto &rotPercentDone = feedback->percent_rot_done;
        double speed = goal->dig_goal;
        auto result = std::make_shared<Dig::Result>();
        ctre::phoenix::unmanaged::FeedEnable(pow(static_cast<float>(loop_rate_hz), -1));

        lActDutyCycle.Output = speed;
        lRotDutyCycle.Output = speed;

        lActMotor.SetControl(lActDutyCycle);
        lRotMotor.SetControl(lRotDutyCycle);

        RCLCPP_INFO(this->get_logger(), "DIG: moving to position");
        actPercentDone = (abs(goal->dig_act_goal) - abs(current_act_pos))/abs(goal->dig_act_goal) * 100;
        rotPercentDone = (abs(goal->dig_rot_goal) - abs(current_rot_pos))/abs(goal->dig_rot_goal) * 100;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
      }

      has_goal = false;

      if (rclcpp::ok())
      {
        result->est_dig_act_goal = current_act_pos;
        result->est_dig_rot_goal = current_rot_pos;

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "DIG: Goal succeeded");
      }

    }


    /**************************************************************************
     * Autonomous dig handling                                                *
     *                                                                        *
     *                                                                        *
     *************************************************************************/

    rclcpp_action::GoalResponse handle_auto_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TriggerAction::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "DIG: Received goal request with order %f", goal->dig_goal);

      if(!has_goal){
        RCLCPP_INFO(this->get_logger(),"DIG: Accepted goal");
        has_goal = true;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      else{
        RCLCPP_INFO(this->get_logger(),"DIG: Rejected goal; one is still executing");

        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    rclcpp_action::CancelResponse handle_auto_cancel(
        const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "DIG: Received request to cancel goal");

      // stop motion
      lActDutyCycle.Output = 0;
      lRotDutyCycle.Output = 0;

      // set class vars
      dig_goal_handle = nullptr;
      has_goal = false;

      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_auto_accepted(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&DigActionServer::execute_auto, this, _1), goal_handle}.detach();
    }

    void execute_auto(const std::shared_ptr<GoalHandleDig> goal_handle)
    {

    }

  }; // class DigActionServer

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(dig_server::DigActionServer)
