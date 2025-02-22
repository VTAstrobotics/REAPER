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
#include "PIDController.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "../include/utils.h"

using namespace action_interfaces::action;
using namespace ctre::phoenix6;
using namespace std::placeholders;
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

      this->action_server_ = rclcpp_action::create_server<Dig>(
          this,
          "dig_action",
          std::bind(&DigActionServer::handle_goal, this, _1, _2),
          std::bind(&DigActionServer::handle_cancel, this, _1),
          std::bind(&DigActionServer::handle_accepted, this, _1));

      // TODO: change to logging severity to INFO
      RCLCPP_INFO(get_logger(), "Setting severity threshold to DEBUG");
      auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }

      // Linkage motor configuration
      // configs::TalonFXConfiguration link_configs{};
      configs::Slot0Configs linkPIDConfig{};
      // configs::Slot0Configs& linkPIDConfig = link_configs.Slot0;
      float K_u = 5.9, T_u = 0.04;
      linkPIDConfig.kP = 0.8 * K_u;
      linkPIDConfig.kI = 0; // 0; PD controller
      linkPIDConfig.kD = 0.1 * K_u * T_u;
      l_link_mtr_.GetConfigurator().Apply(linkPIDConfig);
      r_link_mtr_.GetConfigurator().Apply(linkPIDConfig);

      configs::CurrentLimitsConfigs linkLimConfig{};
      // linkLimConfig.SupplyCurrentLimit = 60;
      linkLimConfig.SupplyCurrentLimit = 10; // for testing
      linkLimConfig.SupplyCurrentLimitEnable = true;
      l_link_mtr_.GetConfigurator().Apply(linkLimConfig);
      r_link_mtr_.GetConfigurator().Apply(linkLimConfig);

      // auto& link_mm_configs = link_configs.MotionMagic;
      // link_mm_configs.MotionMagicCruiseVelocity = 3;
      // link_mm_configs.MotionMagicAcceleration = 20;
      // l_link_mtr_.GetConfigurator().Apply(link_configs);
      // r_link_mtr_.GetConfigurator().Apply(link_configs);

      // enable brake mode TODO: test this
      configs::MotorOutputConfigs link_mtr_configs{};
      link_mtr_configs.NeutralMode = signals::NeutralModeValue::Brake;
      link_mtr_configs.PeakForwardDutyCycle = 0.1;
      link_mtr_configs.PeakReverseDutyCycle = -0.1;
      l_link_mtr_.GetConfigurator().Apply(link_mtr_configs);
      r_link_mtr_.GetConfigurator().Apply(link_mtr_configs);

      // set right motors to follow left motors
      r_link_mtr_.SetControl(controls::Follower{l_link_mtr_.GetDeviceID(), true}); // true because they are mounted inverted

      // Bucket motor configuration
      configs::Slot0Configs bcktPIDConfig{};
      K_u = 3.9, T_u = 0.04; // TODO: tune these values for the bucket.
      bcktPIDConfig.kP = 0.8 * K_u;
      bcktPIDConfig.kI = 0; // 0; PD controller
      bcktPIDConfig.kD = 0.1 * K_u * T_u;
      l_bckt_mtr_.GetConfigurator().Apply(bcktPIDConfig);

      configs::CurrentLimitsConfigs bcktLimConfig{};
      // bcktLimConfig.SupplyCurrentLimit = 40;
      bcktLimConfig.SupplyCurrentLimit = 10;
      bcktLimConfig.SupplyCurrentLimitEnable = true;
      l_bckt_mtr_.GetConfigurator().Apply(bcktLimConfig);
      r_bckt_mtr_.GetConfigurator().Apply(bcktLimConfig);

      // enable brake mode TODO: test this
      configs::MotorOutputConfigs bckt_mtr_configs{};
      bckt_mtr_configs.NeutralMode = signals::NeutralModeValue::Brake;
      l_bckt_mtr_.GetConfigurator().Apply(bckt_mtr_configs);
      r_bckt_mtr_.GetConfigurator().Apply(bckt_mtr_configs);

      // set right motors to follow left motors
      r_bckt_mtr_.SetControl(controls::Follower{l_bckt_mtr_.GetDeviceID(), false});

      // TODO finish this?
      // controls::PositionVoltage linkPV = controls::PositionVoltage{0_tr}.WithSlot(0);

      // Left vibration motor (neo550) configuration
      // l_vib_mtr_.SetIdleMode(IdleMode::kCoast);
      // l_vib_mtr_.SetMotorType(MotorType::kBrushless);
      // l_vib_mtr_.SetSmartCurrentFreeLimit(10.0);
      // l_vib_mtr_.SetSmartCurrentStallLimit(10.0);
      // l_vib_mtr_.BurnFlash();

      // Right vibration motor (neo550) configuration
      // r_vib_mtr_.SetIdleMode(IdleMode::kCoast);
      // r_vib_mtr_.SetMotorType(MotorType::kBrushless);
      // r_vib_mtr_.SetSmartCurrentFreeLimit(10.0);
      // r_vib_mtr_.SetSmartCurrentStallLimit(10.0);
      // r_vib_mtr_.BurnFlash();

      // Hardstop linear actuator configuration
      // hstp_mtr_.SetIdleMode(IdleMode::kBrake);
      // hstp_mtr_.SetMotorType(MotorType::kBrushed);
      // hstp_mtr_.SetSmartCurrentFreeLimit(10.0);
      // hstp_mtr_.SetSmartCurrentStallLimit(10.0);

      // PIDController hstp_pid(hstp_mtr_);
      // K_u = 3.9, T_u = 0.04; // TODO: tune these values for the hardstop.
      // hstp_pid.SetP(0, 0.8 * K_u);
      // hstp_pid.SetI(0, 0.);
      // hstp_pid.SetD(0, 0.1 * K_u * T_u); // 0; PD controller

      // // Configure smart motion settings for velocity control
      // hstp_pid.SetSmartMotionMaxVelocity(0, 100.);  // Max velocity in RPM
      // hstp_pid.SetSmartMotionMaxAccel(0, 10.);     // Max acceleration in RPM/s

      // hstp_mtr_.BurnFlash();
    }

  private:
    rclcpp_action::Server<Dig>::SharedPtr action_server_;

    // linkage actuators
    hardware::TalonFX l_link_mtr_{20, "can0"}; // canid (each motor), can interface (same for all)
    controls::DutyCycleOut l_link_pwr_duty_cycle_{0}; // [-1, 1]
    controls::PositionDutyCycle l_link_pos_duty_cycle_{0 * 0_tr}; // absolute position to reach (in rotations)
    hardware::TalonFX r_link_mtr_{23, "can0"};

    // bucket rotators
    hardware::TalonFX l_bckt_mtr_{21, "can0"};
    controls::DutyCycleOut l_bckt_pwr_duty_cycle_{0};
    controls::PositionDutyCycle l_bckt_pos_duty_cycle_{0 * 0_tr}; // absolute position to reach (in rotations)
    hardware::TalonFX r_bckt_mtr_{24, "can0"};

    // hardstop linear actuator
    // SparkMax hstp_mtr_{"can0", 26};

    // vibration motors
    // SparkMax l_vib_mtr_{"can0", 22};
    // SparkMax r_vib_mtr_{"can0", 25};

    bool has_goal_{false};
    const int LOOP_RATE_HZ_{50};
    const float HSTP_VEL_{2.9}; // in/s. estimate. TODO: remove when sensor
    /* ridiculous number and recognizable.
     * CORRESPONDS TO THE ACTION DEFINITION (.action)
     * DO NOT CHANGE WITHOUT CHANGING THE ACTION DEFINITION! */
    const float DEFAULT_VAL_{-987654.321};
    const float LINK_GEAR_RATIO_{100}; // 100:1
    const float BCKT_GEAR_RATIO_{80}; // 80:1
    double goal_received_time_{-1}; // in seconds

    // subs to actuator position topics
    // should always be aligned so only 1 per pair of acts
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr link_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/link", 2, std::bind(&DigActionServer::link_cb, this, std::placeholders::_1));

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bckt_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/bckt", 2, std::bind(&DigActionServer::bckt_cb, this, std::placeholders::_1));

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hstp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/hstp", 2, std::bind(&DigActionServer::hstp_cb, this, std::placeholders::_1));

    // if this is DEFAULT_VAL_ then it means that we have not reseeded the starting pos for the run.
    float starting_link_pos_{DEFAULT_VAL_};
    float current_link_pos_{DEFAULT_VAL_};

    // if this is DEFAULT_VAL_ then it means that we have not reseeded the starting pos for the run.
    float starting_bckt_pos_{DEFAULT_VAL_};
    float current_bckt_pos_{DEFAULT_VAL_};

    // if this is DEFAULT_VAL_ then it means that we have not reseeded the starting pos for the run.
    float starting_hstp_pos_{DEFAULT_VAL_};
    float current_hstp_pos_{0}; // TODO this is temporary ok

    // position limits
    const float LINK_MIN_POS_{-abs(DEFAULT_VAL_) + 1}; // TODO replace temp value
    const float LINK_MAX_POS_{abs(DEFAULT_VAL_) - 1}; // TODO replace temp value
    const float BCKT_MIN_POS_{-abs(DEFAULT_VAL_) + 1}; // TODO replace temp value
    const float BCKT_MAX_POS_{abs(DEFAULT_VAL_) - 1}; // TODO replace temp value
    const float HSTP_MIN_POS_{0};
    const float HSTP_MAX_POS_{3}; // TODO replace with absolute pot max (for 3 in)

    // lookup table for auto dig
// time (s),actuator angle (rots),bucket angle (rots), linact hardstop (encoder [0,4096]),vibration (duty cycle [-1,1])
    float LOOKUP_TB_[7][5] = {
      0,0,0,0,0,
      1,1,1,0.5,0.2,
      2,2,2,1,0.4,
      3,3,3,1.5,0.6,
      4,4,4,2,0.8,
      5,5,5,2.5,0.6,
      6,5,5,3,0,
    };

    /**
     * this gets us the sensor data for where our linkage actuators are at
     */
    void link_cb(const std_msgs::msg::Float32 msg){
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
    void bckt_cb(const std_msgs::msg::Float32 msg){
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

    /**
     * this gets us the sensor data for where our hardstop is at
     */
    void hstp_cb(const std_msgs::msg::Float32 msg){
      RCLCPP_INFO(this->get_logger(), "/dig/hstp: %f", msg.data);
      if(abs(abs(starting_hstp_pos_) - 987654) < 2){ // first pos
        starting_hstp_pos_ = msg.data;
        RCLCPP_INFO(this->get_logger(), "starting hardstop position is %f", starting_hstp_pos_);

      }
      else{
        current_hstp_pos_ = msg.data;
        RCLCPP_INFO(this->get_logger(), "current hardstop position is %f", current_hstp_pos_);

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
      // RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->dig_goal); // TODO decide what to do with this
      (void)uuid, (void)goal; // for unused warning

      goal_received_time_ = this->now().seconds();

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
      (void) goal_handle; // for unused warning

      // stop motion
      link_pwr(0);
      bckt_pwr(0);
      vibr_pwr(0);
      hstp_pwr(0);

      // set class vars
      has_goal_ = false;

      RCLCPP_INFO(this->get_logger(), "Goal canceled");
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
      auto feedback = std::make_shared<Dig::Feedback>();
      auto result = std::make_shared<Dig::Result>();
      std::vector<std::thread> threads;

      if (goal->auton) { // full auto
        RCLCPP_DEBUG(this->get_logger(), "execute: autonomous control");
        execute_auton(goal_handle, feedback, result); // note we do not need a new thread

      } else { // at least some manual control
        // linkage
        if (!APPROX(goal->link_pos_goal, DEFAULT_VAL_)) {
          RCLCPP_DEBUG(this->get_logger(), "execute: linkage position control");
          threads.emplace_back(std::thread{std::bind(&DigActionServer::exe_link_pos, this, _1, _2, _3), goal_handle, feedback, result});

        } else {
          RCLCPP_DEBUG(this->get_logger(), "execute: linkage power control");
          threads.emplace_back(std::thread{std::bind(&DigActionServer::exe_link_pwr, this, _1, _2, _3), goal_handle, feedback, result});

        }

        // bucket
        if (!APPROX(goal->bckt_pos_goal, DEFAULT_VAL_)) {
          RCLCPP_DEBUG(this->get_logger(), "execute: bucket position control");
          threads.emplace_back(std::thread{std::bind(&DigActionServer::exe_bckt_pos, this, _1, _2, _3), goal_handle, feedback, result});

        } else {
          RCLCPP_DEBUG(this->get_logger(), "execute: bucket power control");
          threads.emplace_back(std::thread{std::bind(&DigActionServer::exe_bckt_pwr, this, _1, _2, _3), goal_handle, feedback, result});

        }

        // hardstop
        if (!APPROX(goal->hstp_pos_goal, DEFAULT_VAL_)) {
          RCLCPP_DEBUG(this->get_logger(), "execute: hardstop position control");
          threads.emplace_back(std::thread{std::bind(&DigActionServer::exe_hstp_pos, this, _1, _2, _3), goal_handle, feedback, result});

        } else {
          RCLCPP_DEBUG(this->get_logger(), "execute: hardstop power control");
          threads.emplace_back(std::thread{std::bind(&DigActionServer::exe_hstp_pwr, this, _1, _2, _3), goal_handle, feedback, result});

        }

        // vibration
        exe_vibr_pwr(goal_handle, feedback, result); // note we do not need a new thread

        // wait for all goals to finish
        for (auto& thread : threads) {
          RCLCPP_DEBUG(this->get_logger(), "Waiting for thread to finish...");
          thread.join();
        }

      } // end if/else for autonomy

      // handle goal completion
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->succeed(result);
      }

      has_goal_ = false;
    }

    /**************************************************************************
     * Power control handling                                                 *
     *                                                                        *
     *                                                                        *
     *************************************************************************/

    /**
     * sets the linkage motors to run with a specific duty cycle in [-1, 1]
     * @param pwr the duty cycle for the motors to run at
     */
    void link_pwr(double pwr) {
      if (!pwr_in_bounds(pwr))
      {
        RCLCPP_ERROR(this->get_logger(), "link_pwr: %f was out of bounds. Power goals should always be in [-1, 1]", pwr);
        pwr = 0;
      }

      l_link_pwr_duty_cycle_.Output = pwr;
      l_link_mtr_.SetControl(l_link_pwr_duty_cycle_);
    }

    /**
     * sets the bucket motors to run with a specific duty cycle in [-1, 1]
     * @param pwr the duty cycle for the motors to run at
     */
    void bckt_pwr(double pwr) {
      if (!pwr_in_bounds(pwr))
      {
        RCLCPP_ERROR(this->get_logger(), "bckt_pwr: %f was out of bounds. Power goals should always be in [-1, 1]", pwr);
        pwr = 0;
      }

      l_bckt_pwr_duty_cycle_.Output = pwr;
      l_bckt_mtr_.SetControl(l_bckt_pwr_duty_cycle_);
    }

    /**
     * sets the vibration motors to run with a specific duty cycle in [-1, 1]
     * @param pwr the duty cycle for the motors to run at
     */
    void vibr_pwr(double pwr){
      if (!pwr_in_bounds(pwr))
      {
        RCLCPP_ERROR(this->get_logger(), "vibr_pwr: %lf was out of bounds. Power goals should always be in [-1, 1]", pwr);
        pwr = 0;
      }

      // l_vib_mtr_.Heartbeat();
      // r_vib_mtr_.Heartbeat();

      // l_vib_mtr_.SetDutyCycle(pwr);
      // r_vib_mtr_.SetDutyCycle(pwr);
    }

    /**
     * sets the hardstop motor to run with a specific duty cycle in [-1, 1]
     * @param pwr the duty cycle for the motor to run at
     */
    void hstp_pwr(double pwr){
      if (!pwr_in_bounds(pwr))
      {
        RCLCPP_ERROR(this->get_logger(), "hstp_pwr: %lf was out of bounds. Power goals should always be in [-1, 1]", pwr);
        pwr = 0;
      }

      // hstp_mtr_.Heartbeat();
      // hstp_mtr_.SetDutyCycle(pwr);
    }

    void execute_pwr(const std::shared_ptr<GoalHandleDig> goal_handle,
      std::shared_ptr<Dig::Feedback> feedback,
      std::shared_ptr<Dig::Result> result, double goal_val,
      float& percent_done, std::function<void(double)> pwr_func,
      float& est_goal, const char* print_prefix)
    {
      (void) result; // for unused warning
      if (goal_handle->is_canceling()) { return; }

      RCLCPP_DEBUG_ONCE(this->get_logger(), "execute_pwr: Loop rate %f ms", 1000 * (1.0/(double)(LOOP_RATE_HZ_))); //this is the correct math with correct units :)
      ctre::phoenix::unmanaged::FeedEnable(1000 * (1.0/(double)(LOOP_RATE_HZ_)));

RCLCPP_INFO(this->get_logger(), "cur %lf", (double)l_link_mtr_.GetPosition().GetValue());
      pwr_func(goal_val);

      percent_done = 100;
      goal_handle->publish_feedback(feedback);

      goal_done_helper(est_goal, goal_val, this->get_logger(), print_prefix);
    }

    /**
     * runs the dig linkage motors to a duty cycle goal
     * @param goal_handle pointer to the goal
     */
    void exe_link_pwr(const std::shared_ptr<GoalHandleDig> goal_handle,
      std::shared_ptr<Dig::Feedback> feedback,
      std::shared_ptr<Dig::Result> result) {
      double linkage_goal = goal_handle->get_goal()->link_pwr_goal;
      float& link_percent_done = feedback->percent_link_done;

      execute_pwr(
        goal_handle,
        feedback,
        result,
        linkage_goal,
        link_percent_done,
        std::bind(&DigActionServer::link_pwr, this, _1),
        result->est_link_goal,
        __func__
      );
    }

    /**
     * runs the dig linkage motors to a duty cycle goal
     * @param goal_handle pointer to the goal
     */
    void exe_bckt_pwr(const std::shared_ptr<GoalHandleDig> goal_handle,
      std::shared_ptr<Dig::Feedback> feedback,
      std::shared_ptr<Dig::Result> result) {
      double bucket_goal = goal_handle->get_goal()->bckt_pwr_goal;
      float& bckt_percent_done = feedback->percent_bckt_done;
      RCLCPP_INFO(this->get_logger(), "buck goal = %f", bucket_goal);

      execute_pwr(
        goal_handle,
        feedback,
        result,
        bucket_goal,
        bckt_percent_done,
        std::bind(&DigActionServer::bckt_pwr, this, _1),
        result->est_bckt_goal,
        __func__
      );
    }

    /**
     * runs the dig linkage motors to a duty cycle goal
     * @param goal_handle pointer to the goal
     */
    void exe_vibr_pwr(const std::shared_ptr<GoalHandleDig> goal_handle,
      std::shared_ptr<Dig::Feedback> feedback,
      std::shared_ptr<Dig::Result> result) {
      double vibration_goal = goal_handle->get_goal()->vibr_pwr_goal;
      float& vibr_percent_done = feedback->percent_vibr_done;

      execute_pwr(
        goal_handle,
        feedback,
        result,
        vibration_goal,
        vibr_percent_done,
        std::bind(&DigActionServer::vibr_pwr, this, _1),
        result->est_vibr_goal,
        __func__
      );
    }

    /**
     * runs the dig linkage motors to a duty cycle goal
     * @param goal_handle pointer to the goal
     */
    void exe_hstp_pwr(const std::shared_ptr<GoalHandleDig> goal_handle,
      std::shared_ptr<Dig::Feedback> feedback,
      std::shared_ptr<Dig::Result> result) {
      double hardstop_goal = goal_handle->get_goal()->hstp_pwr_goal;
      float& hstp_percent_done = feedback->percent_hstp_done;

      execute_pwr(
        goal_handle,
        feedback,
        result,
        hardstop_goal,
        hstp_percent_done,
        std::bind(&DigActionServer::hstp_pwr, this, _1),
        result->est_hstp_goal,
        __func__
      );
    }

    /**************************************************************************
     * Position control handling                                              *
     *                                                                        *
     *                                                                        *
     *************************************************************************/

    /**
     * Checks if the requested position is in the bounds
     * @param pos is the requested position
     * @param min is the minimum boundary
     * @param max is the maximum boundary
     * @return true if in bounds, fales if out of bounds
     */
    bool pos_in_bounds(double pos, float min, float max) {
      return !(pos < min || pos > max);
    }

    /**
     * Checks if the requested position is in the linkage bounds
     * @param pos is the requested position to drive the linkage
     * @return true if in bounds, fales if out of bounds
     */
    bool linkage_in_bounds(double pos) {
      if (pos_in_bounds(pos, LINK_MIN_POS_, LINK_MAX_POS_)) {
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "linkage_in_bounds: Linkage goal was out of bounds. Linkage goal was %f but should be in [%f, %f]", pos, LINK_MIN_POS_, LINK_MAX_POS_);
        return false;
      }
    }

    /**
     * Checks if the requested position is in the bucket bounds
     * @param pos is the requested position to drive the bucket
     * @return true if in bounds, fales if out of bounds
     */
    bool bucket_in_bounds(double pos) {
      if (pos_in_bounds(pos, BCKT_MIN_POS_, BCKT_MAX_POS_)) {
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "bucket_in_bounds: Bucket goal was out of bounds. Bucket goal was %f but should be in [%f, %f]", pos, BCKT_MIN_POS_, BCKT_MAX_POS_);
        return false;
      }
    }

    /**
     * Checks if the requested position is in the hardstop bounds
     * @param pos is the requested position to drive the hardstop
     * @return true if in bounds, fales if out of bounds
     */
    bool hardstop_in_bounds(double pos) {
      if (pos_in_bounds(pos, HSTP_MIN_POS_, HSTP_MAX_POS_)) {
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "hardstop_in_bounds: Hardstop goal was out of bounds. Hardstop goal was %f but should be in [%f, %f]", pos, HSTP_MIN_POS_, HSTP_MAX_POS_);
        return false;
      }
    }

    /**
     * sets the linkage motors to go to a position within its bounds
     * @param pos the position for the linkage to go to
     */
    void link_pos(double pos, double vel = 1) {
      if (!linkage_in_bounds(pos)) { return; }
      (void)vel; // for unused warning

      units::angle::turn_t angle{pos * 1_tr};
      units::angular_velocity::turns_per_second_t speed{vel};

      l_link_pos_duty_cycle_.Velocity = speed; // rotations per sec
      l_link_pos_duty_cycle_.Position = angle;
      // controls::MotionMagicVoltage link_req{0_tr};
      // l_link_mtr_.SetControl(link_req);
      l_link_mtr_.SetControl(l_link_pos_duty_cycle_);

      current_link_pos_ = (double)l_link_mtr_.GetPosition().GetValue();
    }

    /**
     * sets the linkage motors to go to a position within its bounds
     * @param pos the position for the linkage to go to
     */
    void bckt_pos(double pos, double vel = 1) {
      if (!bucket_in_bounds(pos)) { return; }
      (void)vel; // for unused warning

      units::angle::turn_t angle{pos * 1_tr};
      units::angular_velocity::turns_per_second_t speed{vel};

      l_link_pos_duty_cycle_.Velocity = speed; // rotations per sec
      l_bckt_pos_duty_cycle_.Position = angle;
      l_bckt_mtr_.SetControl(l_bckt_pos_duty_cycle_);

      current_bckt_pos_ = (double)l_bckt_mtr_.GetPosition().GetValue();
    }

    /**
     * sets the linkage motors to go to a position within its bounds
     * @param pos the position for the linkage to go to
     */
    void hstp_pos(double pos, double vel = 1) {
      if (!hardstop_in_bounds(pos)) { return; }
      (void)vel; // for unused warning

      // temporarily use power/time
      // 3 in/s unloaded, 2.5 full load. maybe we can assume like 2.9 and tune?
      float hstp_duty_cycle = (pos - current_hstp_pos_) > 0 ? 1 : -1;
      RCLCPP_DEBUG(this->get_logger(), "hstp_pos: hstp_duty_cycle = %f", hstp_duty_cycle);
      hstp_pwr(hstp_duty_cycle);

      // TODO remove this when we get hstop sensor !
      // update estimate pos temporary power time estimate
      // duty cycle drives the direction
      // (duration extending) x (current_power) x (max_speed)
      float val = 0.05 * (this->now().seconds() - goal_received_time_);
      current_hstp_pos_ += hstp_duty_cycle * HSTP_VEL_ / (float)LOOP_RATE_HZ_ / val;
      RCLCPP_DEBUG(this->get_logger(), "hstp_pos: current_hstp_pos_ = %f", current_hstp_pos_);
    }

    /**
     * returns true if we are done moving to a position, false if we need to keep going
     * this means it will return true if the request is out of bounds!
     * @param current_pos is the current position
     * @param goal is the goal position
     * @param min is the minimum position
     * @param max is the maximum position
     * @return true if we have reached the position OR the goal is out of bounds (so we aren't trying to go anyway)
     */
    bool reached_pos(double current_pos, double goal, float min, float max) {
      return !pos_in_bounds(goal, min, max) || APPROX(current_pos, goal);
    }

    void execute_pos(const std::shared_ptr<GoalHandleDig> goal_handle,
      std::shared_ptr<Dig::Feedback> feedback,
      std::shared_ptr<Dig::Result> result, double goal_val, double vel,
      float& current_pos, const double MIN_POS, const double MAX_POS,
      float& percent_done, std::function<void(double, double)> pos_func,
      float& est_goal, const char* print_prefix)
    {
      (void) result; // for unused warning
      rclcpp::Rate loop_rate(LOOP_RATE_HZ_);

      while (!reached_pos(current_pos, goal_val, MIN_POS, MAX_POS))
      {
        RCLCPP_INFO(this->get_logger(), "cur %f, goal %f, min %f, max %f", current_pos, goal_val, MIN_POS, MAX_POS);
        if (goal_handle->is_canceling()) { return; }

        RCLCPP_DEBUG_ONCE(this->get_logger(), "execute_pos: Loop rate %f ms", 1000 * (1.0/(double)(LOOP_RATE_HZ_))); //this is the correct math with correct units :)
        ctre::phoenix::unmanaged::FeedEnable(1000 * (1.0/(double)(LOOP_RATE_HZ_)));

        pos_func(goal_val, vel);

        percent_done = (abs(goal_val) - abs(current_pos))/abs(goal_val) * 100;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
      }

      goal_done_helper(est_goal, goal_val, this->get_logger(), print_prefix);
    }

    /**
     * runs the dig linkage motors to a duty cycle goal
     * @param goal_handle pointer to the goal
     */
    void exe_link_pos(const std::shared_ptr<GoalHandleDig> goal_handle,
      std::shared_ptr<Dig::Feedback> feedback,
      std::shared_ptr<Dig::Result> result) {
      double linkage_goal = goal_handle->get_goal()->link_pos_goal;
      float& link_percent_done = feedback->percent_link_done;

      // TODO: need to factor in the absolute encoders (and any other sensor data) from the callback above
      current_link_pos_ = (double)l_link_mtr_.GetPosition().GetValue();

      execute_pos(
        goal_handle,
        feedback,
        result,
        linkage_goal,
        1, // vel
        current_link_pos_,
        LINK_MIN_POS_,
        LINK_MAX_POS_,
        link_percent_done,
        std::bind(&DigActionServer::link_pos, this, _1, _2),
        result->est_link_goal,
        __func__
      );
    }

    /**
     * runs the dig linkage motors to a duty cycle goal
     * @param goal_handle pointer to the goal
     */
    void exe_bckt_pos(const std::shared_ptr<GoalHandleDig> goal_handle,
      std::shared_ptr<Dig::Feedback> feedback,
      std::shared_ptr<Dig::Result> result) {
      double bucket_goal = goal_handle->get_goal()->bckt_pos_goal;
      float& bckt_percent_done = feedback->percent_bckt_done;

      // TODO: need to factor in the absolute encoders (and any other sensor data) from the callback above
      current_bckt_pos_ = (double)l_bckt_mtr_.GetPosition().GetValue();

      execute_pos(
        goal_handle,
        feedback,
        result,
        bucket_goal,
        1, // vel
        current_bckt_pos_,
        BCKT_MIN_POS_,
        BCKT_MAX_POS_,
        bckt_percent_done,
        std::bind(&DigActionServer::bckt_pos, this, _1, _2),
        result->est_bckt_goal,
        __func__
      );
    }

    /**
     * runs the dig linkage motors to a duty cycle goal
     * @param goal_handle pointer to the goal
     */
    void exe_hstp_pos(const std::shared_ptr<GoalHandleDig> goal_handle,
      std::shared_ptr<Dig::Feedback> feedback,
      std::shared_ptr<Dig::Result> result) {
      double hardstop_goal = goal_handle->get_goal()->hstp_pos_goal;
      float& hstp_percent_done = feedback->percent_hstp_done;

      execute_pos(
        goal_handle,
        feedback,
        result,
        hardstop_goal,
        1, // vel
        current_hstp_pos_,
        HSTP_MIN_POS_,
        HSTP_MAX_POS_,
        hstp_percent_done,
        std::bind(&DigActionServer::hstp_pos, this, _1, _2),
        result->est_hstp_goal,
        __func__
      );
    }

    /**************************************************************************
     * Autonomous dig handling                                                *
     *                                                                        *
     *                                                                        *
     *************************************************************************/

    /**
     * autonomously moves the dig actuators to scoop
     * @param goal_handle pointer to the goal
     */
    void execute_auton(const std::shared_ptr<GoalHandleDig> goal_handle,
      const std::shared_ptr<Dig::Feedback> feedback,
      const std::shared_ptr<Dig::Result> result)
    {
      RCLCPP_DEBUG(this->get_logger(), "execute_auton: executing...");
      rclcpp::Rate loop_rate(LOOP_RATE_HZ_);

      auto &link_percent_done = feedback->percent_link_done;
      auto &bckt_percent_done = feedback->percent_bckt_done;
      auto &hstp_percent_done = feedback->percent_hstp_done;
      auto &vibr_percent_done = feedback->percent_vibr_done;

      // TODO: need to factor in the absolute encoders (and any other sensor data) from the callback above
      current_link_pos_ = (double)l_link_mtr_.GetPosition().GetValue();
      current_bckt_pos_ = (double)l_bckt_mtr_.GetPosition().GetValue();

// time (s),linkage angle (rots),bucket angle (rots), linact hardstop (encoder [0,4096]),vibration (duty cycle [-1,1])
      for (size_t i = 0; i < sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0]); i++)
      {
        RCLCPP_DEBUG(this->get_logger(), "execute_auton: i=%ld", i);

        // get the starting time for this iteration of the loop
        double next_goal_time = this->now().seconds();

        if (i == sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0]) - 1)
        {
          // if it's the last iteration, we can't look-ahead, so assume some constant length of time
          next_goal_time += 1; // TODO change this??
        } else {
          next_goal_time += (LOOKUP_TB_[i+1][0] - LOOKUP_TB_[i][0]);
        }

        RCLCPP_DEBUG(this->get_logger(), "now = %f, nex goal = %f", this->now().seconds(), next_goal_time);

        for (size_t j = 0; j < sizeof(LOOKUP_TB_[0])/sizeof(LOOKUP_TB_[0][0]); j++) {
          RCLCPP_DEBUG(this->get_logger(), "%f, ", LOOKUP_TB_[i][j]);
        }

        while (this->now().seconds() < next_goal_time)
        {
          if (goal_handle->is_canceling()) { return; }

          // linkage, bucket, and hardstop to a set position
          link_pos(LOOKUP_TB_[i][1]);
          bckt_pos(LOOKUP_TB_[i][2]);
          hstp_pos(LOOKUP_TB_[i][3]);

          // vibration motors duty cycle
          vibr_pwr(LOOKUP_TB_[i][4]);

          link_percent_done = (i/sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0])) * 100;
          bckt_percent_done = (i/sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0])) * 100;
          hstp_percent_done = (i/sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0])) * 100;
          vibr_percent_done = (i/sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0])) * 100;
          goal_handle->publish_feedback(feedback);

          loop_rate.sleep();
        }

      }

      if (rclcpp::ok())
      {
        result->est_link_goal = current_link_pos_;
        result->est_bckt_goal = current_bckt_pos_;
        result->est_hstp_goal = current_hstp_pos_;
        result->est_vibr_goal = 0;

        // goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "execute_auton: Goal succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "execute_auton: Goal failed");
      }

      // has_goal_ = false;
    }

  }; // class DigActionServer

} // namespace dig_server

RCLCPP_COMPONENTS_REGISTER_NODE(dig_server::DigActionServer)
