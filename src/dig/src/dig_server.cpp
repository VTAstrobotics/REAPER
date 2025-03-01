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

      // TODO: change to logging severity to INFO
      RCLCPP_INFO(get_logger(), "Setting severity threshold to DEBUG");
      auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }

      // Linkage motor configuration
      configs::Slot0Configs linkPIDConfig{};
      float K_u = 3.9, T_u = 0.04;
      linkPIDConfig.kP = 0.8 * K_u;
      linkPIDConfig.kI = 0; // 0; PD controller
      linkPIDConfig.kD = 0.1 * K_u * T_u;
      l_link_mtr_.GetConfigurator().Apply(linkPIDConfig);

      configs::CurrentLimitsConfigs linkLimConfig{};
      linkLimConfig.SupplyCurrentLimit = 60;
      linkLimConfig.SupplyCurrentLimitEnable = true;
      l_link_mtr_.GetConfigurator().Apply(linkLimConfig);
      r_link_mtr_.GetConfigurator().Apply(linkLimConfig);

      // enable brake mode
      l_link_pwr_duty_cycle_.OverrideBrakeDurNeutral = true;
      l_link_pos_duty_cycle_.OverrideBrakeDurNeutral = true;

      // enable brake mode
      controls::StaticBrake static_brake;
      // l_link_mtr_.SetControl(static_brake);

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
      bcktLimConfig.SupplyCurrentLimit = 40;
      bcktLimConfig.SupplyCurrentLimitEnable = true;
      l_bckt_mtr_.GetConfigurator().Apply(bcktLimConfig);
      r_bckt_mtr_.GetConfigurator().Apply(bcktLimConfig);

      // enable brake mode
      // l_bckt_mtr_.SetControl(static_brake);

      // set right motors to follow left motors
      r_bckt_mtr_.SetControl(controls::Follower{l_bckt_mtr_.GetDeviceID(), false});

      // TODO finish this?
      // controls::PositionVoltage linkPV = controls::PositionVoltage{0_tr}.WithSlot(0);

      // Left vibration motor (neo550) configuration
      l_vib_mtr_.SetIdleMode(IdleMode::kCoast);
      l_vib_mtr_.SetMotorType(MotorType::kBrushless);
      l_vib_mtr_.SetSmartCurrentFreeLimit(10.0);
      l_vib_mtr_.SetSmartCurrentStallLimit(10.0);
      l_vib_mtr_.BurnFlash();

      // Right vibration motor (neo550) configuration
      r_vib_mtr_.SetIdleMode(IdleMode::kCoast);
      r_vib_mtr_.SetMotorType(MotorType::kBrushless);
      r_vib_mtr_.SetSmartCurrentFreeLimit(10.0);
      r_vib_mtr_.SetSmartCurrentStallLimit(10.0);
      r_vib_mtr_.BurnFlash();

      // Hardstop linear actuator configuration
      hstp_mtr_.SetIdleMode(IdleMode::kBrake);
      hstp_mtr_.SetMotorType(MotorType::kBrushed);
      hstp_mtr_.SetSmartCurrentFreeLimit(10.0);
      hstp_mtr_.SetSmartCurrentStallLimit(10.0);

      PIDController hstp_pid(hstp_mtr_);
      K_u = 3.9, T_u = 0.04; // TODO: tune these values for the hardstop.
      hstp_pid.SetP(0, 0.8 * K_u);
      hstp_pid.SetI(0, 0.);
      hstp_pid.SetD(0, 0.1 * K_u * T_u); // 0; PD controller

      // Configure smart motion settings for velocity control
      hstp_pid.SetSmartMotionMaxVelocity(0, 100.);  // Max velocity in RPM
      hstp_pid.SetSmartMotionMaxAccel(0, 10.);     // Max acceleration in RPM/s

      hstp_mtr_.BurnFlash();

      RCLCPP_DEBUG(this->get_logger(), "Ready for action");
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
    SparkMax hstp_mtr_{"can0", 26};

    // vibration motors
    SparkMax l_vib_mtr_{"can0", 22};
    SparkMax r_vib_mtr_{"can0", 25};

    bool has_goal_{false};
    const int LOOP_RATE_HZ_{50};
    std::shared_ptr<GoalHandleDig> dig_goal_handle_;
    const float HSTP_VEL_{2.9}; // in/s. estimate. TODO: remove when sensor

    // subs to actuator position topics
    // should always be aligned so only 1 per pair of acts
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr link_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/link", 2, std::bind(&DigActionServer::dig_link_cb, this, std::placeholders::_1));

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bckt_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/bckt", 2, std::bind(&DigActionServer::dig_bckt_cb, this, std::placeholders::_1));

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hstp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/dig/hstp", 2, std::bind(&DigActionServer::dig_hstp_cb, this, std::placeholders::_1));

    // if this is negative 987654 then it means that we have not reseeded the starting pos for the run.
    // Note that even the absolute value is an entirely unrealistic position and a recognizable number.
    float starting_link_pos_{-987654};
    float current_link_pos_{-987654};

    // if this is negative 987654 then it means that we have not reseeded the starting pos for the run.
    // Note that even the absolute value is an entirely unrealistic position and a recognizable number.
    float starting_bckt_pos_{-987654};
    float current_bckt_pos_{-987654};

    // if this is negative 987654 then it means that we have not reseeded the starting pos for the run.
    // Note that even the absolute value is an entirely unrealistic position and a recognizable number.
    float starting_hstp_pos_{-987654};
    float current_hstp_pos_{0}; // TODO this is temporary ok

    // position limits
    const float LINK_MIN_POS_{-1000000}; // TODO replace temp value
    const float LINK_MAX_POS_{1000000}; // TODO replace temp value
    const float BCKT_MIN_POS_{-1000000}; // TODO replace temp value
    const float BCKT_MAX_POS_{1000000}; // TODO replace temp value
    const float HSTP_MIN_POS_{0};
    const float HSTP_MAX_POS_{4096};

    // lookup table for auto dig
// time (s),actuator angle (rots),bucket angle (rots), linact hardstop (encoder [0,4096]),vibration (duty cycle [-1,1])
    float LOOKUP_TB_[7][5] = {
      0,0,0,0,0,
      1,1,1,20,0.2,
      2,2,2,40,0.4,
      3,3,3,60,0.6,
      4,4,4,80,0.8,
      5,5,5,100,1,
      6,5,5,100,0,
    };

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

    /**
     * this gets us the sensor data for where our hardstop is at
     */
    void dig_hstp_cb(const std_msgs::msg::Float32 msg){
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
      link_pwr(0);
      bckt_pwr(0);
      vib_pwr(0);
      hstp_pwr(0);

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
     * sets the linkage motors to run with a specific duty cycle
     * @param pwr the duty cycle for the motors to run at. accepts [-1, 1]
     */
    void link_pwr(double pwr) {
      if (pwr < -1 || pwr > 1)
      {
        RCLCPP_ERROR(this->get_logger(), "link_pwr: Goal was out of bounds. Power goals should always be in [-1, 1]");
        // TODO: should this just be set to 0?
        std::clamp(pwr, -1., 1.);
      }

      l_link_pwr_duty_cycle_.Output = pwr;
      l_link_mtr_.SetControl(l_link_pwr_duty_cycle_);
    }

    /**
     * sets the bucket motors to run with a specific duty cycle
     * @param pwr the duty cycle for the motors to run at. accepts [-1, 1]
     */
    void bckt_pwr(double pwr) {
      if (pwr < -1 || pwr > 1)
      {
        RCLCPP_ERROR(this->get_logger(), "link_pwr: Goal was out of bounds. Power goals should always be in [-1, 1]");
        // TODO: should this just be set to 0?
        std::clamp(pwr, -1., 1.);
      }

      l_bckt_pwr_duty_cycle_.Output = pwr;
      l_bckt_mtr_.SetControl(l_bckt_pwr_duty_cycle_);
    }

    /**
     * sets the vibration motors to run with a specific duty cycle
     * @param pwr the duty cycle for the motors to run at. accepts [-1, 1]
     */
    void vib_pwr(double pwr){
      if (pwr < -1 || pwr > 1)
      {
        RCLCPP_ERROR(this->get_logger(), "vib_pwr: Goal was out of bounds. Power goals should always be in [-1, 1]");
        // TODO: should this just be set to 0?
        std::clamp(pwr, -1., 1.);
      }

      l_vib_mtr_.Heartbeat();
      r_vib_mtr_.Heartbeat();

      l_vib_mtr_.SetDutyCycle(pwr);
      r_vib_mtr_.SetDutyCycle(pwr);
    }

    /**
     * sets the hardstop motor to run with a specific duty cycle
     * @param pwr the duty cycle for the motor to run at. accepts [-1, 1]
     */
    void hstp_pwr(double pwr){
      if (pwr < -1 || pwr > 1)
      {
        RCLCPP_ERROR(this->get_logger(), "hstp_pwr: Goal was out of bounds. Power goals should always be in [-1, 1]");
        // TODO: should this just be set to 0?
        std::clamp(pwr, -1., 1.);
      }

      hstp_mtr_.Heartbeat();
      hstp_mtr_.SetDutyCycle(pwr);
    }

    /**
     * runs the dig motors to a duty cycle goal
     * @param goal_handle pointer to the goal
     */
    void execute_pwr(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_DEBUG(this->get_logger(), "execute_pwr: executing...");

      const auto goal = goal_handle->get_goal();
      double lnkage_goal = goal->dig_link_pwr_goal;
      double bucket_goal = goal->dig_bckt_pwr_goal;
      double hrdstp_goal = goal->dig_hstp_pwr_goal;
      double vibrtn_goal = goal->dig_vibr_pwr_goal;
      RCLCPP_DEBUG(this->get_logger(), "execute_pwr: hrdstp_goal = %f", hrdstp_goal);

      // check that goal is allowable (duty cycle takes [-1, 1])
      if (lnkage_goal < -1 || lnkage_goal > 1 ||
          bucket_goal < -1 || bucket_goal > 1 ||
          hrdstp_goal < -1 || hrdstp_goal > 1 ||
          vibrtn_goal < -1 || vibrtn_goal > 1)
      {
        RCLCPP_ERROR(this->get_logger(), "execute_pwr: Goal was out of bounds. Power goals should always be in [-1, 1]");

        // TODO: should this just be set to 0?
        std::clamp(lnkage_goal, -1., 1.);
        std::clamp(bucket_goal, -1., 1.);
        std::clamp(hrdstp_goal, -1., 1.);
        std::clamp(vibrtn_goal, -1., 1.);
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
      auto &hstpPercentDone = feedback->percent_hstp_done;
      auto &vibrPercentDone = feedback->percent_vibr_done;

      RCLCPP_DEBUG(this->get_logger(), "Running for %f ms", 1000 * (1.0/(double)(LOOP_RATE_HZ_))); //this is the correct math with correct units :)
      ctre::phoenix::unmanaged::FeedEnable(1000 * (1.0/(double)(LOOP_RATE_HZ_)));

      link_pwr(lnkage_goal);
      bckt_pwr(bucket_goal);
      vib_pwr(vibrtn_goal);
      hstp_pwr(hrdstp_goal);

      linkPercentDone = 100;
      bcktPercentDone = 100;
      hstpPercentDone = 100; // TODO
      vibrPercentDone = 100;
      goal_handle->publish_feedback(feedback);

      if (rclcpp::ok())
      {
        result->est_dig_link_goal = lnkage_goal;
        result->est_dig_bckt_goal = bucket_goal;
        result->est_dig_hstp_goal = hrdstp_goal;
        result->est_dig_vibr_goal = vibrtn_goal;

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "execute_pwr: Goal succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "execute_pwr: Goal failed");
      }

      dig_goal_handle_ = nullptr;
      has_goal_ = false;
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
    bool pos_in_bounds(double pos, double min, double max) {
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
    bool hrdstp_in_bounds(double pos) {
      if (pos_in_bounds(pos, HSTP_MIN_POS_, HSTP_MAX_POS_)) {
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "hrdstp_in_bounds: Hardstop goal was out of bounds. Hardstop goal was %f but should be in [%f, %f]", pos, HSTP_MIN_POS_, HSTP_MAX_POS_);
        return false;
      }
    }

    /**
     * Checks if the requested positions are in bounds
     * @param lnkage_pos is the requested position to drive the linkage
     * @param bucket_pos is the requested position to drive the bucket
     * @param hrdstp_pos is the requested position to drive the hardstop
     * @return true if ALL in bounds, fales if ANY out of bounds
     */
    bool positions_in_bounds(double lnkage_pos, double bucket_pos, double hrdstp_pos) {
      return (linkage_in_bounds(lnkage_pos) && bucket_in_bounds(bucket_pos) && hrdstp_in_bounds(hrdstp_pos));
    }

    /**
     * given a goal, set motors to go to that goal
     * no vibration motors bc position control makes no sense for them
     * @param lnkage_goal position in rotations for the linkage motors
     * @param bucket_goal position in rotations for the bucket  motors
     * @param hrdstp_goal position in encoder ticks for the hardstop motor [0, 4096]
     * @param start_time TODO remove this parameter but it's just to estimate wehre the hardstop is until we get sensor
     */
    void goto_pos(double lnkage_goal, double bucket_goal, double hrdstp_goal, double start_time)
    {
      if (!positions_in_bounds(lnkage_goal, bucket_goal, hrdstp_goal))
      {
        RCLCPP_ERROR(this->get_logger(), "goto_pos: Goal was out of bounds");
        return;
      }

      // TODO: need to factor in the absolute encoders (and any other sensor data) from the callback above
      current_link_pos_ = (double)l_link_mtr_.GetPosition().GetValue();
      current_bckt_pos_ = (double)l_bckt_mtr_.GetPosition().GetValue();
      // TODO: hardstop

      RCLCPP_DEBUG(this->get_logger(), "Running for %f ms", 1000 * (1.0/(double)(LOOP_RATE_HZ_))); //this is the correct math with correct units :)
      ctre::phoenix::unmanaged::FeedEnable(1000 * (1.0/(double)(LOOP_RATE_HZ_)));
      units::angle::turn_t lnkage_angl{lnkage_goal * 1_tr};
      units::angle::turn_t bucket_angl{bucket_goal * 1_tr};

      l_link_pos_duty_cycle_.Position = lnkage_angl;
      l_bckt_pos_duty_cycle_.Position = bucket_angl;

      units::angular_velocity::turns_per_second_t lnkage_speed{1};
      units::angular_velocity::turns_per_second_t bucket_speed{1};
      l_link_pos_duty_cycle_.Velocity = lnkage_speed; // rotations per sec
      l_bckt_pos_duty_cycle_.Velocity = bucket_speed; // rotations per sec

      l_link_mtr_.SetControl(l_link_pos_duty_cycle_);
      l_bckt_mtr_.SetControl(l_bckt_pos_duty_cycle_);

      // TODO: hardstop (need sensor lol)
      // hstp_pid.SetReference(100, CtrlType::kPosition);

      // temporarily use power/time
      // 3 in/s unloaded, 2.5 full load. maybe we can assume like 2.9 and tune?
      float hstp_duty_cycle = 1 ? (hrdstp_goal - current_hstp_pos_) : -1;
      hstp_pwr(hstp_duty_cycle);
      // TODO remove this when we get hstop sensor !
      // update estimate pos temporary power time estimate
      // duty cycle drives the direction
      // (duration extending) x (current_power) x (max_speed)
      current_hstp_pos_ += (this->now().seconds() - start_time) * hstp_duty_cycle * HSTP_VEL_;
      RCLCPP_DEBUG(this->get_logger(), "current_hstp_pos_ = %f", current_hstp_pos_);
    }

    /**
     * moves the dig motors to a position goal (except the vibration motors, which are just set to a power)
     * @param goal_handle pointer to the goal
     */
    void execute_pos(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_DEBUG(this->get_logger(), "execute_pos: executing...");

      rclcpp::Rate loop_rate(LOOP_RATE_HZ_);
      const auto goal = goal_handle->get_goal();
      double lnkage_goal = goal->dig_link_pos_goal;
      double bucket_goal = goal->dig_bckt_pos_goal;
      double hrdstp_goal = goal->dig_hstp_pos_goal;
      double vibrtn_goal = goal->dig_vibr_pwr_goal;

      auto feedback = std::make_shared<Dig::Feedback>();
      auto result = std::make_shared<Dig::Result>();

      auto &linkPercentDone = feedback->percent_link_done;
      auto &bcktPercentDone = feedback->percent_bckt_done;
      auto &hstpPercentDone = feedback->percent_hstp_done;
      auto &vibrPercentDone = feedback->percent_vibr_done;

      // TODO: need to factor in the absolute encoders (and any other sensor data) from the callback above
      current_link_pos_ = (double)l_link_mtr_.GetPosition().GetValue();
      current_bckt_pos_ = (double)l_bckt_mtr_.GetPosition().GetValue();
      // TODO: hardstop

      while (!APPROX(current_link_pos_, lnkage_goal) ||
             !APPROX(current_bckt_pos_, bucket_goal) ||
             !APPROX(current_hstp_pos_, hrdstp_goal))
      { // keep sending the request because CTRE's watchdog
        if (goal_handle->is_canceling())
        {
          RCLCPP_INFO(this->get_logger(), "Goal is canceling");
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          dig_goal_handle_ = nullptr;  // Reset the active goal
          has_goal_ = false;
          return;
        }

        // linkage, bucket, and hardstop to a set position
        double start_time = this->now().seconds(); // TODO: remove this once hstp sensor
        goto_pos(lnkage_goal, bucket_goal, hrdstp_goal, start_time);

        // vibration motors duty cycle
        vib_pwr(vibrtn_goal);

        linkPercentDone = (abs(lnkage_goal) - abs(current_link_pos_))/abs(lnkage_goal) * 100;
        bcktPercentDone = (abs(bucket_goal) - abs(current_bckt_pos_))/abs(bucket_goal) * 100;
        hstpPercentDone = (abs(hrdstp_goal) - abs(current_hstp_pos_))/abs(hrdstp_goal) * 100;
        vibrPercentDone = 100;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
      }

      if (rclcpp::ok())
      {
        result->est_dig_link_goal = current_link_pos_;
        result->est_dig_bckt_goal = current_bckt_pos_;
        result->est_dig_hstp_goal = current_hstp_pos_;
        result->est_dig_vibr_goal = vibrtn_goal;

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
     * autonomously moves the dig actuators to scoop
     * @param goal_handle pointer to the goal
     */
    void execute_auton(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      RCLCPP_DEBUG(this->get_logger(), "execute_auton: executing...");

      rclcpp::Rate loop_rate(LOOP_RATE_HZ_);

      auto feedback = std::make_shared<Dig::Feedback>();
      auto result = std::make_shared<Dig::Result>();

      auto &linkPercentDone = feedback->percent_link_done;
      auto &bcktPercentDone = feedback->percent_bckt_done;
      auto &hstpPercentDone = feedback->percent_hstp_done;
      auto &vibrPercentDone = feedback->percent_vibr_done;

      // TODO: need to factor in the absolute encoders (and any other sensor data) from the callback above
      current_link_pos_ = (double)l_link_mtr_.GetPosition().GetValue();
      current_bckt_pos_ = (double)l_bckt_mtr_.GetPosition().GetValue();

// time (s),actuator angle (rots),bucket angle (rots), linact hardstop (encoder [0,4096]),vibration (duty cycle [-1,1])
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

        while (this->now().seconds() < next_goal_time)
        {
          if (goal_handle->is_canceling())
          {
            RCLCPP_INFO(this->get_logger(), "Goal is canceling");
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            dig_goal_handle_ = nullptr;  // Reset the active goal
            has_goal_ = false;
            return;
          }

          // linkage, bucket, and hardstop to a set position
          double start_time = this->now().seconds(); // TODO: remove this once hstp sensor
          goto_pos(LOOKUP_TB_[i][1], LOOKUP_TB_[i][2], LOOKUP_TB_[i][3], start_time);

          // vibration motors duty cycle
          vib_pwr(LOOKUP_TB_[i][4]);

          linkPercentDone = (i/sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0])) * 100;
          bcktPercentDone = (i/sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0])) * 100;
          hstpPercentDone = (i/sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0])) * 100;
          vibrPercentDone = (i/sizeof(LOOKUP_TB_)/sizeof(LOOKUP_TB_[0])) * 100;
          goal_handle->publish_feedback(feedback);

          loop_rate.sleep();
        }

      }

      if (rclcpp::ok())
      {
        result->est_dig_link_goal = current_link_pos_;
        result->est_dig_bckt_goal = current_bckt_pos_;
        result->est_dig_hstp_goal = current_hstp_pos_;
        result->est_dig_vibr_goal = 0;

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "execute_pos: Goal succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "execute_pos: Goal failed");
      }

      dig_goal_handle_ = nullptr;
      has_goal_ = false;
    }

  }; // class DigActionServer

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(dig_server::DigActionServer)
