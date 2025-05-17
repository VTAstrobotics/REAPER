#include "action_interfaces/action/dig.hpp"

#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ctre/phoenix6/mechanisms/SimpleDifferentialMechanism.hpp"
#include "SparkMax.hpp"
#include "state_messages_utils/motor_to_msg.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "../include/utils.h"

using namespace action_interfaces::action;
using namespace ctre::phoenix6;
using namespace std::placeholders;
namespace dig_server
{
  class DigActionServerNoCancoder : public rclcpp::Node
  {
  public:
    using Dig = action_interfaces::action::Dig;
    using GoalHandleDig = rclcpp_action::ServerGoalHandle<Dig>;

    explicit DigActionServerNoCancoder(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("dig_action_server", options)
    {
      l_link_mtr_.ClearStickyFaults();
      r_link_mtr_.ClearStickyFaults();
      r_bckt_mtr_.ClearStickyFaults();
      l_bckt_mtr_.ClearStickyFaults();

      this->action_server_ = rclcpp_action::create_server<Dig>(
          this,
          "dig",
          std::bind(&DigActionServerNoCancoder::handle_goal, this, _1, _2),
          std::bind(&DigActionServerNoCancoder::handle_cancel, this, _1),
          std::bind(&DigActionServerNoCancoder::handle_accepted, this, _1));

      // RCLCPP_INFO(get_logger(), "Setting severity threshold to DEBUG");
      // auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      // if (ret != RCUTILS_RET_OK) {
      //   RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      //   rcutils_reset_error();
      // }

      // Linkage motor configuration
      configs::TalonFXConfiguration link_configs{};

      // Slot 0 gains
      float K_u = 1.0, T_u = 0.04;
      link_configs.Slot0.GravityType = signals::GravityTypeValue::Arm_Cosine;
      link_configs.Slot0.kS = 0.003;
      link_configs.Slot0.kV = 0.80;
      //link_configs.Slot0.kA = 0.05;
      link_configs.Slot0.kG = 0.325;
      // link_configs.Slot0.kP = 0.1;//0.8 * K_u;
      // link_configs.Slot0.kI = 0; // 0; PD controller
      // link_configs.Slot0.kD = 0.1;//0.1 * K_u * T_u;

      // Slot 1 gains
      link_configs.Slot1.GravityType = signals::GravityTypeValue::Arm_Cosine;
      link_configs.Slot1.kP = 3; // 0.8 * K_u;
      // link_configs.Slot1.kI = 0; // 0; PD controller
      link_configs.Slot1.kD = 0.1; //0.1 * K_u * T_u;

      // Set linkage current limits
      /* calculated by 80 Nm from mechanical as max output on output shaft, at 100:1 gear ratio
       * so 0.8 Nm on motor / 0.01926 kT = 41.5 A. Round down to 40 just in case.
       * see https://ctre.download/files/datasheet/Motor%20Performance%20Analysis%20Report.pdf for KrakenX60 kT */
      link_configs.CurrentLimits.StatorCurrentLimit = 40;
      link_configs.CurrentLimits.StatorCurrentLimitEnable = true;

      // based on wire awg (10)
      link_configs.CurrentLimits.SupplyCurrentLimit = 45;
      link_configs.CurrentLimits.SupplyCurrentLimitEnable = true;

      // Motion Magic!!
      link_configs.MotionMagic.MotionMagicCruiseVelocity = 0.1 ;
      link_configs.MotionMagic.MotionMagicAcceleration = 0.2;
      // link_configs.MotionMagic.MotionMagicJerk = 0; // optional value, skipping now

      // Enable brake mode on the linkage
      link_configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

      // Set leader (left) linkage motor to clockwise positive
      link_configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

      // Apply left linkage configs
      r_link_mtr_.SetControl(controls::Follower{l_link_mtr_.GetDeviceID(), true});
      l_link_mtr_.GetConfigurator().Apply(link_configs);

      // Invert because right motor is mounted inverted to the left (leader)
      link_configs.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

      // Apply right linkage configs
      r_link_mtr_.GetConfigurator().Apply(link_configs);

      // Bucket motor configuration
      configs::TalonFXConfiguration bckt_configs{};

      // Slot 0 gains
      K_u = 0.5, T_u = 0.04;
      // not an arm but the gravity changes based on the angle
      bckt_configs.Slot0.GravityType = signals::GravityTypeValue::Arm_Cosine;
      bckt_configs.Slot0.kS = 0.008;
      bckt_configs.Slot0.kV = 0.73;
      // bckt_configs.Slot0.kA = 0;
      bckt_configs.Slot0.kG = 0.01;
      // bckt_configs.Slot0.kP = 0.8 * K_u;
      // bckt_configs.Slot0.kI = 0; // 0; PD controller
      // bckt_configs.Slot0.kD = 0.1 * K_u * T_u;

      // Slot 1 gains
      bckt_configs.Slot1.GravityType = signals::GravityTypeValue::Arm_Cosine;
      // bckt_configs.Slot1.kP = .03; // 0.8 * K_u;
      // bckt_configs.Slot1.kI = 0; // 0; PD controller
      // bckt_configs.Slot1.kD = 0; //0.1 * K_u * T_u;

      // Set bucket current limits
      /* calculated by 55 Nm from mechanical as max output on output shaft, at 75:1 gear ratio
       * so 0.73 Nm on motor / 0.01926 kT = 38.1 A. Round down to 35 just in case.
       * see https://ctre.download/files/datasheet/Motor%20Performance%20Analysis%20Report.pdf for KrakenX60 kT */
      bckt_configs.CurrentLimits.StatorCurrentLimit = 35;
      bckt_configs.CurrentLimits.StatorCurrentLimitEnable = true;

      // based on wire awg (10)
      bckt_configs.CurrentLimits.SupplyCurrentLimit = 45;
      bckt_configs.CurrentLimits.SupplyCurrentLimitEnable = true;

      // Motion Magic
      bckt_configs.MotionMagic.MotionMagicCruiseVelocity = 0.1;
      bckt_configs.MotionMagic.MotionMagicAcceleration = 0.2;
      // bckt_configs.MotionMagic.MotionMagicJerk = 0; // optional value, skipping now

      // Enable brake mode on the bucket
      bckt_configs.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

      // Set leader (left) bucket motor to clockwise positive
      bckt_configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

      // Soft limits
      bckt_configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      bckt_configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = BCKT_MAX_POS_;
      bckt_configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
      bckt_configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BCKT_MIN_POS_;

      // Apply left bucket configs
      l_bckt_mtr_.GetConfigurator().Apply(bckt_configs);
      r_bckt_mtr_.SetControl(controls::Follower{l_bckt_mtr_.GetDeviceID(), false});

      // Individual configs for the right bucket motor
      // Apply right bucket configs
      r_bckt_mtr_.GetConfigurator().Apply(bckt_configs);

      // Set linkage and bucket to SimpleDifferentialMechanisms

      // TODO finish this?
      // controls::PositionVoltage linkPV = controls::PositionVoltage{0_tr}.WithSlot(0);

      // Left vibration motor (NEO550) configuration
      // l_vib_mtr_.SetIdleMode(IdleMode::kCoast);
      // l_vib_mtr_.SetMotorType(MotorType::kBrushless);
      // l_vib_mtr_.SetSmartCurrentFreeLimit(10.0);
      // l_vib_mtr_.SetSmartCurrentStallLimit(10.0);
      // l_vib_mtr_.BurnFlash();

      // Right vibration motor (NEO550) configuration
      // r_vib_mtr_.SetIdleMode(IdleMode::kCoast);
      // r_vib_mtr_.SetMotorType(MotorType::kBrushless);
      // r_vib_mtr_.SetSmartCurrentFreeLimit(10.0);
      // r_vib_mtr_.SetSmartCurrentStallLimit(10.0);
      // r_vib_mtr_.BurnFlash();

      RCLCPP_DEBUG(this->get_logger(), "Ready for action");
    }

  private:
    rclcpp_action::Server<Dig>::SharedPtr action_server_;

    std::shared_ptr<rclcpp::ParameterEventHandler> parameter_callback;

    // linkage actuators
    hardware::TalonFX l_link_mtr_{20, "can1"}; // canid (each motor), can interface (same for all)
    hardware::TalonFX r_link_mtr_{23, "can1"};
    controls::PositionDutyCycle l_link_pos_duty_cycle_{0 * 0_tr}; // absolute position to reach (in rotations)

    // bucket rotators
    hardware::TalonFX l_bckt_mtr_{21, "can1"};
    hardware::TalonFX r_bckt_mtr_{24, "can1"};
    controls::PositionDutyCycle l_bckt_pos_duty_cycle_{0 * 0_tr}; // absolute position to reach (in rotations)


    // vibration motors
    // SparkMax l_vib_mtr_{"can1", 22};
    // SparkMax r_vib_mtr_{"can1", 25};

    bool has_goal_{false};
    const int LOOP_RATE_HZ_{50};
    /* ridiculous number and recognizable.
     * CORRESPONDS TO THE ACTION DEFINITION (.action)
     * DO NOT CHANGE WITHOUT CHANGING THE ACTION DEFINITION!
     * TODO: def this in header file used in both places? */
    const float DEFAULT_VAL_{-987654.321};

    // position limits
    const float LINK_MIN_POS_{-.15};
    const float LINK_MAX_POS_{0.35};
    const float BCKT_MIN_POS_{-0.35};
    const float BCKT_MAX_POS_{0.35};

    // lookup table for auto dig
    // time (s),actuator angle (external rotation [0, 1]),bucket angle (rotations [0, 1]), vibration (duty cycle [-1,1])
    // float SCOOP_LUT_[7][4] = {
    std::vector<std::vector<float>> SCOOP_LUT_ = {
      {0,0.1,-0.3,0},
      {1,-0.5,0.5,0},
      {2,-0.1,0.16,0},
      {3,-0.1,0.16,0},
      {4,-0.1,0.16,0},
      {5,-0.1,0.16,0},
      {6,-0.1,0.16,0},
    };

    std::vector<std::vector<float>> DIG_TO_DUMP_LUT_ = {
      {0,0.1,-0.3,0},
      {1,-0.5,0.5,0},
      {2,-0.1,0.16,0},
      {3,-0.1,0.16,0},
      {4,-0.1,0.16,0},
      {5,-0.1,0.16,0},
      {6,-0.1,0.16,0},
    };


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
      (void) goal_handle; // for unused warning

      // stop motion
      link_pwr(0);
      bckt_pwr(0);
      vibr_pwr(0);

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
      std::thread{std::bind(&DigActionServerNoCancoder::execute, this, _1), goal_handle}.detach();
    }

    /**
     * Parses the parameters and calls the appropriate helper function
     */
    void execute(const std::shared_ptr<GoalHandleDig> goal_handle)
    {
      static auto left_linkage_logger = state_messages_utils::kraken_to_msg(this->shared_from_this(), "left_linkage", &l_link_mtr_, 100);
      static auto right_linkage_logger = state_messages_utils::kraken_to_msg(this->shared_from_this(), "right_linkage", &r_link_mtr_, 100);
      static auto left_bucket_logger = state_messages_utils::kraken_to_msg(this->shared_from_this(), "left_bucket", &l_bckt_mtr_, 50);
      static auto right_bucket_logger = state_messages_utils::kraken_to_msg(this->shared_from_this(), "right_bucket", &r_bckt_mtr_, 50);

      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<Dig::Feedback>();
      auto result = std::make_shared<Dig::Result>();
      std::vector<std::thread> threads;

      if (goal->scoop || goal->dig_to_dump) { // full auto
        RCLCPP_DEBUG(this->get_logger(), "execute: autonomous control");
        execute_auton(goal_handle, feedback, result); // note we do not need a new thread

      } else { // at least some manual control
        // linkage
        if (!APPROX(goal->link_pos_goal, DEFAULT_VAL_)) {
          RCLCPP_DEBUG(this->get_logger(), "execute: linkage position control");
          threads.emplace_back(std::thread{std::bind(&DigActionServerNoCancoder::exe_link_pos, this, _1, _2, _3), goal_handle, feedback, result});

        } else {
          RCLCPP_DEBUG(this->get_logger(), "execute: linkage power control");
          threads.emplace_back(std::thread{std::bind(&DigActionServerNoCancoder::exe_link_pwr, this, _1, _2, _3), goal_handle, feedback, result});

        }

        // bucket
        if (!APPROX(goal->bckt_pos_goal, DEFAULT_VAL_)) {
          RCLCPP_DEBUG(this->get_logger(), "execute: bucket position control");
          threads.emplace_back(std::thread{std::bind(&DigActionServerNoCancoder::exe_bckt_pos, this, _1, _2, _3), goal_handle, feedback, result});

        } else {
          RCLCPP_DEBUG(this->get_logger(), "execute: bucket power control");
          threads.emplace_back(std::thread{std::bind(&DigActionServerNoCancoder::exe_bckt_pwr, this, _1, _2, _3), goal_handle, feedback, result});

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

      RCLCPP_INFO(this->get_logger(), "link_pwr: = %lf", pwr);

      if (APPROX(pwr, 0)) { // hold position
        controls::DutyCycleOut velocity_command{static_cast<units::dimensionless::scalar_t>(0)}; // velocity turns per second
        l_link_mtr_.SetControl(velocity_command);
      } else {
        controls::DutyCycleOut power_command{static_cast<units::dimensionless::scalar_t>(pwr)};
        l_link_mtr_.SetControl(power_command); // SLOW IF NOT CONNECTED TO THE MOTOR.
      }
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

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "bckt_pwr: bckt_pos L:%lf | R:%lf", (double) l_bckt_mtr_.GetPosition().GetValue(),
        (double) r_bckt_mtr_.GetPosition().GetValue());

      if (pwr == 0) { // hold position
        controls::DutyCycleOut velocity_command{0}; // velocity turns per second
        l_bckt_mtr_.SetControl(velocity_command);
      } else {
        controls::DutyCycleOut power_command{static_cast<units::dimensionless::scalar_t>(pwr)};
        l_bckt_mtr_.SetControl(power_command); // SLOW IF NOT CONNECTED TO THE MOTOR.
      }
    }

    /**
     * sets the vibration motors to run with a specific duty cycle in [-1, 1]
     * @param pwr the duty cycle for the motors to run at
     */
    void vibr_pwr(double pwr){
      if (!pwr_in_bounds(pwr))
      {
        RCLCPP_ERROR(this->get_logger(), "vibr_pwr: %lf was out of bounds. Power goals should always be in [-1, 1]", pwr);
        return;
      }

      // l_vib_mtr_.Heartbeat();
      // r_vib_mtr_.Heartbeat();

      // l_vib_mtr_.SetDutyCycle(pwr);
      // r_vib_mtr_.SetDutyCycle(pwr);
    }

    /**
     *
     */
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
        std::bind(&DigActionServerNoCancoder::link_pwr, this, _1),
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


      execute_pwr(
        goal_handle,
        feedback,
        result,
        bucket_goal,
        bckt_percent_done,
        std::bind(&DigActionServerNoCancoder::bckt_pwr, this, _1),
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
        std::bind(&DigActionServerNoCancoder::vibr_pwr, this, _1),
        result->est_vibr_goal,
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
     * sets the linkage motors to go to a position within its bounds
     * @param pos the position for the linkage to go to
     */
    void link_pos(double pos, double vel = 1) {
        (void)pos, (void)vel; // for unused warning
        RCLCPP_ERROR(this->get_logger(), "IN NO ENCODER FALL BACK MODE, THERE IS NO POSITION CONTROL");
        return;
    }

    /**
     * sets the bucket motors to go to a position within its bounds
     * @param pos the position for the linkage to go to
     */
    void bckt_pos(double pos, double vel = 1) {
        (void)pos, (void)vel; // for unused warning
        RCLCPP_ERROR(this->get_logger(), "IN NO ENCODER FALL BACK MODE, THERE IS NO POSITION CONTROL");
        return;
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
      hardware::TalonFX* motor, const double MIN_POS, const double MAX_POS,
      float& percent_done, std::function<void(double, double)> pos_func,
      float& est_goal, const char* print_prefix)
    {
      (void) result; // for unused warning
      rclcpp::Rate loop_rate(LOOP_RATE_HZ_);
      float current_pos = (float)motor->GetPosition().GetValue();

      while (!reached_pos(current_pos, goal_val, MIN_POS, MAX_POS))
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "execute_pos: cur %f, goal %f, min %f, max %f", current_pos, goal_val, MIN_POS, MAX_POS);
        if (goal_handle->is_canceling()) { return; }

        RCLCPP_DEBUG_ONCE(this->get_logger(), "execute_pos: Loop rate %f ms", 1000 * (1.0/(double)(LOOP_RATE_HZ_))); //this is the correct math with correct units :)
        ctre::phoenix::unmanaged::FeedEnable(1000 * (1.0/(double)(LOOP_RATE_HZ_)));

        pos_func(goal_val, vel);
        current_pos = (float)motor->GetPosition().GetValue();

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

      execute_pos(
        goal_handle,
        feedback,
        result,
        linkage_goal,
        1, // vel
        &l_link_mtr_,
        LINK_MIN_POS_,
        LINK_MAX_POS_,
        link_percent_done,
        std::bind(&DigActionServerNoCancoder::link_pos, this, _1, _2),
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

      execute_pos(
        goal_handle,
        feedback,
        result,
        bucket_goal,
        1, // vel
        &l_bckt_mtr_,
        BCKT_MIN_POS_,
        BCKT_MAX_POS_,
        bckt_percent_done,
        std::bind(&DigActionServerNoCancoder::bckt_pos, this, _1, _2),
        result->est_bckt_goal,
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

      std::vector<std::vector<float>>& lookup_tb = goal_handle->get_goal()->scoop ? SCOOP_LUT_ : DIG_TO_DUMP_LUT_;

      float& link_percent_done = feedback->percent_link_done;
      float& bckt_percent_done = feedback->percent_bckt_done;
      float& vibr_percent_done = feedback->percent_vibr_done;

      // time (s),linkage angle (rots),bucket angle (rots), vibration (duty cycle [-1,1])
      for (size_t i = 0; i < sizeof(lookup_tb)/sizeof(lookup_tb.at(0)); i++)
      {
        // get the starting time for this iteration of the loop
        double next_goal_time = this->now().seconds();

        if (i == sizeof(lookup_tb)/sizeof(lookup_tb.at(0)) - 1)
        {
          // if it's the last iteration, we can't look-ahead, so assume some constant length of time
          next_goal_time += 1; // TODO change this??
        } else {
          next_goal_time += (lookup_tb.at(i+1).at(0) - lookup_tb.at(i).at(0));
        }

        RCLCPP_DEBUG(this->get_logger(), "execute_auton: i=%ld now = %f, nex goal = %f", i, this->now().seconds(), next_goal_time);

        for (size_t j = 0; j < sizeof(lookup_tb.at(0))/sizeof(lookup_tb.at(0).at(0)); j++) {
          RCLCPP_DEBUG(this->get_logger(), "%f, ", lookup_tb.at(i).at(j));
        }

        std::array<std::thread, 2> threads;

        while (this->now().seconds() < next_goal_time)
        {
          if (goal_handle->is_canceling()) { return; }

          // linkage and bucket to a set position
          //link_pos(lookup_tb.at(i).at(1));
          //bckt_pos(lookup_tb.at(i).at(2));

          threads[0] = std::thread([this, goal_handle, feedback, result, lookup_tb, i, &link_percent_done]() {
              execute_pos(
                  goal_handle,
                  feedback,
                  result,
                  lookup_tb.at(i).at(1),
                  1, // vel
                  &l_link_mtr_,
                  LINK_MIN_POS_,
                  LINK_MAX_POS_,
                  link_percent_done,
                  std::bind(&DigActionServerNoCancoder::link_pos, this, std::placeholders::_1, std::placeholders::_2), // Keep the bind for link_pos
                  result->est_link_goal,
                  __func__);
          });

          threads[1] = std::thread([this, goal_handle, feedback, result, lookup_tb, i, &bckt_percent_done]() {
              execute_pos(
                  goal_handle,
                  feedback,
                  result,
                  lookup_tb.at(i).at(2),
                  1, // vel
                  &l_bckt_mtr_,
                  BCKT_MIN_POS_,
                  BCKT_MAX_POS_,
                  bckt_percent_done,
                  std::bind(&DigActionServerNoCancoder::bckt_pos, this, std::placeholders::_1, std::placeholders::_2), // Keep the bind for link_pos
                  result->est_bckt_goal,
                  __func__);
          });

          threads[0].join();
          threads[1].join();

          // vibration motors duty cycle
//          vibr_pwr(lookup_tb.at(i).at(3));

          link_percent_done = (i/sizeof(lookup_tb)/sizeof(lookup_tb.at(0))) * 100;
          bckt_percent_done = (i/sizeof(lookup_tb)/sizeof(lookup_tb.at(0))) * 100;
          vibr_percent_done = (i/sizeof(lookup_tb)/sizeof(lookup_tb.at(0))) * 100;
          goal_handle->publish_feedback(feedback);

          loop_rate.sleep();
        }

      }

      if (rclcpp::ok())
      {
        result->est_link_goal = (double)l_link_mtr_.GetPosition().GetValue();
        result->est_bckt_goal = (double)l_bckt_mtr_.GetPosition().GetValue();
        result->est_vibr_goal = 0;

        // goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "execute_auton: Goal succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "execute_auton: Goal failed");
      }

      // has_goal_ = false;
    }

  }; // class DigActionServerNoCancoder

} // namespace dig_server

RCLCPP_COMPONENTS_REGISTER_NODE(dig_server::DigActionServerNoCancoder)
