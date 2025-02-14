#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "sensor_msgs/msg/joy.hpp"
#include "action_interfaces/action/dump.hpp"
#include "action_interfaces/action/dig.hpp"
#include "action_interfaces/action/drive.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "settings.h"
#include "utils.h"

using std::placeholders::_1;
using std::placeholders::_2;

namespace teleop_control
{

/*
 * Takes in controller inputs and sends a goal to the various action servers (ex. drive, dump, etc)
*/
class Distributor : public rclcpp::Node
{
public:
    using Dump = action_interfaces::action::Dump;
    using DumpGoalHandle = rclcpp_action::ClientGoalHandle<Dump>;
    using Dig = action_interfaces::action::Dig;
    using DigGoalHandle = rclcpp_action::ClientGoalHandle<Dig>;
    using Drive = action_interfaces::action::Drive;
    using DriveGoalHandle = rclcpp_action::ClientGoalHandle<Drive>;

    explicit Distributor(const rclcpp::NodeOptions& options)
        : Node("distributor", options)
    {
        this->dump_ptr_ = rclcpp_action::create_client<Dump>(this, "dump");
        this->dig_ptr_ = rclcpp_action::create_client<Dig>(this, "Dig");
        this->drive_ptr_ = rclcpp_action::create_client<Drive>(this, "Drive");

        this->joy1_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Distributor::joy1_cb, this, _1));

    }

    // void send_goal()
    // {

    // }

private:

    /*
    Sharepoints
    */
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy1_sub_;
    rclcpp_action::Client<Dump>::SharedPtr dump_ptr_;
    rclcpp_action::Client<Dig>::SharedPtr dig_ptr_;
    rclcpp_action::Client<Drive>::SharedPtr drive_ptr_;

    /*
     * Joystick 1
    */
    void joy1_cb(const sensor_msgs::msg::Joy &raw)
    {
        // Drive controls
        auto drive_goal = Drive::Goal();
        geometry_msgs::msg::Twist drive_vel;
        drive_vel.linear.x = 1;
        drive_vel.linear.y = 2;
        drive_vel.linear.z = 3;
        drive_vel.angular.x = 4;
        drive_vel.angular.y = 5;
        drive_vel.angular.z = 6;

        // Dig controls
        auto dig_goal = Dig::Goal();
        dig_goal.auton = true;

        // Dump controls
        auto dump_goal = Dump::Goal();
        dump_goal.deposition_goal = 0.01;

        if (raw.buttons[BUTTON_A]){ //
            auto send_dump_goal_options = rclcpp_action::Client<Dump>::SendGoalOptions();
            send_dump_goal_options.goal_response_callback = std::bind(&Distributor::dump_response_cb, this, _1);
            send_dump_goal_options.feedback_callback = std::bind(&Distributor::dump_fb_cb, this, _1, _2);
            send_dump_goal_options.result_callback = std::bind(&Distributor::dump_result_cb, this, _1);
            this->dump_ptr_->async_send_goal(dump_goal, send_dump_goal_options);
        }

        if (raw.buttons[BUTTON_B]){ //
            auto send_dig_goal_options = rclcpp_action::Client<Dig>::SendGoalOptions();
            send_dig_goal_options.goal_response_callback = std::bind(&Distributor::dig_response_cb, this, _1);
            send_dig_goal_options.feedback_callback = std::bind(&Distributor::dig_fb_cb, this, _1, _2);
            send_dig_goal_options.result_callback = std::bind(&Distributor::dig_result_cb, this, _1);
            this->dig_ptr_->async_send_goal(dig_goal, send_dig_goal_options);
        }

        if (raw.buttons[BUTTON_X]){ //
            auto send_drive_goal_options = rclcpp_action::Client<Drive>::SendGoalOptions();
            send_drive_goal_options.goal_response_callback = std::bind(&Distributor::drive_response_cb, this, _1);
            send_drive_goal_options.feedback_callback = std::bind(&Distributor::drive_fb_cb, this, _1, _2);
            send_drive_goal_options.result_callback = std::bind(&Distributor::drive_result_cb, this, _1);
            this->drive_ptr_->async_send_goal(drive_goal, send_drive_goal_options);
        }

        // if (raw.buttons[BUTTON_Y]){ //
        //     auto send_dump_goal_options = rclcpp_action::Client<Dump>::SendGoalOptions();
        //     send_dump_goal_options.goal_response_callback = std::bind(&Distributor::dump_response_cb, this, _1);
        //     send_dump_goal_options.feedback_callback = std::bind(&Distributor::dump_fb_cb, this, _1, _2);
        //     send_dump_goal_options.result_callback = std::bind(&Distributor::dump_result_cb, this, _1);
        //     this->dump_ptr_->async_send_goal(dump_goal, send_dump_goal_options);
        // }

    }

    /**
     * Sending Goals (dump, dig, drive)
    */
    void goal_msgs(){
        auto dumpSend_goal = rclcpp_action::Client<Dump>::SendGoalOptions();
        auto digSend_goal = rclcpp_action::Client<Dig>::SendGoalOptions();
        auto driveSend_goal = rclcpp_action::Client<Drive>::SendGoalOptions();

    }

    void drive_response_cb(const DriveGoalHandle::SharedPtr& goal_handle)
    {
        if (goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Drive goal accepted by server, waiting for result");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Drive goal was rejected by server");
        }
    }

    /**
     * @param goal_handle
     */
    void drive_fb_cb(DriveGoalHandle::SharedPtr, const std::shared_ptr<const Drive::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Drive instantaneous velocity:\nlinear\n\tx = %f\n\ty = %f\n\tz = %f\nangular\n\tx = %f\n\ty = %f\n\tz = %f",
        feedback->inst_velocity.linear.x, feedback->inst_velocity.linear.y, feedback->inst_velocity.linear.z,
        feedback->inst_velocity.angular.x, feedback->inst_velocity.angular.y, feedback->inst_velocity.angular.z);
    }

    /**
     * @param result
     */
    void drive_result_cb(const DriveGoalHandle::WrappedResult& result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Drive velocity:\nlinear\n\tx = %f\n\ty = %f\n\tz = %f\nangular\n\tx = %f\n\ty = %f\n\tz = %f",
        result.result->curr_velocity.linear.x, result.result->curr_velocity.linear.y, result.result->curr_velocity.linear.z,
        result.result->curr_velocity.angular.x, result.result->curr_velocity.angular.y, result.result->curr_velocity.angular.z);
    }

    void dig_response_cb(const DigGoalHandle::SharedPtr& goal_handle)
    {
        if (goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Dig goal accepted by server, waiting for result");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Dig goal was rejected by server");
        }
    }

    /**
     * @param goal_handle
     */
    void dig_fb_cb(DigGoalHandle::SharedPtr, const std::shared_ptr<const Dig::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Dig linkage %f%% completed and bucket %f%% completed", feedback->percent_link_done, feedback->percent_bckt_done);
    }

    /**
     * @param result
     */
    void dig_result_cb(const DigGoalHandle::WrappedResult& result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Dig linkage at %f and bucket at %f (estimated)", result.result->est_dig_link_goal, result.result->est_dig_bckt_goal);
    }

    /**
     * @param goal_handle
     */
    void dump_response_cb(const DumpGoalHandle::SharedPtr& goal_handle)
    {
        if (goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Dump goal accepted by server, waiting for result");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Dump goal was rejected by server");
        }
    }

    /**
     * @param goal_handle
     */
    void dump_fb_cb(DumpGoalHandle::SharedPtr, const std::shared_ptr<const Dump::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Dump action %f%% completed", feedback->percent_done);
    }

    /**
     * @param result
     */
    void dump_result_cb(const DumpGoalHandle::WrappedResult& result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Dump deposited %f m^3 (estimation)", result.result->est_deposit_goal);
    }

}; // class Distributor

} // namespace teleop_control

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_control::Distributor)
