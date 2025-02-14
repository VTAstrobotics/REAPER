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
        auto dump_goal = Dump::Goal();
        dump_goal.deposition_goal = 0.01;
        // auto dig_goal = Dig::Goal();
        // auto drive_goal = Drive::Goal();

        // Dump controls
        if (raw.buttons[BUTTON_Y]){
            auto send_dump_goal_options = rclcpp_action::Client<Dump>::SendGoalOptions();
            send_dump_goal_options.goal_response_callback = std::bind(&Distributor::dump_cb, this , _1);

            // send_dump_goal_options.feedback_callback = std::bind(&Distributor::dumpFeedbackCallback, this, _1, _2);
            // send_dump_goal_options.result_callback = std::bind(&Distributor::dumpResultCallback, this, _1);
            this->dump_ptr_->async_send_goal(dump_goal, send_dump_goal_options);
        }

        /*
        if (raw.buttons[BUTTON_A]){
            auto send_dump_goal_options = rclcpp_action::Client<Dump>::SendGoalOptions();
            send_dump_goal_options.goal_response_callback = std::bind(&Distributor::dump_cb, this, _2);
            //send_dump_goal_options.feedback_callback = std::bind(&Distributor::dumpFeedbackCallback, this, _1, _2);
            //send_dump_goal_options.result_callback = std::bind(&Distributor::dumpResultCallback, this, _1);
            this->dump_ptr_->async_send_goal(dump_goal, send_dump_goal_options);
        }
       */

        /*
        // Dig controls
        if ()
        {

        }
        // Drive controls
        if ()
        {

        }
        */
    }

    /*
    Sending Goals (dump, dig, drive)
    */
    void goal_msgs(){
        auto dumpSend_goal = rclcpp_action::Client<Dump>::SendGoalOptions();
        auto digSend_goal = rclcpp_action::Client<Dig>::SendGoalOptions();
        auto driveSend_goal = rclcpp_action::Client<Drive>::SendGoalOptions();

    }

    /*
    Call backs (dump, dig, drive)
     */
    void dump_cb(const DumpGoalHandle::SharedPtr& goal_handle)
    {
        if (!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "Dump goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Dump goal accepted by server, waiting for result");
        }
    }

/*
    void dig_cb(const DigGoalHandle::SharedPtr& goal_handle)
    {
        if (!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "Dig goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Dig goal accepted by server, waiting for result");
        }
    }

    void drive_cb(const DriveGoalHandle::SharedPtr& goal_handle)
    {
        if (!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "Drive goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Drive goal accepted by server, waiting for result");
        }
    }
*/

}; // class Distributor

} // namespace teleop_control

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_control::Distributor)
