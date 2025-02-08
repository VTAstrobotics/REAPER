#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <functional>
#include "action_interfaces/action/dump.hpp"
#include "action_interfaces/action/dig.hpp"
#include "action_interfaces/action/drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/client.hpp"
#include "settings.h"
#include "utils.h"

using namespace std::placeholders;

/*
Takes in controller inputs and sends a goal to the various action servers (ex. drive, dump, etc)
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


    explicit Distributor(const rclcpp::NodeOptions &options)
        : Node("distrib", options)
    {
        this->Dump_ptr_ = rclcpp_action::create_client<action_interfaces::action::Dump>(this, "Dump");
        this->Dig_ptr_ = rclcpp_action::create_client<action_interfaces::action::Dig>(this, "Dig");
        this->Drive_ptr_ = rclcpp_action::create_client<action_interfaces::action::Drive>(this, "Drive");

        Joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 2, std::bind(&Distributor::topic_callback, this, _1));

    }

    Distributor() : Node("distrib")
    {
        this->Dump_ptr_ = rclcpp_action::create_client<action_interfaces::action::Dump>(this, "Dump");
        this->Dig_ptr_ = rclcpp_action::create_client<action_interfaces::action::Dig>(this, "Dig");
        this->Drive_ptr_ = rclcpp_action::create_client<action_interfaces::action::Drive>(this, "Drive");

        Joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 2, std::bind(&Distributor::topic_callback, this, _1));

    }
private:
    
    /*
    Sharepoints
    */
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Joy_subscription;
    rclcpp_action::Client<action_interfaces::action::Dump>::SharedPtr Dump_ptr_;
    rclcpp_action::Client<action_interfaces::action::Dig>::SharedPtr Dig_ptr_;
    rclcpp_action::Client<action_interfaces::action::Drive>::SharedPtr Drive_ptr_;

    /*
    ?????Joystick messages?????
    */
    void topic_callback(const sensor_msgs::msg::Joy &distribRaw)
    {
        auto Dump_goal_msg = Dump::Goal();
        auto Dig_goal_msg = Dig::Goal();
        auto Drive_goal_msg = Drive::Goal();
        
        // Dump controls
        if (distribRaw.buttons[BUTTON_Y]){
            auto send_dump_goal_options = rclcpp_action::Client<Dump>::SendGoalOptions();
            send_dump_goal_options.goal_response_callback = std::bind(&Distributor::dumpResponseCallback, this , _1);

            //send_dump_goal_options.feedback_callback = std::bind(&Distributor::dumpFeedbackCallback, this, _1, _2);
            //send_dump_goal_options.result_callback = std::bind(&Distributor::dumpResultCallback, this, _1);
            this->Dump_ptr_->async_send_goal(Dump_goal_msg, send_dump_goal_options);
        }
        /*
        if (distribRaw.buttons[BUTTON_A]){
            auto send_dump_goal_options = rclcpp_action::Client<Dump>::SendGoalOptions();
            send_dump_goal_options.goal_response_callback = std::bind(&Distributor::dumpResponseCallback, this, _2);
            //send_dump_goal_options.feedback_callback = std::bind(&Distributor::dumpFeedbackCallback, this, _1, _2);
            //send_dump_goal_options.result_callback = std::bind(&Distributor::dumpResultCallback, this, _1);
            this->Dump_ptr_->async_send_goal(Dump_goal_msg, send_dump_goal_options);
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
        auto dumpSend_goal = rclcpp_action::Client<action_interfaces::action::Dump>::SendGoalOptions();
        auto digSend_goal = rclcpp_action::Client<action_interfaces::action::Dig>::SendGoalOptions();
        auto driveSend_goal = rclcpp_action::Client<action_interfaces::action::Drive>::SendGoalOptions();

    }

    /*
    Call backs (dump, dig, drive)
     */
    void dumpResponseCallback(std::shared_future<DumpGoalHandle::SharedPtr> future)
    {
        auto extracted_goal_handle = DumpGoalHandle.get();
        if (!extracted_goal_handle){
            RCLCPP_ERROR(this->get_logger(), "Dump goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Dump goal accepted by server, waiting for result");
        }
    }    
/*
    void digResponseCallback(std::shared_future<DigGoalHandle::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "Dig goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Dig goal accepted by server, waiting for result");
        }    
    }

    void driveResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<action_interfaces::action::Dump>> goal_handle)
    {
        auto goal_handle = future.get();
        if (!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "Drive goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Drive goal accepted by server, waiting for result");
        }    
    }
*/
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Distributor>());
    rclcpp::shutdown();
    return 0;
}