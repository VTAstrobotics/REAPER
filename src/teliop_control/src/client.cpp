#include <future>
#include <memory>
#include <string>
#include <sstream>

// #include "sensor_msgs/msg/dig.hpp"
// #include "sensor_msgs/msg/drivetrain.hpp"
#include "action_interfaces/action/dump.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/client.hpp"
//#include "rclcpp/client_goal_handle.hpp"
//#include "rclcpp/client_goal_handle_impl.hpp"
//#include "rclcpp_action.hpp"
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


    explicit Distributor(const rclcpp::NodeOptions &options)
        : Node("distrib", options)
    {
        this->Dump_ptr_ = rclcpp_action::create_client<action_interfaces::action::Dump>(this, "Dump");

        Joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 2, std::bind(&Distributor::topic_callback, this, _1));

        auto goal_msg = Dump::Goal();

        digMode = false;
        cooldown = false;
        last_time = 0;
    }

    Distributor() : Node("distrib")
    {
        this->Dump_ptr_ = rclcpp_action::create_client<action_interfaces::action::Dump>(this, "Dump");

        Joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 2, std::bind(&Distributor::topic_callback, this, _1));

        auto goal_msg = Dump::Goal();

        digMode = false;
        cooldown = false;
        last_time = 0;
    }
private:
    
    /*
    Sharepoints
    */
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr Joy_subscription;
    rclcpp_action::Client<action_interfaces::action::Dump>::SharedPtr Dump_ptr_;
    //rclcpp_action::Client<action_interfaces::action::Dig>::SharedPtr Dig_ptr_;
    //rclcpp_action::Client<action_interfaces::action::Drive>::SharedPtr Drive_ptr_;

    /*
    ?????Joystick messages?????
    */
    void topic_callback(const sensor_msgs::msg::Joy &distribRaw)
    {

    }
    bool cooldown;
    bool digMode;
    int32_t last_time;
    

    /*
    Sending Goals (dump, dig, drive)
    */
    void goal_msgs(){
        auto dumpSend_goal = rclcpp_action::Client<action_interfaces::action::Dump>::SendGoalOptions();

    }

    /*
    Call backs (dump, dig, drive)
     */
    void dumpResponseCallback(std::shared_future<DumpGoalHandle::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "Dump goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Dummp goal accepted by server, waiting for result");
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

    void driveResponseCallback(std::shared_future<DriveGoalHandle::SharedPtr> future)
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