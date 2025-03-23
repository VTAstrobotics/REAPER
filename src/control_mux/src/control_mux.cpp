#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// https://index.ros.org/p/joy/
// Auto Assist functions
static const int DIG_BUTTON{0};  // A
static const int DUMP_BUTTON{3}; // Y
// manual controls
static const int DEPOSIT_TO_DUMP_MANUAL{11}; // DPAD UP
static const int MOVE_TO_DIGGING_POS{12};    // DPAD DOWN
// manual drivetrain controls
static const int DRIVETRAIN_DRIVE_POWER_AXIS{1}; // left stick y axis
static const int DRIVETRAIN_TURNING_AXIS{2};     // right stick x axis
// safety controls
static const int ROBOT_START{5};      // Xbox Button
static const int ESTOP_LEFT_HALF{4};  // back
static const int ESTOP_RIGHT_HALF{6}; // start

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
   public:
    MinimalSubscriber() : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "topic", 10,
            std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

   private:
    void topic_callback(const sensor_msgs::msg::Joy msg) const
    {
        (void)msg; // for unused warning
                   // handle all the buttons actually doing things
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
