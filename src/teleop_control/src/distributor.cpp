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
        this->dig_ptr_ = rclcpp_action::create_client<Dig>(this, "dig_action");
        this->drive_ptr_ = rclcpp_action::create_client<Drive>(this, "drive");

        this->joy1_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Distributor::joy1_cb, this, _1));

        for (size_t i = 0; i < sizeof(last_btn_press_)/sizeof(*last_btn_press_); i++)
        {
            last_btn_press_[i] = this->now().seconds();
        }
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

    bool slow_turn_ = false; // toggle to slow drive turning
    const float SLOW_DRIVE_TURN_VAL_ = 0.5; // what rate to slow turning
    const float SLOW_BCKT_ROT_VAL_ = 0.125;
    /*
     * store the last time each button was pressed.
     * if the button was JUST pressed we ignore it to avoid unwanted/dup presses
     * array structure is same as the joy message buttons array!
    */
    float last_btn_press_[11];
    const float BUTTON_COOLDOWN_MS_ = 0.050;
    bool teleop_disabled_ = false;
    bool stop_mode_ = false;

    /**
     * Given the button index, returns true if there was a valid press
     * @param button INDEX in the arrays
     * @param raw pointer to the raw joy msg
     * @return true if button was pressed AND if it wasn't a duplicate press (i.e. 1 press shouldnt count as 2)
     */
    bool valid_press (int button, const sensor_msgs::msg::Joy& raw)
    {
        return (raw.buttons[button] && ((this->now().seconds() - last_btn_press_[button]) > BUTTON_COOLDOWN_MS_));
    }

    /**
     * Given an array of button indices, returns true if all were pressed
     * @param buttons is an array of button indices, corresponding to the joy msg
     * @param size is the number of elements in the array
     * @param raw pointer to the raw joy msg
     * @return true if all buttons were pressed AND none were duplicate presses
     */
    bool valid_presses (const int buttons[], const int size, const sensor_msgs::msg::Joy& raw)
    {
        for (int i = 0; i < size; i++)
        {
            if (!valid_press(buttons[i], raw)) { return false; }
        }

        return true;
    }

    /**
     * Joystick 1
     * @param raw the raw message data from the Joy topic
    */
    void joy1_cb(const sensor_msgs::msg::Joy& raw)
    {
        const int STOP_SEQ_BTNS[] = { BUTTON_BACK, BUTTON_START, BUTTON_MANUFACTURER };
        if (valid_presses(STOP_SEQ_BTNS, sizeof(STOP_SEQ_BTNS)/sizeof(*STOP_SEQ_BTNS), raw) && !stop_mode_) {
            RCLCPP_INFO(this->get_logger(), "STOP SEQUENCE DETECTED. SHUTTING DOWN");
            teleop_disabled_ = !teleop_disabled_;
            stop_mode_ = true;
        } else {
            stop_mode_ = false;
        }

        if (teleop_disabled_) { return; }
        /**********************************************************************
         *                                                                    *
         * ACTION SERVER GOALS                                                *
         * You can only send one goal per action server at a time, so each    *
         * control can modify the goal for that iteration to send a goal      *
         *                                                                    *
         **********************************************************************/
        // Drive action server
        auto drive_goal = Drive::Goal();
        geometry_msgs::msg::Twist drive_vel;
        auto send_drive_goal_options = rclcpp_action::Client<Drive>::SendGoalOptions();
        send_drive_goal_options.goal_response_callback = std::bind(&Distributor::drive_response_cb, this, _1);
        send_drive_goal_options.feedback_callback = std::bind(&Distributor::drive_fb_cb, this, _1, _2);
        send_drive_goal_options.result_callback = std::bind(&Distributor::drive_result_cb, this, _1);

        // Dig action server
        auto dig_goal = Dig::Goal();
        auto send_dig_goal_options = rclcpp_action::Client<Dig>::SendGoalOptions();
        send_dig_goal_options.goal_response_callback = std::bind(&Distributor::dig_response_cb, this, _1);
        send_dig_goal_options.feedback_callback = std::bind(&Distributor::dig_fb_cb, this, _1, _2);
        send_dig_goal_options.result_callback = std::bind(&Distributor::dig_result_cb, this, _1);

        // Dump action server
        auto dump_goal = Dump::Goal();
        auto send_dump_goal_options = rclcpp_action::Client<Dump>::SendGoalOptions();
        send_dump_goal_options.goal_response_callback = std::bind(&Distributor::dump_response_cb, this, _1);
        send_dump_goal_options.feedback_callback = std::bind(&Distributor::dump_fb_cb, this, _1, _2);
        send_dump_goal_options.result_callback = std::bind(&Distributor::dump_result_cb, this, _1);

        /**********************************************************************
         *                                                                    *
         * BUTTON CONTROLS                                                    *
         *                                                                    *
         **********************************************************************/

        if (valid_press(BUTTON_A, raw)) {
            RCLCPP_INFO(this->get_logger(), "A: Dumping 0.01 m^3");

            dump_goal.deposition_goal = 0.01;

            this->dump_ptr_->async_send_goal(dump_goal, send_dump_goal_options);
        }

        if (valid_press(BUTTON_B, raw)) {
            RCLCPP_INFO(this->get_logger(), "B: Digging autonomously disabled until encoder values are known (so we dont break the robot)");

            // dig_goal.auton = true;

            // this->dig_ptr_->async_send_goal(dig_goal, send_dig_goal_options);
        }

        if (valid_press(BUTTON_X, raw)) {
            RCLCPP_INFO(this->get_logger(), "X: Dumping 0.1 m^3");

            dump_goal.deposition_goal = 0.01;

            this->dump_ptr_->async_send_goal(dump_goal, send_dump_goal_options);
        }

        if (valid_press(BUTTON_Y, raw)) {
            slow_turn_ = !slow_turn_;
            if (slow_turn_) {
                RCLCPP_INFO(this->get_logger(), "Y: Decreasing the max turning rate");
            } else {
                RCLCPP_INFO(this->get_logger(), "Y: Turning back to full speed");
            }
        }

        if (valid_press(BUTTON_LBUMPER, raw)) {
            RCLCPP_INFO(this->get_logger(), "LB: Lowering the dig linkage");
            dig_goal.dig_link_pwr_goal += 0.05;
        }

        if (valid_press(BUTTON_RBUMPER, raw)) {
            RCLCPP_INFO(this->get_logger(), "RB: Raising the dig linkage");
            dig_goal.dig_link_pwr_goal -= 0.15;
        }

        if (valid_press(BUTTON_BACK, raw)) {
            RCLCPP_INFO(this->get_logger(), "Back: Not yet implemented. Doing nothing...");
        }

        if (valid_press(BUTTON_START, raw)) {
            RCLCPP_INFO(this->get_logger(), "Start: Not yet implemented. Doing nothing...");
        }

        if (valid_press(BUTTON_MANUFACTURER, raw)) {
            RCLCPP_INFO(this->get_logger(), "Xbox: Not yet implemented. Doing nothing...");
        }

        if (valid_press(BUTTON_LSTICK, raw)) {
            RCLCPP_INFO(this->get_logger(), "LS (down): Not yet implemented. Doing nothing...");
        }

        if (valid_press(BUTTON_RSTICK, raw)) {
            RCLCPP_INFO(this->get_logger(), "RS (down): Not yet implemented. Doing nothing...");
        }

        /**********************************************************************
         *                                                                    *
         * PRIMARY DRIVER AXIS CONTROLS                                       *
         *                                                                    *
         *  raw.axes[AXIS_LEFTX] // Drive turn                                *
         *  raw.axes[AXIS_LEFTY] //                                           *
         *                                                                    *
         *  raw.axes[AXIS_RIGHTX] //                                          *
         *  raw.axes[AXIS_RIGHTY] // Dig bucket rotation (up lifts front)     *
         *  raw.axes[AXIS_LTRIGGER] // Drive reverse throttle                 *
         *  raw.axes[AXIS_RTRIGGER] // Drive forward throttle                 *
         *  raw.axes[AXIS_DPAD_X] //                                          *
         *  raw.axes[AXIS_DPAD_Y] //                                          *
         *                                                                    *
         *  drive_vel.linear.x; // straight                                   *
         *  drive_vel.linear.y; //                                            *
         *  drive_vel.linear.z; //                                            *
         *  drive_vel.angular.x; //                                           *
         *  drive_vel.angular.y; //                                           *
         *  drive_vel.angular.z; // turn (positive left, negative right)      *
         *                                                                    *
         **********************************************************************/

        /**********************************************************************
         *                                                                    *
         * DRIVETRAIN CONTROLS                                                *
         *                                                                    *
         **********************************************************************/

        // Drive throttle
        float LT = raw.axes[AXIS_LTRIGGER];
        float RT = raw.axes[AXIS_RTRIGGER];

        /*
         * Shift triggers from [-1, 1], where
         *    1 = not pressed
         *   -1 = fully pressed
         *    0 = halfway
         * to [0, 1] where
         *    0 = not pressed
         *    1 = fully pressed
         */
        LT = ((-1 * LT) + 1) * 0.5;
        RT = ((-1 * RT) + 1) * 0.5;

        // Apply cubic function for better control
        LT = std::pow(LT, 3);
        RT = std::pow(RT, 3);

        drive_vel.linear.x = RT - LT; // [-1, 1]
        drive_vel.linear.x *= 1.5;

        // Drive turning
        float LSX = raw.axes[AXIS_LEFTX]; // [-1 ,1] where -1 = left, 1 = right

        // Apply cubic function for better control
        LSX = std::pow(LSX, 3);

        drive_vel.angular.z = LSX; // [-1, 1]

        if (slow_turn_) { drive_vel.angular.z *= SLOW_DRIVE_TURN_VAL_; }

        /**********************************************************************
         *                                                                    *
         * DIG SYSTEM CONTROLS                                                *
         *                                                                    *
         **********************************************************************/
        // [-1, 1] where -1 = the leading edge of the bucket up, 1 = down
        float RSY = raw.axes[AXIS_RIGHTY];

        // Apply cubic function for better control
        RSY = std::pow(RSY, 3);
        dig_goal.dig_bckt_pwr_goal = RSY;
        dig_goal.dig_bckt_pwr_goal *= SLOW_BCKT_ROT_VAL_;

        /**********************************************************************
         *                                                                    *
         * DUMP SYSTEM CONTROLS                                               *
         *                                                                    *
         **********************************************************************/
        if (raw.axes[AXIS_DPAD_X]) { // in (-1, 0, 1) where -1 = left, 1 = right, 0 = none
            dump_goal.pwr_goal = 0.25 * raw.axes[AXIS_DPAD_X];
            RCLCPP_INFO(this->get_logger(), "Dpad X: Dump with power %f", dump_goal.pwr_goal);
            dump_goal.auton = false;
            this->dump_ptr_->async_send_goal(dump_goal, send_dump_goal_options);
        }

        if (raw.axes[AXIS_DPAD_Y]) { // in (-1, 0, 1) where -1 = down, 1 = up, 0 = none
            RCLCPP_INFO(this->get_logger(), "Dpad Y: Not yet implemented. Doing nothing...");
        }

        // [-1, 1] where -1 = the leading edge of the bucket up, 1 = down
        float LSY = raw.axes[AXIS_LEFTY];

        // Apply cubic function for better control
        LSY = std::pow(LSY, 3);
        dump_goal.pwr_goal = LSY;

        /**********************************************************************
         *                                                                    *
         * SEND ACTION SERVER GOALS                                           *
         * always send a goal for now because autonomy goals would have       *
         * already been sent, so this will be disregarded by the server       *
         * anyway, but in the future we can do something to check if we       *
         * should send.                                                       *
         *                                                                    *
         **********************************************************************/
	drive_goal.velocity_goal = drive_vel;
        this->drive_ptr_->async_send_goal(drive_goal, send_drive_goal_options);
        this->dig_ptr_->async_send_goal(dig_goal, send_dig_goal_options);
        this->dump_ptr_->async_send_goal(dump_goal, send_dump_goal_options);
    }

    /**
     * Sending Goals (dump, dig, drive)
    */
    void goal_msgs(){
        auto dumpSend_goal = rclcpp_action::Client<Dump>::SendGoalOptions();
        auto digSend_goal = rclcpp_action::Client<Dig>::SendGoalOptions();
        auto driveSend_goal = rclcpp_action::Client<Drive>::SendGoalOptions();

    }

    /**
     * @param goal_handle
     */
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

    /**
     * @param goal_handle
     */
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
