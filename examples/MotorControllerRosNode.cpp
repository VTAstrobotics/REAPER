#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ctre/Phoenix.h"
#include "SparkMax.hpp"
#include "PIDController.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

class MotorControllerNode : public rclcpp::Node
{
public:
    MotorControllerNode() : Node("motor_controller_node"), last_command_time_(this->now())
    {
        // Create publishers for motor data
        voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/voltage", 10); //temp locations for now until i confirm where the motors are actually located
        current_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/current", 10);
        temperature_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/temperature", 10);
        duty_cycle_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/duty_cycle", 10);
        position_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/position", 10);
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/velocity", 10);
        encoder_count_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/encoder_count", 10);
        goal_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/current_goal", 10);

        // Create subscriber for motor control (goal inputs)
        goal_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "motor/goal", 10, std::bind(&MotorControllerNode::handleGoal, this, std::placeholders::_1));

        // Initialize SparkMax motor
        spark_max_motor_ = std::make_shared<SparkMax>("can0", 47);
        spark_max_motor_->SetIdleMode(1);  // Brake mode
        spark_max_motor_->SetMotorType(1); // Brushless
        
        pid_controller_ = std::make_shared<PIDController>(*spark_max_motor_);
        configurePIDController();  // Configure PID settings

        // Initialize Kraken motor (replace with actual motor object initialization for Kraken using Phoenix API)
        kraken_motor_ = std::make_shared<ctre::phoenix::motorcontrol::can::TalonFX>(1); // Example ID

        // Timer for publishing motor data
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MotorControllerNode::publishMotorData, this));

        // Watchdog timer to stop motor if no command received in 200ms
        watchdog_timer_ = this->create_wall_timer(
            200ms, std::bind(&MotorControllerNode::checkWatchdog, this));
    }

private:
    void configurePIDController()
    {
        uint8_t slot = 0;  // PID slot 0 for the SparkMax motor
        pid_controller_->SetP(slot, 0.00006f);
        pid_controller_->SetI(slot, 0.000001f);
        pid_controller_->SetD(slot, 1.0f);
        pid_controller_->SetIZone(slot, 0.0f);
        pid_controller_->SetF(slot, 0.000015f);
        pid_controller_->SetSmartMotionMaxVelocity(slot, 5000.0f);
        pid_controller_->SetSmartMotionMaxAccel(slot, 3000.0f);
        spark_max_motor_->BurnFlash();
        RCLCPP_INFO(this->get_logger(), "SparkMax PID configuration applied.");
    }

    void handleGoal(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Assuming msg->data contains [speed, voltage, current, position, torque]
        double speed = msg->data[0];
        double voltage = msg->data[1];
        double current = msg->data[2];
        double position = msg->data[3];
        double torque = msg->data[4];

        // Apply motor control logic based on received goals
        // Control SparkMax motor
        pid_controller_->SetReference(speed, 2); // Velocity control

        // Control Kraken motor
        kraken_motor_->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);

        last_command_time_ = this->now();  // Update watchdog timer
    }

    void publishMotorData()
    {
        // Collect data from SparkMax
        double spark_max_voltage = spark_max_motor_->GetBusVoltage();
        double spark_max_current = spark_max_motor_->GetOutputCurrent();
        double spark_max_temp = spark_max_motor_->GetMotorTemperature();
        double spark_max_velocity = spark_max_motor_->GetVelocity();
        
        // Collect data from Kraken using Phoenix API
        double kraken_voltage = kraken_motor_->GetBusVoltage();
        double kraken_current = kraken_motor_->GetSupplyCurrent();
        double kraken_temp = kraken_motor_->GetTemperature();
        double kraken_position = kraken_motor_->GetSelectedSensorPosition();
        double kraken_velocity = kraken_motor_->GetSelectedSensorVelocity();

        // Publish motor data (you can choose which to publish or publish both)
        std_msgs::msg::Float32 msg;
        
        msg.data = spark_max_voltage;
        voltage_pub_->publish(msg);
        msg.data = spark_max_current;
        current_pub_->publish(msg);
        msg.data = spark_max_temp;
        temperature_pub_->publish(msg);
        msg.data = spark_max_velocity;
        velocity_pub_->publish(msg);

        // Publish Kraken data similarly (or combine into a single topic)
        // More data can be published as needed
    }

    void checkWatchdog()
    {
        // If more than 200 ms have passed since the last command, stop both motors
        auto now = this->now();
        if (now - last_command_time_ > rclcpp::Duration::from_seconds(0.2))
        {
            spark_max_motor_->SetSetpoint(0.0f);  // Stop SparkMax motor
            kraken_motor_->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);  // Stop Kraken motor
            RCLCPP_WARN(this->get_logger(), "No command received for 200ms, stopping motors.");
        }
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_cycle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_count_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr goal_pub_;

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr goal_sub_;

    // Timer for periodic motor data publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Watchdog timer
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_command_time_;

    // Motor control objects
    std::shared_ptr<SparkMax> spark_max_motor_;
    std::shared_ptr<PIDController> pid_controller_;
    std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonFX> kraken_motor_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

