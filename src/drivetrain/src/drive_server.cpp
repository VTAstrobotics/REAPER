#include "action_interfaces/action/drive.hpp"
#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"
#include "SparkMax.hpp"
#include "SparkFlex.hpp"
#include "SparkBase.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/imu.hpp"
namespace drive_server
{
  class DriveActionServer : public rclcpp::Node
  {
  public:
    using Drive = action_interfaces::action::Drive;
    using GoalHandleDrive = rclcpp_action::ServerGoalHandle<Drive>;

    explicit DriveActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("drive_action_server", options)
    {
      using namespace std::placeholders;
      

      this->action_server_ = rclcpp_action::create_server<Drive>(
          this,
          "drive",
          std::bind(&DriveActionServer::handle_goal, this, _1, _2),
          std::bind(&DriveActionServer::handle_cancel, this, _1),
          std::bind(&DriveActionServer::handle_accepted, this, _1));

        left_motor.SetIdleMode(IdleMode::kBrake);
        left_motor.SetMotorType(MotorType::kBrushless);
        // left_motor.SetSmartCurrentFreeLimit(50.0);
        left_motor.SetSmartCurrentStallLimit(80.0); // 0.8 Nm
        left_motor.BurnFlash();

        right_motor.SetIdleMode(IdleMode::kBrake);
        right_motor.SetMotorType(MotorType::kBrushless);
        right_motor.SetInverted(true);
        // left_motor.SetSmartCurrentFreeLimit(50.0);
        right_motor.SetSmartCurrentStallLimit(80.0); // 0.8 Nm
        right_motor.BurnFlash();
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/drive/velocity", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/drive/pose", 10);
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&DriveActionServer::imu_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(8), std::bind(&DriveActionServer::timer_callback, this));
        past_time = this->now();
        pastLeftPos = left_motor.GetPosition();
        pastRightPos = right_motor.GetPosition();

      RCLCPP_INFO(this->get_logger(), "Drive action server is ready");
    }

  private:
    rclcpp_action::Server<Drive>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Imu current_imu;

        double drive_left_duty = 0;
        double drive_right_duty = 0;
        SparkMax left_motor{"can0", 10};
        SparkMax right_motor{"can0", 11};
    bool has_goal{false};
    int loop_rate_hz{120};
    std::shared_ptr<GoalHandleDrive> Drive_Goal_Handle;
    double currentLeftPos = 0;
    double currentRightPos = 0;
    double rightVelocity, leftVelocity, new_left_position, 
    new_right_position, left_dist, right_dist;
    double x_     = 0.0;
    double y_     = 0.0;
    double theta_ = 0.0; 
    rclcpp::Time past_time;
    double pastLeftPos;
    double pastRightPos;


    //*****************NOTE***************
    //THESE VALUES NEED TO BE CHANGED TO THE ACTUAL ROBOT VALUES TO TEST
    double wheelCircumference = 0.3; //placeholder
    double normalization_constant = 1; //change this during testing
    double track_width{1.0};


    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Drive::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->velocity_goal.linear.x); // change to drive specific
      (void)uuid;
      if(!has_goal){
        RCLCPP_INFO(this->get_logger(),"Accepted Goal and Will soon Execute it");
        has_goal = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      else{
        RCLCPP_INFO(this->get_logger(),"rejected goal, there must be one still executing");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDrive> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
         // Stop motors immediately
        left_motor.SetDutyCycle(0.0);
        right_motor.SetDutyCycle(0.0);
        RCLCPP_INFO(this->get_logger(), "MOTORS STOPPED");

        has_goal = false;
        (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDrive> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&DriveActionServer::execute, this, _1), goal_handle}.detach();
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
      current_imu = *msg;

    }

    struct Quaterniond
    {
      double w;
      double x;
      double y;
      double z;
    };
    
    Quaterniond toQuat(double yaw, double pitch, double roll)  
    {
      double cy = std::cos(yaw   * 0.5);
      double sy = std::sin(yaw   * 0.5);
      double cp = std::cos(pitch * 0.5);
      double sp = std::sin(pitch * 0.5);
      double cr = std::cos(roll  * 0.5);
      double sr = std::sin(roll  * 0.5);
    
      Quaterniond q;
      q.w = cy * cp * cr + sy * sp * sr;
      q.x = cy * cp * sr - sy * sp * cr;
      q.y = sy * cp * sr + cy * sp * cr;
      q.z = sy * cp * cr - cy * sp * sr;
      return q;
    }

    void getVelocityMessage(geometry_msgs::msg::Twist &velocity_message, double dt){
      new_left_position  = left_motor.GetPosition();  
      new_right_position = right_motor.GetPosition(); 

      double delta_left_ticks  = (new_left_position  - pastLeftPos);
      double delta_right_ticks = (new_right_position - pastRightPos);

      double left_revs  = delta_left_ticks  / 42; //42 ticks per revolution
      double right_revs = delta_right_ticks / 42;

    left_dist  = left_revs  * wheelCircumference;  
    right_dist = right_revs * wheelCircumference;  
      
      if(dt > 0){
        leftVelocity  = left_dist  / dt;
        rightVelocity = right_dist / dt;
      }
      else{
        leftVelocity  = 0;
        rightVelocity = 0;
      }
  
      
      double v = (leftVelocity + rightVelocity) * 0.5;
      double w = (rightVelocity - leftVelocity) / track_width;

      velocity_message.linear.x = v;
      velocity_message.angular.z = w;
    }

    void getPoseMessage(geometry_msgs::msg::Twist &velocity_message, geometry_msgs::msg::Pose &pose_message, double dt){
      double v = velocity_message.linear.x;

      auto q = current_imu.orientation;


      double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      double yaw = std::atan2(siny_cosp, cosy_cosp);

      x_ += v * std::cos(yaw) * dt;
      y_ += v * std::sin(yaw) * dt;

      pose_message.position.x = x_;
      pose_message.position.y = y_;
      pose_message.position.z = 0.0;

      Quaterniond quat = toQuat(yaw, 0.0, 0.0);

      geometry_msgs::msg::Quaternion quat_msg;
      quat_msg.w = quat.w;
      quat_msg.x = quat.x;
      quat_msg.y = quat.y;
      quat_msg.z = quat.z;

      pose_message.orientation = quat_msg;
    }

    void timer_callback(){
      auto current_time = this->now();
      double dt = (current_time - past_time).seconds();
      past_time = current_time;
      geometry_msgs::msg::Twist velocity_message;
      getVelocityMessage(velocity_message, dt);
      geometry_msgs::msg::Pose pose_msg;
      getPoseMessage(velocity_message, pose_msg, dt);
      
      velocity_publisher_->publish(velocity_message);
      pose_publisher_->publish(pose_msg);
      pastLeftPos = new_left_position;
      pastRightPos = new_right_position;
    }

    
    void execute(const std::shared_ptr<GoalHandleDrive> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(loop_rate_hz); // this should be 20 hz which I can't imagine not being enough for the dump

      const auto goal = goal_handle->get_goal();

      auto feedback = std::make_shared<Drive::Feedback>();
      auto result = std::make_shared<Drive::Result>();
      double linear  = goal->velocity_goal.linear.x;
      double angular = goal->velocity_goal.angular.z;

      double v_left  = linear  - angular;
      double v_right = linear  + angular;
      auto start_time = this->now();
      auto end_time = start_time + rclcpp::Duration::from_seconds(0.1);
      while (rclcpp::ok() && this->now() < end_time)
      {
        if (goal_handle->is_canceling()) {
          RCLCPP_INFO(this->get_logger(), "Goal is canceling");
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          Drive_Goal_Handle = nullptr;
          has_goal = false;
          return;
        }
	      left_motor.Heartbeat();
	      right_motor.Heartbeat();
        left_motor.SetDutyCycle(std::min(std::max(v_left, -1.), 1.));
        right_motor.SetDutyCycle(std::min(std::max(v_right, -1.), 1.));
	      feedback->inst_velocity.linear.x  = linear;
        feedback->inst_velocity.angular.z = angular;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
      }

      if (rclcpp::ok())
      {
        result->curr_velocity = goal->velocity_goal;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        Drive_Goal_Handle = nullptr;
        has_goal = false;
      }
    }
  }; // class DriveActionServer

} // namespace drive_server

RCLCPP_COMPONENTS_REGISTER_NODE(drive_server::DriveActionServer)