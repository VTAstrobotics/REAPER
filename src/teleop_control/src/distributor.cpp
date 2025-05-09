#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bucket_interfaces/action/dig.hpp"
#include "bucket_interfaces/action/dump.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yaml-cpp/yaml.h"

#include <unordered_map>
#include <string>

class Distributor : public rclcpp::Node {
public:
  using Dig = bucket_interfaces::action::Dig;
  using Dump = bucket_interfaces::action::Dump;

  Distributor() : Node("distributor"), slow_turn_(false) {
    this->declare_parameter<std::string>("config_file", "joystick_config.yaml");
    std::string config_path = this->get_parameter("config_file").as_string();
    load_config(config_path);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Distributor::joy_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    bucket_pub_ = this->create_publisher<std_msgs::msg::Bool>("bucket", 10);
    dig_client_ = rclcpp_action::create_client<Dig>(this, "dig");
    dump_client_ = rclcpp_action::create_client<Dump>(this, "dump");
  }

private:
  std::unordered_map<int, std::string> button_bindings_;
  std::unordered_map<int, std::string> axis_bindings_;
  std::unordered_map<std::string, int> action_to_button_;
  std::unordered_map<std::string, int> action_to_axis_;

  bool slow_turn_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bucket_pub_;
  rclcpp_action::Client<Dig>::SharedPtr dig_client_;
  rclcpp_action::Client<Dump>::SharedPtr dump_client_;

  void load_config(const std::string &path) {
    YAML::Node config = YAML::LoadFile(path);
    for (const auto &b : config["buttons"]) {
      int index = b.first.as<int>();
      std::string action = b.second.as<std::string>();
      action_to_button_[action] = index;
    }

    for (const auto &a : config["axes"]) {
      int index = a.first.as<int>();
      std::string action = a.second.as<std::string>();
      action_to_axis_[action] = index;
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Process button actions
    for (const auto &[action, index] : action_to_button_) {
      if (index < msg->buttons.size() && msg->buttons[index]) {
        if (action == "dig") {
          send_dig_goal(5.0);
        } else if (action == "dump") {
          send_dump_goal(0.01);
        } else if (action == "dig_up") {
          RCLCPP_INFO(this->get_logger(), "Digger going up");
        } else if (action == "dig_down") {
          RCLCPP_INFO(this->get_logger(), "Digger going down");
        } else if (action == "toggle_slow_turn") {
          slow_turn_ = !slow_turn_;
        }
      }
    }

    // Process axis actions
    float throttle = get_axis_value(msg, "throttle");
    float turn = get_axis_value(msg, "turn");
    float bucket = get_axis_value(msg, "bucket");

    geometry_msgs::msg::Twist twist;
    twist.linear.x = throttle;
    twist.angular.z = slow_turn_ ? turn * 0.5 : turn;
    twist_pub_->publish(twist);

    std_msgs::msg::Bool bucket_msg;
    bucket_msg.data = (bucket > 0.5);
    bucket_pub_->publish(bucket_msg);
  }

  float get_axis_value(const sensor_msgs::msg::Joy::SharedPtr &msg, const std::string &action) {
    if (action_to_axis_.count(action)) {
      int index = action_to_axis_[action];
      if (index < msg->axes.size()) {
        return msg->axes[index];
      }
    }
    return 0.0f;
  }

  void send_dig_goal(float duration) {
    auto goal_msg = Dig::Goal();
    goal_msg.duration = duration;

    if (!dig_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Dig server not available");
      return;
    }
    dig_client_->async_send_goal(goal_msg);
  }

  void send_dump_goal(float duration) {
    auto goal_msg = Dump::Goal();
    goal_msg.duration = duration;

    if (!dump_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Dump server not available");
      return;
    }
    dump_client_->async_send_goal(goal_msg);
  }
};
