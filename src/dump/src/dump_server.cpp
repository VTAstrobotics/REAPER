
#include "action_interfaces/action/dump.hpp"
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace action_interfaces::action;
namespace action_tutorials_cpp
{
class DumpActionServer : public rclcpp::Node
{
public:
  using Dump = action_interfaces::action::Dump;
  using GoalHandleDump = rclcpp_action::ServerGoalHandle<Dump>;

  explicit DumpActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dump_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Dump>(
      this,
      "fibonacci",
      std::bind(&DumpActionServer::handle_goal, this, _1, _2),
      std::bind(&DumpActionServer::handle_cancel, this, _1),
      std::bind(&DumpActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Dump>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Dump::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->deposition_goal);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDump> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDump> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DumpActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDump> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Dump::Feedback>();
    auto & sequence = feedback->percent_done;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Dump::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class DumpActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::DumpActionServer)