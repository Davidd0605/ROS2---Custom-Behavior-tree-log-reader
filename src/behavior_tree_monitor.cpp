#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "std_msgs/msg/bool.hpp"

class BehaviorTreeMonitor : public rclcpp::Node
{
public:
  BehaviorTreeMonitor()
  : Node("behavior_tree_monitor")
  {
    // Subscriber to behavior_tree_log
    subscription_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      "/behavior_tree_log", 10,
      std::bind(&BehaviorTreeMonitor::btLogCallback, this, std::placeholders::_1));

    // Publisher for goal reachable boolean
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reachable", 10);
  }

private:
void btLogCallback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg)
{
  bool goal_reachable = true;

  for (const auto & log_entry : msg->event_log) {
    std::string current_status = log_entry.current_status;

    if (current_status.find("current_status: FAILURE") != std::string::npos)
    {
      goal_reachable = false;
      break;
    }
  }

  std_msgs::msg::Bool bool_msg;
  bool_msg.data = goal_reachable;

  RCLCPP_INFO(this->get_logger(), "Publishing goal_reachable: %s", goal_reachable ? "true" : "false");
  publisher_->publish(bool_msg);
}

  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorTreeMonitor>());
  rclcpp::shutdown();
  return 0;
}
