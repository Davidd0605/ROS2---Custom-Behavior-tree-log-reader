#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "std_msgs/msg/bool.hpp"

class BehaviorTreeMonitor : public rclcpp::Node
{
public:
  BehaviorTreeMonitor()
  : Node("behavior_tree_monitor")
  {
    subscription_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      "/behavior_tree_log", 10,
      std::bind(&BehaviorTreeMonitor::btLogCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reachable", 10);
  }

private:
  void btLogCallback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg)
  {
    bool goal_reachable = true;

    for (const auto & entry : msg->event_log) {
      std::ostringstream oss;
      oss << entry;  // dump entry into stringstream
      std::string log_str = oss.str();

      // Debug print the full string if needed
      RCLCPP_DEBUG(this->get_logger(), "Entry: %s", log_str.c_str());

      // If "FAILURE" or "IDLE" appears anywhere
      if (log_str.find("FAILURE") != std::string::npos ||
          log_str.find("IDLE") != std::string::npos) {
        goal_reachable = false;
        break;
      }
    }

    std_msgs::msg::Bool result;
    result.data = goal_reachable;
    RCLCPP_INFO(this->get_logger(), "Publishing goal_reachable: %s", goal_reachable ? "true" : "false");
    publisher_->publish(result);
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
