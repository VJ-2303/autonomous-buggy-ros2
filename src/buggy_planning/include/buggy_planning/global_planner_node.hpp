#ifndef BUGGY_PLANNING_GLOBAL_PLANNER_NODE_HPP_
#define BUGGY_PLANNING_GLOBAL_PLANNER_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "buggy_planning/i_global_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace buggy_planning
{

class GlobalPlannerNode : public rclcpp::Node
{
public:
  explicit GlobalPlannerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<IGlobalPlanner> planner = nullptr);

private:
  void on_gps_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void on_bms_received(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void on_goal_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bms_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::unique_ptr<IGlobalPlanner> planner_;

  std::optional<sensor_msgs::msg::NavSatFix> latest_gps_;
  std::optional<sensor_msgs::msg::BatteryState> latest_bms_;
  std::optional<geometry_msgs::msg::PoseStamped> active_goal_;

  double min_battery_threshold_{0.15};
};

}  // namespace buggy_planning

#endif  // BUGGY_PLANNING_GLOBAL_PLANNER_NODE_HPP_
