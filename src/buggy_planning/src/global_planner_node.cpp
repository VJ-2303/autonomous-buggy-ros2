#include "buggy_planning/global_planner_node.hpp"
#include "buggy_planning/mock_global_planner.hpp"

namespace buggy_planning
{

GlobalPlannerNode::GlobalPlannerNode(
  const rclcpp::NodeOptions & options,
  std::unique_ptr<IGlobalPlanner> planner)
: Node("global_planner_node", options),
  planner_(std::move(planner))
{
  this->declare_parameter<double>("min_battery_threshold", 0.15);
  this->declare_parameter<double>("campus_origin_lat", 13.0827);
  this->declare_parameter<double>("campus_origin_lon", 80.2707);
  this->declare_parameter<std::string>("map_file_path", "");

  min_battery_threshold_ = this->get_parameter("min_battery_threshold").as_double();
  double campus_lat = this->get_parameter("campus_origin_lat").as_double();
  double campus_lon = this->get_parameter("campus_origin_lon").as_double();
  std::string map_path = this->get_parameter("map_file_path").as_string();

  // If no planner instance was injected, instantiate default MockGlobalPlanner
  if (!planner_) {
    auto mock = std::make_unique<MockGlobalPlanner>();
    mock->set_campus_origin(campus_lat, campus_lon);
    planner_ = std::move(mock);
  }

  planner_->initialize(map_path, min_battery_threshold_);

  // Subscriptions to canonical sensor topics
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/sensors/gps/fix", rclcpp::QoS(10),
    std::bind(&GlobalPlannerNode::on_gps_received, this, std::placeholders::_1));

  bms_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "/sensors/bms/battery_state", rclcpp::QoS(10),
    std::bind(&GlobalPlannerNode::on_bms_received, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::QoS(10),
    std::bind(&GlobalPlannerNode::on_goal_received, this, std::placeholders::_1));

  // Publisher for calculated path
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(),
    "Global Planner Node initialized. Listening to /sensors/gps/fix, /sensors/bms/battery_state, /goal_pose -> Publishing to /plan");
}

void GlobalPlannerNode::on_gps_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  latest_gps_ = *msg;
}

void GlobalPlannerNode::on_bms_received(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  latest_bms_ = *msg;
}

void GlobalPlannerNode::on_goal_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  active_goal_ = *msg;

  RCLCPP_INFO(
    this->get_logger(),
    "New goal received at (%.2f, %.2f) in frame '%s'. Triggering route planning...",
    msg->pose.position.x, msg->pose.position.y, msg->header.frame_id.c_str());

  if (!latest_gps_.has_value()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Cannot compute route: Waiting for valid GPS fix from /sensors/gps/fix");
    return;
  }

  if (!latest_bms_.has_value()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Cannot compute route: Waiting for valid BMS status from /sensors/bms/battery_state");
    return;
  }

  PlannerResult result = planner_->compute_route(*latest_gps_, *latest_bms_, *active_goal_);

  if (result.success) {
    path_pub_->publish(result.path);
    RCLCPP_INFO(this->get_logger(), "%s", result.message.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "%s", result.message.c_str());
  }
}

}  // namespace buggy_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<buggy_planning::GlobalPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
