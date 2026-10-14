#include "buggy_planning/mock_global_planner.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

namespace buggy_planning
{

constexpr double METERS_PER_DEGREE_LAT = 111320.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;

MockGlobalPlanner::MockGlobalPlanner() = default;

bool MockGlobalPlanner::initialize(const std::string & /*map_path*/, double min_battery_threshold)
{
  min_battery_threshold_ = min_battery_threshold;
  return true;
}

void MockGlobalPlanner::set_campus_origin(double lat, double lon)
{
  campus_origin_lat_ = lat;
  campus_origin_lon_ = lon;
}

PlannerResult MockGlobalPlanner::compute_route(
  const sensor_msgs::msg::NavSatFix & current_gps,
  const sensor_msgs::msg::BatteryState & current_bms,
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  PlannerResult result;

  // 1. Safety check: Battery State of Charge
  if (current_bms.percentage < static_cast<float>(min_battery_threshold_)) {
    result.success = false;
    std::ostringstream ss;
    ss << "Planning rejected: Battery too low ("
       << std::fixed << std::setprecision(1) << (current_bms.percentage * 100.0f)
       << "% < threshold " << (min_battery_threshold_ * 100.0) << "%)";
    result.message = ss.str();
    return result;
  }

  // 2. Transform current GPS lat/lon to local map metric coordinates (x, y) relative to origin
  double d_lat = current_gps.latitude - campus_origin_lat_;
  double d_lon = current_gps.longitude - campus_origin_lon_;

  double meters_per_deg_lon = METERS_PER_DEGREE_LAT * std::cos(campus_origin_lat_ * DEG_TO_RAD);
  if (std::abs(meters_per_deg_lon) < 1e-6) {
    meters_per_deg_lon = 1.0;
  }

  double start_x = d_lon * meters_per_deg_lon;
  double start_y = d_lat * METERS_PER_DEGREE_LAT;
  double start_z = 0.0;

  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;
  double goal_z = goal_pose.pose.position.z;

  double dx = goal_x - start_x;
  double dy = goal_y - start_y;
  double dz = goal_z - start_z;
  double total_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  // 3. Populate Path
  result.path.header.stamp = goal_pose.header.stamp;
  result.path.header.frame_id = goal_pose.header.frame_id.empty() ? "map" : goal_pose.header.frame_id;

  int num_waypoints = std::max(2, static_cast<int>(std::ceil(total_distance / waypoint_spacing_m_)));
  double yaw = std::atan2(dy, dx);
  double qz = std::sin(yaw * 0.5);
  double qw = std::cos(yaw * 0.5);

  result.path.poses.reserve(num_waypoints);

  for (int i = 0; i <= num_waypoints; ++i) {
    double alpha = static_cast<double>(i) / static_cast<double>(num_waypoints);
    geometry_msgs::msg::PoseStamped wp;
    wp.header = result.path.header;
    wp.pose.position.x = start_x + (alpha * dx);
    wp.pose.position.y = start_y + (alpha * dy);
    wp.pose.position.z = start_z + (alpha * dz);
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = qz;
    wp.pose.orientation.w = qw;
    result.path.poses.push_back(wp);
  }

  result.success = true;
  result.estimated_distance_m = total_distance;
  result.estimated_duration_s = total_distance / nominal_speed_mps_;

  std::ostringstream ss;
  ss << "Route computed successfully: " << result.path.poses.size()
     << " waypoints, distance: " << std::fixed << std::setprecision(1) << total_distance
     << "m, battery: " << (current_bms.percentage * 100.0f) << "%";
  result.message = ss.str();

  return result;
}

}  // namespace buggy_planning
