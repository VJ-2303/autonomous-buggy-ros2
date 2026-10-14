#ifndef BUGGY_PLANNING_I_GLOBAL_PLANNER_HPP_
#define BUGGY_PLANNING_I_GLOBAL_PLANNER_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace buggy_planning
{

/**
 * @brief Structure returned by the global planner containing the computed path and metadata.
 */
struct PlannerResult
{
  bool success{false};
  std::string message;
  nav_msgs::msg::Path path;
  double estimated_distance_m{0.0};
  double estimated_duration_s{0.0};
};

/**
 * @brief Abstract Interface for the Global Planner Algorithm.
 *
 * The Global Planner Team must implement this interface tomorrow.
 * The ROS 2 integration node (GlobalPlannerNode) uses this interface to compute routes
 * based on live or mock GPS and BMS sensor feeds.
 */
class IGlobalPlanner
{
public:
  virtual ~IGlobalPlanner() = default;

  /**
   * @brief Configure the planner algorithm with roadmap data and thresholds.
   *
   * @param map_path Path to the road network graph or map definition file.
   * @param min_battery_threshold Minimum acceptable battery state-of-charge (0.0 to 1.0).
   * @return true if initialized successfully, false otherwise.
   */
  virtual bool initialize(const std::string & map_path, double min_battery_threshold) = 0;

  /**
   * @brief Compute a route from current GPS position to destination pose.
   *
   * @param current_gps Current vehicle coordinates (sensor_msgs/msg/NavSatFix).
   * @param current_bms Current vehicle battery state (sensor_msgs/msg/BatteryState).
   * @param goal_pose Target destination pose (geometry_msgs/msg/PoseStamped).
   * @return PlannerResult containing success flag, status message, and nav_msgs/msg/Path.
   */
  virtual PlannerResult compute_route(
    const sensor_msgs::msg::NavSatFix & current_gps,
    const sensor_msgs::msg::BatteryState & current_bms,
    const geometry_msgs::msg::PoseStamped & goal_pose) = 0;
};

}  // namespace buggy_planning

#endif  // BUGGY_PLANNING_I_GLOBAL_PLANNER_HPP_
