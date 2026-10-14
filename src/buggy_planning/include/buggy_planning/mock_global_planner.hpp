#ifndef BUGGY_PLANNING_MOCK_GLOBAL_PLANNER_HPP_
#define BUGGY_PLANNING_MOCK_GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "buggy_planning/i_global_planner.hpp"

namespace buggy_planning
{

class MockGlobalPlanner : public IGlobalPlanner
{
public:
  MockGlobalPlanner();
  ~MockGlobalPlanner() override = default;

  bool initialize(const std::string & map_path, double min_battery_threshold) override;

  PlannerResult compute_route(
    const sensor_msgs::msg::NavSatFix & current_gps,
    const sensor_msgs::msg::BatteryState & current_bms,
    const geometry_msgs::msg::PoseStamped & goal_pose) override;

  void set_campus_origin(double lat, double lon);

private:
  double min_battery_threshold_{0.15};
  double campus_origin_lat_{13.0827};
  double campus_origin_lon_{80.2707};
  double waypoint_spacing_m_{1.0};
  double nominal_speed_mps_{2.0};
};

}  // namespace buggy_planning

#endif  // BUGGY_PLANNING_MOCK_GLOBAL_PLANNER_HPP_
