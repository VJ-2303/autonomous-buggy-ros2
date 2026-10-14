#include <gtest/gtest.h>
#include <cmath>

#include "buggy_planning/mock_global_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

TEST(GlobalPlannerTest, ComputesValidPathWhenBatteryHealthy)
{
  buggy_planning::MockGlobalPlanner planner;
  planner.initialize("", 0.15);
  planner.set_campus_origin(13.0827, 80.2707);

  sensor_msgs::msg::NavSatFix gps;
  gps.latitude = 13.0827;
  gps.longitude = 80.2707;
  gps.altitude = 10.0;

  sensor_msgs::msg::BatteryState bms;
  bms.percentage = 0.85f;  // 85% healthy

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 20.0;
  goal.pose.position.y = 10.0;

  auto result = planner.compute_route(gps, bms, goal);

  EXPECT_TRUE(result.success);
  EXPECT_GT(result.path.poses.size(), 1u);
  EXPECT_NEAR(result.estimated_distance_m, std::sqrt(20.0 * 20.0 + 10.0 * 10.0), 0.1);
  EXPECT_EQ(result.path.header.frame_id, "map");
}

TEST(GlobalPlannerTest, RejectsRouteWhenBatteryLow)
{
  buggy_planning::MockGlobalPlanner planner;
  planner.initialize("", 0.20);  // 20% min threshold
  planner.set_campus_origin(13.0827, 80.2707);

  sensor_msgs::msg::NavSatFix gps;
  gps.latitude = 13.0827;
  gps.longitude = 80.2707;

  sensor_msgs::msg::BatteryState bms;
  bms.percentage = 0.10f;  // 10% (below 20% threshold)

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 50.0;
  goal.pose.position.y = 0.0;

  auto result = planner.compute_route(gps, bms, goal);

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.path.poses.empty());
  EXPECT_NE(result.message.find("Battery too low"), std::string::npos);
}
