#ifndef BUGGY_SENSORS_MOCK_GPS_NODE_HPP_
#define BUGGY_SENSORS_MOCK_GPS_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace buggy_sensors
{

class MockGpsNode : public rclcpp::Node
{
public:
  explicit MockGpsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_fix();

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  std::string motion_mode_;
  double base_lat_;
  double base_lon_;
  double base_alt_;
  double radius_m_;
  double speed_mps_;
  double elapsed_seconds_{0.0};
  double publish_period_s_{0.2};
};

}  // namespace buggy_sensors

#endif  // BUGGY_SENSORS_MOCK_GPS_NODE_HPP_
