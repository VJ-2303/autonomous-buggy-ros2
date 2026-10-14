#ifndef BUGGY_SENSORS_MOCK_BMS_NODE_HPP_
#define BUGGY_SENSORS_MOCK_BMS_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace buggy_sensors
{

class MockBmsNode : public rclcpp::Node
{
public:
  explicit MockBmsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_bms();

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr bms_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  double nominal_voltage_;
  double current_percentage_;
  double discharge_rate_per_sec_;
  double current_draw_a_;
  double temperature_c_;
  double publish_period_s_{1.0};
};

}  // namespace buggy_sensors

#endif  // BUGGY_SENSORS_MOCK_BMS_NODE_HPP_
