#include "buggy_sensors/mock_bms_node.hpp"

#include <algorithm>

namespace buggy_sensors
{

MockBmsNode::MockBmsNode(const rclcpp::NodeOptions & options)
: Node("mock_bms_node", options)
{
  this->declare_parameter<double>("publish_rate_hz", 1.0);
  this->declare_parameter<std::string>("frame_id", "bms_frame");
  this->declare_parameter<double>("nominal_voltage", 48.0);
  this->declare_parameter<double>("initial_percentage", 0.95);
  this->declare_parameter<double>("discharge_rate_per_min", 0.005);
  this->declare_parameter<double>("current_draw_a", -5.2);
  this->declare_parameter<double>("temperature_c", 28.5);

  double rate_hz = this->get_parameter("publish_rate_hz").as_double();
  if (rate_hz <= 0.0) {
    rate_hz = 1.0;
  }
  publish_period_s_ = 1.0 / rate_hz;

  frame_id_ = this->get_parameter("frame_id").as_string();
  nominal_voltage_ = this->get_parameter("nominal_voltage").as_double();
  current_percentage_ = this->get_parameter("initial_percentage").as_double();
  double discharge_rate_per_min = this->get_parameter("discharge_rate_per_min").as_double();
  discharge_rate_per_sec_ = discharge_rate_per_min / 60.0;
  current_draw_a_ = this->get_parameter("current_draw_a").as_double();
  temperature_c_ = this->get_parameter("temperature_c").as_double();

  // Canonical topic for BMS battery state
  bms_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    "/sensors/bms/battery_state", rclcpp::QoS(10));

  auto timer_period = std::chrono::duration<double>(publish_period_s_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&MockBmsNode::publish_bms, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Mock BMS Node initialized. Voltage: %.1fV, Batt: %.1f%% -> Topic: /sensors/bms/battery_state",
    nominal_voltage_, current_percentage_ * 100.0);
}

void MockBmsNode::publish_bms()
{
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;

  // Slowly discharge battery over time
  current_percentage_ = std::max(0.0, current_percentage_ - (discharge_rate_per_sec_ * publish_period_s_));

  // Estimate instantaneous voltage based on SOC
  // Nominal 48V pack: 42V (empty) to 54.6V (full)
  double min_v = nominal_voltage_ * 0.875;
  double max_v = nominal_voltage_ * 1.1375;
  double current_voltage = min_v + (max_v - min_v) * current_percentage_;

  msg.voltage = static_cast<float>(current_voltage);
  msg.temperature = static_cast<float>(temperature_c_);
  msg.current = static_cast<float>(current_draw_a_);
  msg.percentage = static_cast<float>(current_percentage_);
  msg.charge = static_cast<float>(current_percentage_ * 50.0);  // e.g. 50Ah pack
  msg.capacity = 50.0f;
  msg.design_capacity = 50.0f;
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  msg.present = true;

  bms_pub_->publish(msg);
}

}  // namespace buggy_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<buggy_sensors::MockBmsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
