#include "buggy_sensors/mock_gps_node.hpp"

#include <cmath>

namespace buggy_sensors
{

constexpr double METERS_PER_DEGREE_LAT = 111320.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;

MockGpsNode::MockGpsNode(const rclcpp::NodeOptions & options)
: Node("mock_gps_node", options)
{
  this->declare_parameter<double>("publish_rate_hz", 5.0);
  this->declare_parameter<std::string>("frame_id", "gps");
  this->declare_parameter<std::string>("motion_mode", "circle");
  this->declare_parameter<double>("latitude_deg", 13.0827);
  this->declare_parameter<double>("longitude_deg", 80.2707);
  this->declare_parameter<double>("altitude_m", 10.0);
  this->declare_parameter<double>("radius_m", 25.0);
  this->declare_parameter<double>("speed_mps", 1.5);

  double rate_hz = this->get_parameter("publish_rate_hz").as_double();
  if (rate_hz <= 0.0) {
    rate_hz = 5.0;
  }
  publish_period_s_ = 1.0 / rate_hz;

  frame_id_ = this->get_parameter("frame_id").as_string();
  motion_mode_ = this->get_parameter("motion_mode").as_string();
  base_lat_ = this->get_parameter("latitude_deg").as_double();
  base_lon_ = this->get_parameter("longitude_deg").as_double();
  base_alt_ = this->get_parameter("altitude_m").as_double();
  radius_m_ = this->get_parameter("radius_m").as_double();
  speed_mps_ = this->get_parameter("speed_mps").as_double();

  // Canonical topic for GPS fix
  fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/sensors/gps/fix", rclcpp::QoS(10));

  auto timer_period = std::chrono::duration<double>(publish_period_s_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&MockGpsNode::publish_fix, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Mock GPS Node initialized. Mode: %s, Rate: %.1f Hz -> Topic: /sensors/gps/fix",
    motion_mode_.c_str(), rate_hz);
}

void MockGpsNode::publish_fix()
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;

  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  double current_lat = base_lat_;
  double current_lon = base_lon_;
  double current_alt = base_alt_;

  if (motion_mode_ == "circle" && radius_m_ > 0.0) {
    double angular_velocity = speed_mps_ / radius_m_;
    double angle = angular_velocity * elapsed_seconds_;
    double dx = radius_m_ * std::cos(angle);
    double dy = radius_m_ * std::sin(angle);

    double meters_per_degree_lon = METERS_PER_DEGREE_LAT * std::cos(base_lat_ * DEG_TO_RAD);
    if (std::abs(meters_per_degree_lon) < 1e-6) {
      meters_per_degree_lon = 1.0;
    }

    current_lat = base_lat_ + (dy / METERS_PER_DEGREE_LAT);
    current_lon = base_lon_ + (dx / meters_per_degree_lon);
  } else if (motion_mode_ == "linear") {
    double distance = speed_mps_ * elapsed_seconds_;
    current_lat = base_lat_ + (distance / METERS_PER_DEGREE_LAT);
  }

  msg.latitude = current_lat;
  msg.longitude = current_lon;
  msg.altitude = current_alt;

  // Covariance indicating high confidence RTK/GPS fix
  msg.position_covariance[0] = 0.04;  // 0.2m stddev
  msg.position_covariance[4] = 0.04;
  msg.position_covariance[8] = 0.16;  // 0.4m altitude stddev
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix_pub_->publish(msg);
  elapsed_seconds_ += publish_period_s_;
}

}  // namespace buggy_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<buggy_sensors::MockGpsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
