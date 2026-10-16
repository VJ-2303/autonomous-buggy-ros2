
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, BatteryState


class HealthMonitorNode(Node):
    def __init__(self):
        super().__init__("health_monitor_node")

        self.gps_timeout = 3.0
        self.battery_timeout = 3.0

        self.last_gps_time = None
        self.last_battery_time = None

        self.create_subscription(
            NavSatFix,
            "/sensors/gps/fix",
            self.gps_callback,
            10,
        )

        self.create_subscription(
            BatteryState,
            "/sensors/bms/battery_state",
            self.battery_callback,
            10,
        )

        self.create_timer(1.0, self.check_health)

        self.get_logger().info("Health monitor started.")

    def gps_callback(self, msg):
        self.last_gps_time = self.get_clock().now()

    def battery_callback(self, msg):
        self.last_battery_time = self.get_clock().now()

    def is_stale(self, last_time, timeout):
        if last_time is None:
            return True

        elapsed = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return elapsed > timeout

    def check_health(self):
        if self.is_stale(self.last_gps_time, self.gps_timeout):
            self.get_logger().warning("GPS data is missing or stale.")
        else:
            self.get_logger().info("GPS: OK")

        if self.is_stale(self.last_battery_time, self.battery_timeout):
            self.get_logger().warning("Battery data is missing or stale.")
        else:
            self.get_logger().info("Battery: OK")



def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
