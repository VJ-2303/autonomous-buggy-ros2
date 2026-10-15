"""
ROS 2 Car Motion Node: Kinematic simulation and Nav2 waypoint execution for KCT Autonomous Buggy.
Publishes live Odometry, TF transforms, NavSatFix GPS, and BMS BatteryState telemetry.
"""

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy.node
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Empty, Int32
from tf2_ros import TransformBroadcaster

from .kct_map import KctMap, meters_to_gps


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw angle (radians) to ROS Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class CarMotionNode(rclpy.node.Node):
    """Simulates real vehicle kinematics, motion execution, and sensor outputs."""

    def __init__(self):
        super().__init__("car_motion_node")

        self.map = KctMap()

        # Parameters
        self.declare_parameter("speed_mps", 5.0)
        self.declare_parameter("sim_speed_multiplier", 3.0)  # 3x speed for fluid interactive demo
        self.declare_parameter("dwell_time_s", 1.0)
        self.declare_parameter("initial_battery", 80.0)

        self.speed_mps = float(self.get_parameter("speed_mps").value)
        self.sim_speed_multiplier = float(self.get_parameter("sim_speed_multiplier").value)
        self.dwell_time_s = float(self.get_parameter("dwell_time_s").value)
        self.battery_pct = float(self.get_parameter("initial_battery").value)

        # Start position at South Gate (M Gate) or first landmark
        start_node = self.map.name_to_id.get("South Gate (M Gate)", next(iter(self.map.name_to_id.values())))
        start_data = self.map.nodes[start_node]
        self.curr_x = float(start_data["x_m"])
        self.curr_y = float(start_data["y_m"])
        self.curr_yaw = 0.0
        self.odometer_m = 0.0

        # Motion queue
        self.waypoints: List[Tuple[float, float, int]] = []
        self.dwell_timer_s = 0.0
        self.is_driving = False

        # Subscriptions
        self.sub_plan = self.create_subscription(Path, "/plan", self.on_new_plan, 10)
        self.sub_reset = self.create_subscription(Empty, "/buggy/reset", self.on_reset, 10)

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_gps = self.create_publisher(NavSatFix, "/sensors/gps/fix", 10)
        self.pub_bms = self.create_publisher(BatteryState, "/sensors/bms/battery_state", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/buggy/car_pose", 10)
        self.pub_arrived = self.create_publisher(Int32, "/buggy/car_arrived_node", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # 20 Hz simulation loop (dt = 0.05s)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_kinematics)

        self.get_logger().info("Car Motion & Kinematics Node running at 20 Hz.")

    def on_reset(self, _msg: Empty):
        """Reset car to initial position."""
        start_node = self.map.name_to_id.get("South Gate (M Gate)", next(iter(self.map.name_to_id.values())))
        start_data = self.map.nodes[start_node]
        self.curr_x = float(start_data["x_m"])
        self.curr_y = float(start_data["y_m"])
        self.curr_yaw = 0.0
        self.waypoints.clear()
        self.dwell_timer_s = 0.0
        self.is_driving = False
        self.battery_pct = float(self.get_parameter("initial_battery").value)
        self.odometer_m = 0.0
        self.get_logger().info("Car motion reset to South Gate (M Gate).")

    def on_new_plan(self, msg: Path):
        """Receive new path waypoints from planner."""
        self.waypoints.clear()
        if not msg.poses:
            self.is_driving = False
            return

        for p in msg.poses:
            wx = p.pose.position.x
            wy = p.pose.position.y
            # Identify closest node in graph
            closest_node_id = self._find_closest_node(wx, wy)
            self.waypoints.append((wx, wy, closest_node_id))

        self.is_driving = True
        self.get_logger().info(f"Received new path with {len(self.waypoints)} waypoints.")

    def _find_closest_node(self, xm: float, ym: float) -> int:
        """Find the graph node closest to metric coordinates (xm, ym)."""
        best_id = 0
        best_dist = float("inf")
        for node in self.map.nodes:
            d = math.hypot(node["x_m"] - xm, node["y_m"] - ym)
            if d < best_dist:
                best_dist = d
                best_id = node["id"]
        return best_id

    def update_kinematics(self):
        """Update vehicle position along the waypoint trajectory."""
        now = self.get_clock().now()

        # Handle dwelling at stops
        if self.dwell_timer_s > 0.0:
            self.dwell_timer_s -= self.dt
            self.publish_states(now, current_speed=0.0)
            return

        effective_speed = 0.0

        if self.is_driving and self.waypoints:
            target_x, target_y, target_node = self.waypoints[0]
            dx = target_x - self.curr_x
            dy = target_y - self.curr_y
            dist_to_wp = math.hypot(dx, dy)

            # Move towards target
            step = self.speed_mps * self.sim_speed_multiplier * self.dt
            effective_speed = self.speed_mps * self.sim_speed_multiplier

            if dist_to_wp <= step:
                self.curr_x = target_x
                self.curr_y = target_y
                self.odometer_m += dist_to_wp
                self.battery_pct = max(0.0, self.battery_pct - (dist_to_wp * 0.01))
                self.waypoints.pop(0)

                # Broadcast node arrival
                msg_arr = Int32()
                msg_arr.data = target_node
                self.pub_arrived.publish(msg_arr)

                if not self.waypoints:
                    self.is_driving = False
                    self.dwell_timer_s = self.dwell_time_s
                    self.get_logger().info("Completed all route waypoints.")
            else:
                target_yaw = math.atan2(dy, dx)
                # Angular interpolation for smooth realistic vehicle turning
                yaw_diff = (target_yaw - self.curr_yaw + math.pi) % (2 * math.pi) - math.pi
                self.curr_yaw += yaw_diff * min(1.0, 15.0 * self.dt)

                self.curr_x += (dx / dist_to_wp) * step
                self.curr_y += (dy / dist_to_wp) * step
                self.odometer_m += step
                self.battery_pct = max(0.0, self.battery_pct - (step * 0.01))

        self.publish_states(now, current_speed=effective_speed)

    def publish_states(self, now, current_speed: float):
        """Publish Odometry, TF transforms, GPS NavSatFix, and BatteryState."""
        stamp = now.to_msg()
        quat = yaw_to_quaternion(self.curr_yaw)

        # 1. Transform: map -> odom -> base_link
        tf_map_base = TransformStamped()
        tf_map_base.header.stamp = stamp
        tf_map_base.header.frame_id = "map"
        tf_map_base.child_frame_id = "base_link"
        tf_map_base.transform.translation.x = self.curr_x
        tf_map_base.transform.translation.y = self.curr_y
        tf_map_base.transform.translation.z = 0.0
        tf_map_base.transform.rotation = quat
        self.tf_broadcaster.sendTransform(tf_map_base)

        # 2. Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.curr_x
        odom.pose.pose.position.y = self.curr_y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = current_speed
        odom.twist.twist.angular.z = 0.0
        self.pub_odom.publish(odom)

        # 3. PoseStamped
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pub_pose.publish(pose)

        # 4. GPS Fix (Directly from metric to real WGS84 Lat/Lon)
        lat, lon = meters_to_gps(self.curr_x, self.curr_y)

        gps = NavSatFix()
        gps.header.stamp = stamp
        gps.header.frame_id = "gps"
        gps.status.status = 0  # STATUS_FIX
        gps.status.service = 1  # SERVICE_GPS
        gps.latitude = lat
        gps.longitude = lon
        gps.altitude = 10.0
        gps.position_covariance[0] = 0.04
        gps.position_covariance[4] = 0.04
        gps.position_covariance[8] = 0.16
        gps.position_covariance_type = 2
        self.pub_gps.publish(gps)

        # 5. BMS Battery State
        bms = BatteryState()
        bms.header.stamp = stamp
        bms.header.frame_id = "bms_frame"
        bms.voltage = float(42.0 + (54.6 - 42.0) * (self.battery_pct / 100.0))
        bms.percentage = float(self.battery_pct / 100.0)
        bms.current = -5.0 if self.is_driving else 0.0
        bms.temperature = 28.5
        bms.power_supply_status = 2  # DISCHARGING
        bms.power_supply_health = 1  # GOOD
        self.pub_bms.publish(bms)


def main(args=None):
    rclpy.init(args=args)
    node = CarMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
