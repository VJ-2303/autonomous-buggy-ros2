"""
ROS 2 Fleet Manager Node: Coordinates rider booking requests, dynamic multi-rider planning,
and routes for the Autonomous Buggy on KCT Campus.
"""

import json
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
import rclpy.node
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Empty, Int32, String

from .campus_planner import CampusPlanner
from .kct_map import KctMap, gps_to_meters, meters_to_gps

RIDER_COLORS = ["#e67e22", "#8e44ad", "#16a085", "#c0392b", "#2980b9", "#d35400"]
RIDER_LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"


class FleetManagerNode(rclpy.node.Node):
    """Manages campus ride requests, multi-rider constraints, and publishes vehicle routes."""

    def __init__(self):
        super().__init__("fleet_manager_node")

        self.map = KctMap()
        self.planner = CampusPlanner(self.map)

        # Vehicle internal state
        # Start at South Gate (M Gate) or first landmark
        self.default_start_node = self.map.name_to_id.get("South Gate (M Gate)", next(iter(self.map.name_to_id.values())))
        self.current_node = self.default_start_node
        self.battery_pct = 80.0
        self.is_moving = False
        self.total_odometer_m = 0.0

        self.riders: List[Dict] = []
        self.next_rider_idx = 0
        self.active_plan: Optional[Dict] = None
        self.stop_queue: List[Dict] = []
        self.event_logs: List[str] = [
            f"Car initialized at {self.map.id_to_name[self.current_node]}, battery {self.battery_pct:.1f}%."
        ]

        # Subscriptions
        self.sub_book = self.create_subscription(
            String, "/buggy/book_request", self.on_book_request, 10
        )
        self.sub_reset = self.create_subscription(
            Empty, "/buggy/reset", self.on_reset, 10
        )
        self.sub_bms = self.create_subscription(
            BatteryState, "/sensors/bms/battery_state", self.on_bms_state, 10
        )
        self.sub_arrived = self.create_subscription(
            Int32, "/buggy/car_arrived_node", self.on_car_arrived_node, 10
        )

        # Publishers
        self.pub_plan = self.create_publisher(Path, "/plan", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.pub_telemetry = self.create_publisher(String, "/buggy/telemetry", 10)

        # Broadcast telemetry periodically (5 Hz)
        self.timer_telemetry = self.create_timer(0.2, self.publish_telemetry)

        self.get_logger().info(
            f"Fleet Manager Node ready. Start node: {self.map.id_to_name[self.current_node]} ({self.current_node})"
        )

    def on_bms_state(self, msg: BatteryState):
        """Update battery state from live BMS or simulation."""
        self.battery_pct = float(msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage)

    def on_reset(self, _msg: Empty):
        """Reset fleet manager state to beginning."""
        self.current_node = self.default_start_node
        self.battery_pct = 80.0
        self.is_moving = False
        self.riders.clear()
        self.next_rider_idx = 0
        self.active_plan = None
        self.stop_queue.clear()
        self.event_logs.append(f"Reset car to {self.map.id_to_name[self.current_node]}.")
        self.publish_empty_plan()
        self.get_logger().info("Fleet manager reset.")

    def on_book_request(self, msg: String):
        """Handle incoming ride booking request."""
        try:
            data = json.loads(msg.data)
            pickup_raw = data.get("pickup")
            drop_raw = data.get("drop")

            p_id = self._resolve_node_id(pickup_raw)
            d_id = self._resolve_node_id(drop_raw)

            if p_id is None or d_id is None:
                self.event_logs.append(f"Booking rejected: Invalid locations '{pickup_raw}' -> '{drop_raw}'")
                return

            if p_id == d_id:
                self.event_logs.append("Booking rejected: Pickup and drop locations must be different.")
                return

            rider_id = RIDER_LETTERS[self.next_rider_idx % len(RIDER_LETTERS)]
            color = RIDER_COLORS[self.next_rider_idx % len(RIDER_COLORS)]
            self.next_rider_idx += 1

            new_rider = {
                "id": rider_id,
                "pickup": p_id,
                "drop": d_id,
                "status": "wait",
                "color": color,
                "ride_distance_m": 0.0,
            }

            active = [r for r in self.riders if r["status"] in ("wait", "on")]
            if len(active) >= 4:
                new_rider["status"] = "rejected"
                self.riders.append(new_rider)
                self.event_logs.append(f"Rider {rider_id} rejected: Car at maximum 4 seats capacity.")
                return

            plan_result = self.planner.plan(
                car_node=self.current_node,
                battery_pct=self.battery_pct,
                active_riders=active,
                new_rider=new_rider,
            )

            if not plan_result.get("ok"):
                new_rider["status"] = "rejected"
                self.riders.append(new_rider)
                rej = plan_result.get("rejected", {})
                self.event_logs.append(
                    f"Rider {rider_id} rejected ({self.map.id_to_name[p_id]} -> {self.map.id_to_name[d_id]}): "
                    f"Seats: {rej.get('seats', 0)}, Detour: {rej.get('detour', 0)}, Battery: {rej.get('battery', 0)}."
                )
                return

            # Accepted: Update active riders and path
            self.riders.append(new_rider)
            self.active_plan = plan_result
            self.stop_queue = plan_result.get("stops", [])
            self.is_moving = True

            order_str = " -> ".join(plan_result.get("order", []))
            self.event_logs.append(
                f"Rider {rider_id} accepted ({self.map.id_to_name[p_id]} -> {self.map.id_to_name[d_id]}). "
                f"New Route: {order_str} ({plan_result.get('distance_m')}m, {plan_result.get('time_s')}s)."
            )

            # Publish ROS 2 nav_msgs/Path and target goal
            self.publish_ros_plan(plan_result.get("path_nodes", []))

        except Exception as e:
            self.get_logger().error(f"Error handling book request: {e}")

    def on_car_arrived_node(self, msg: Int32):
        """Called when simulated or real car reaches a road node."""
        node_id = msg.data
        self.current_node = node_id

        # Check if any stops in the queue are at this node
        if self.stop_queue and self.stop_queue[0]["node"] == node_id:
            stop = self.stop_queue.pop(0)
            rider_id = stop["rider_id"]
            stop_type = stop["type"]

            for r in self.riders:
                if r["id"] == rider_id:
                    if stop_type == "P" and r["status"] == "wait":
                        r["status"] = "on"
                        self.event_logs.append(f"Picked up Rider {rider_id} at {self.map.id_to_name[node_id]}.")
                    elif stop_type == "D" and r["status"] == "on":
                        r["status"] = "done"
                        direct = self.planner.D[r["pickup"]][r["drop"]]
                        self.event_logs.append(
                            f"Dropped off Rider {rider_id} at {self.map.id_to_name[node_id]}. "
                            f"Total ride: {r['ride_distance_m']:.0f}m (direct: {direct:.0f}m)."
                        )

        if not self.stop_queue:
            self.is_moving = False

    def _resolve_node_id(self, val) -> Optional[int]:
        """Convert name or int to valid node id."""
        if isinstance(val, int):
            return val if 0 <= val < len(self.map.nodes) else None
        if isinstance(val, str):
            if val.isdigit():
                idx = int(val)
                return idx if 0 <= idx < len(self.map.nodes) else None
            return self.map.name_to_id.get(val)
        return None

    def publish_empty_plan(self):
        """Publish empty path to clear RViz and controller."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self.pub_plan.publish(msg)

    def publish_ros_plan(self, path_node_ids: List[int]):
        """Publish standard nav_msgs/msg/Path across fine-grained smooth interpolated waypoints."""
        if not path_node_ids:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        fine_points = self.map.interpolate_path(path_node_ids, step_meters=1.5)

        for xm, ym, lat, lon in fine_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(xm)
            pose.pose.position.y = float(ym)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.pub_plan.publish(path_msg)

        if path_msg.poses:
            self.pub_goal.publish(path_msg.poses[-1])

    def publish_telemetry(self):
        """Publish real-time telemetry JSON for web clients and monitoring."""
        cur_node_data = self.map.nodes[self.current_node]

        telemetry = {
            "car": {
                "node_id": self.current_node,
                "node_name": cur_node_data["name"],
                "x_m": cur_node_data["x_m"],
                "y_m": cur_node_data["y_m"],
                "lat": cur_node_data["lat"],
                "lon": cur_node_data["lon"],
                "battery": round(self.battery_pct, 1),
                "is_moving": self.is_moving,
                "odometer_m": round(self.total_odometer_m, 1),
                "onboard_count": len([r for r in self.riders if r["status"] == "on"]),
                "capacity": 4,
            },
            "riders": self.riders,
            "queue": [f"{s['type']}_{s['rider_id']}" for s in self.stop_queue],
            "decision": self.active_plan,
            "logs": self.event_logs[-30:],
        }

        msg = String()
        msg.data = json.dumps(telemetry)
        self.pub_telemetry.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
