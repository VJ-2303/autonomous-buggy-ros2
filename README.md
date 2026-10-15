# KCT Autonomous Buggy — Live Campus Navigation & Ride-Booking System

An end-to-end, modular ROS 2 Jazzy autonomous vehicle system for Kumaraguru College of Technology (KCT), featuring real-world OpenStreetMap (OSM) georeferencing, multi-rider shared planning algorithms, kinematic vehicle simulation with Nav2 interfaces, and a live web booking and tracking portal.

---

## 1. System Architecture

```mermaid
graph TD
    subgraph WebPortal ["User Web Portal (Port 8000)"]
        Browser["Responsive Web Dashboard<br>(Leaflet.js + OpenStreetMap)"]
        WS["WebSocket & REST API<br>(FastAPI + Uvicorn)"]
        Browser <-->|Live Telemetry & Booking| WS
    end

    subgraph ROS2Bridge ["ROS 2 Communication Layer"]
        WS <-->|rclpy Node| FleetManager["Fleet Manager Node<br>(buggy_fleet)"]
    end

    subgraph CoreAlgorithm ["Campus Navigation & Pooling Engine"]
        FleetManager <-->|evaluates| Planner["Campus Multi-Rider Planner<br>(Dijkstra + A* + Constraints)"]
        Planner <-->|queries| MapGraph["KCT Campus Road Graph<br>(37 Nodes, 45 Edges, WGS84 Georeferenced)"]
    end

    subgraph MotionLayer ["Kinematics & Execution Layer (Nav2 Compatible)"]
        FleetManager -->|/plan (nav_msgs/Path)| CarMotion["Car Motion & Kinematics Node<br>(Nav2 Waypoint Follower)"]
        CarMotion -->|/buggy/car_arrived_node| FleetManager
        CarMotion -->|/odom, /tf| TFEngine["Coordinate Transforms<br>(map -> odom -> base_link -> gps)"]
        CarMotion -->|/sensors/gps/fix| GPS["GPS Fix Telemetry"]
        CarMotion -->|/sensors/bms/battery_state| BMS["Battery Telemetry"]
    end
```

---

## 2. Component Breakdown

The project is structured into clean, single-responsibility modules:

### 1. `buggy_fleet` (Python / `rclpy`)
- **`kct_map.py`**: The complete KCT campus road network (37 nodes, 45 road segments). Georeferenced using an affine transform anchored to Kumaraguru College of Technology GPS coordinates (~11.0797° N, 76.9925° E). Exports full GeoJSON for Leaflet OpenStreetMap.
- **`campus_planner.py`**: Core algorithm matching `doc/campus_nav.py`:
  - **Dijkstra**: Precomputes all-pairs shortest road distances.
  - **A\***: Shortest-path route generation with admissible Euclidean distance heuristic.
  - **Permutations Stop Search**: Generates all valid passenger pickup and drop-off sequences.
  - **Constraint Gating**:
    - **Capacity**: Maximum 4 concurrent seats.
    - **Detour Limit**: Passenger ride distance must not exceed 1.5x direct distance.
    - **Battery Life**: Consumes 0.01% battery per meter; trip rejected if remaining battery drops below 20% reserve.
  - **Dynamic Replanner**: Smoothly inserts new riders into the vehicle's itinerary while the car is en route.
- **`fleet_manager_node.py`**: Coordinates ride bookings, tracks active passengers (`Waiting` &rarr; `Onboard` &rarr; `Completed`), updates stop queues, and publishes canonical `/plan` paths and telemetry.
- **`car_motion_node.py`**: Kinematic vehicle simulator and Nav2 waypoint executor. Smoothly moves the vehicle at cruising speed (5 m/s, configurable simulation speed multiplier), calculates heading angles, updates odometry and TF frames, and simulates battery drain.
- **`web_server.py`**: High-performance FastAPI server running in the ROS 2 environment. Broadcasts 10 Hz real-time telemetry over WebSockets and handles `/api/book` requests.

### 2. `buggy_sensors` (C++ / `rclcpp`)
- C++ mock nodes (`mock_gps_node`, `mock_bms_node`) streaming canonical `sensor_msgs/msg/NavSatFix` and `sensor_msgs/msg/BatteryState` messages for low-latency robotics pipelines.

### 3. `buggy_planning` (C++ / `rclcpp`)
- C++ abstraction layer (`IGlobalPlanner`) defining the contract for point-to-point road routing, with battery safety gating.

### 4. `buggy_bringup`
- Centralized launch files (`full_system.launch.py`, `simulation.launch.py`, `buggy.launch.py`) and pre-configured RViz2 display profile.

### 5. `web/` (Frontend Portal)
- Modern web dashboard built with Leaflet.js and OpenStreetMap.
- Displays live vehicle position icon, seat occupancy counter, animated route line, landmark markers, ride booking interface, decision diagnostic cards, and event logs.

---

## 3. Communication Interface Contract

All components interact through standard ROS 2 message types:

| Topic / Endpoint | Type / Protocol | Rate | Description |
| :--- | :--- | :--- | :--- |
| `/buggy/book_request` | `std_msgs/msg/String` (JSON) | Event | Booking request payload `{"pickup": "...", "drop": "..."}` |
| `/buggy/telemetry` | `std_msgs/msg/String` (JSON) | 5.0 Hz | Comprehensive snapshot of car pose, battery, riders, and stop queue |
| `/plan` | `nav_msgs/msg/Path` | Dynamic | Ordered metric waypoints along the KCT road network for the active route |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Dynamic | Next stop destination waypoint |
| `/odom` | `nav_msgs/msg/Odometry` | 20 Hz | Vehicle kinematic position, orientation, and velocity |
| `/sensors/gps/fix` | `sensor_msgs/msg/NavSatFix` | 20 Hz | Live WGS84 GPS latitude and longitude coordinates |
| `/sensors/bms/battery_state` | `sensor_msgs/msg/BatteryState` | 20 Hz | Pack voltage, percentage, current draw, and health status |
| `ws://localhost:8000/ws` | WebSocket | 10 Hz | Web client live telemetry feed for Leaflet OSM tracking |

---

## 4. Multi-Rider Algorithm Constraints

The routing engine enforces four strict operational rules from `doc/campus_nav.py`:

```
                 Incoming Booking Request
                            │
                            ▼
            ┌───────────────────────────────┐
            │  Generate Stop Permutations   │
            │ (Pickups must precede Drops)  │
            └───────────────┬───────────────┘
                            │
               Check 1: Seat Capacity?
               [Load <= 4 concurrent seats]
                            │ Yes
               Check 2: Detour Ratio?
               [Ride Distance <= 1.5x Direct]
                            │ Yes
               Check 3: Battery Reserve?
               [Battery - (Dist * 0.01%) >= 20%]
                            │ Yes
                            ▼
              ┌───────────────────────────┐
              │ Accept & Dynamically      │
              │ Replan Active Route (A*)  │
              └───────────────────────────┘
```

---

## 5. Quick Start Guide

### 1. Build the Workspace

```bash
cd ~/Code/autonomous
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch the Full System

Run the complete pipeline (fleet manager, car kinematics, web server, and telemetry bridge):

```bash
ros2 launch buggy_bringup full_system.launch.py
```

Optional parameters:
- `sim_speed:=5.0`: Run simulation at 5x speed for faster testing (default: 3.0x).
- `rviz:=true`: Launch RViz2 visualizer alongside the web portal.

### 3. Open the Web Booking Portal

Open your web browser and navigate to:
```
http://localhost:8000
```

- **Interactive Booking**: Click any landmark on the OpenStreetMap canvas (or select from the dropdown) to set pickup and drop-off points.
- **Real-Time Tracking**: Watch the car navigate campus roads, pick up riders, adjust routes on-the-fly when new bookings arrive, and update battery metrics.
- **Reset**: Click the "Reset Car" button in the top navigation bar to reset the simulation back to M Gate.

### 4. Run Automated Test Suite

```bash
colcon test
colcon test-result --verbose
# Output: 7 tests, 0 errors, 0 failures across all packages
```
