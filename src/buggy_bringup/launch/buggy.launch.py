import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Master Bringup Launch for Autonomous Buggy.
    Supports seamless switching between Mock and Real Sensor Hardware via launch arguments:
      ros2 launch buggy_bringup buggy.launch.py use_mock_gps:=true use_mock_bms:=true
      ros2 launch buggy_bringup buggy.launch.py use_mock_gps:=false use_mock_bms:=false
    """
    bringup_dir = get_package_share_directory('buggy_bringup')
    sensors_dir = get_package_share_directory('buggy_sensors')
    planning_dir = get_package_share_directory('buggy_planning')

    sensors_config = os.path.join(sensors_dir, 'config', 'mock_sensors.yaml')
    planner_config = os.path.join(planning_dir, 'config', 'planner_params.yaml')

    declare_use_mock_gps = DeclareLaunchArgument(
        'use_mock_gps',
        default_value='true',
        description='Whether to use simulated mock GPS (true) or real hardware driver (false)'
    )

    declare_use_mock_bms = DeclareLaunchArgument(
        'use_mock_bms',
        default_value='true',
        description='Whether to use simulated mock BMS (true) or real hardware driver (false)'
    )

    # 1. Mock GPS Node (active when use_mock_gps:=true)
    mock_gps_node = Node(
        package='buggy_sensors',
        executable='mock_gps_node',
        name='mock_gps_node',
        output='screen',
        parameters=[sensors_config],
        condition=IfCondition(LaunchConfiguration('use_mock_gps'))
    )

    # Note for team: Real GPS driver will be added here when hardware arrives.
    # It must publish sensor_msgs/msg/NavSatFix to canonical topic: /sensors/gps/fix
    real_gps_log = LogInfo(
        msg='[HARDWARE MODE] Real GPS selected. Ensure physical GPS driver is publishing to /sensors/gps/fix',
        condition=UnlessCondition(LaunchConfiguration('use_mock_gps'))
    )

    # 2. Mock BMS Node (active when use_mock_bms:=true)
    mock_bms_node = Node(
        package='buggy_sensors',
        executable='mock_bms_node',
        name='mock_bms_node',
        output='screen',
        parameters=[sensors_config],
        condition=IfCondition(LaunchConfiguration('use_mock_bms'))
    )

    # Note for team: Real BMS CAN/serial driver will be added here.
    # It must publish sensor_msgs/msg/BatteryState to canonical topic: /sensors/bms/battery_state
    real_bms_log = LogInfo(
        msg='[HARDWARE MODE] Real BMS selected. Ensure physical BMS driver is publishing to /sensors/bms/battery_state',
        condition=UnlessCondition(LaunchConfiguration('use_mock_bms'))
    )

    # 3. Global Planner Node (always active, consumes /sensors/gps/fix and /sensors/bms/battery_state)
    global_planner_node = Node(
        package='buggy_planning',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[planner_config]
    )

    return LaunchDescription([
        declare_use_mock_gps,
        declare_use_mock_bms,
        mock_gps_node,
        real_gps_log,
        mock_bms_node,
        real_bms_log,
        global_planner_node
    ])
