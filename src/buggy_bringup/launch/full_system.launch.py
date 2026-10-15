import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('buggy_bringup')
    default_rviz_config = os.path.join(bringup_dir, 'rviz', 'buggy_view.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2 for 3D visualization'
    )

    declare_sim_speed_arg = DeclareLaunchArgument(
        'sim_speed',
        default_value='3.0',
        description='Simulation speed multiplier (e.g. 1.0 = real time 5m/s, 3.0 = 3x demo speed)'
    )

    # 1. Fleet Manager Node (Ride-sharing, Dijkstra, A*, Constraints, Replanning)
    fleet_manager_node = Node(
        package='buggy_fleet',
        executable='fleet_manager_node',
        name='fleet_manager_node',
        output='screen'
    )

    # 2. Car Motion & Kinematics Node (Nav2 waypoint follower, Odometry, TF, GPS, BMS)
    car_motion_node = Node(
        package='buggy_fleet',
        executable='car_motion_node',
        name='car_motion_node',
        output='screen',
        parameters=[{
            'speed_mps': 5.0,
            'sim_speed_multiplier': LaunchConfiguration('sim_speed'),
            'dwell_time_s': 1.5,
            'initial_battery': 80.0
        }]
    )

    # 3. Web Server & Telemetry WebSocket Bridge (FastAPI, Leaflet OSM portal on port 8000)
    web_server_node = Node(
        package='buggy_fleet',
        executable='web_server',
        name='web_server_node',
        output='screen'
    )

    # 4. Optional RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_sim_speed_arg,
        fleet_manager_node,
        car_motion_node,
        web_server_node,
        rviz_node
    ])
