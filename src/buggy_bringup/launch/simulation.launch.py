import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('buggy_bringup')
    sensors_dir = get_package_share_directory('buggy_sensors')
    planning_dir = get_package_share_directory('buggy_planning')

    default_rviz_config = os.path.join(bringup_dir, 'rviz', 'buggy_view.rviz')
    sensors_config = os.path.join(sensors_dir, 'config', 'mock_sensors.yaml')
    planner_config = os.path.join(planning_dir, 'config', 'planner_params.yaml')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2 for visualization'
    )

    mock_gps_node = Node(
        package='buggy_sensors',
        executable='mock_gps_node',
        name='mock_gps_node',
        output='screen',
        parameters=[sensors_config]
    )

    mock_bms_node = Node(
        package='buggy_sensors',
        executable='mock_bms_node',
        name='mock_bms_node',
        output='screen',
        parameters=[sensors_config]
    )

    global_planner_node = Node(
        package='buggy_planning',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[planner_config]
    )

    # Static transforms for visualization (map -> odom -> base_link -> gps)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--frame-id', 'odom', '--child-frame-id', 'base_link']
    )

    static_tf_base_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_gps',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--frame-id', 'base_link', '--child-frame-id', 'gps']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    health_monitor_node = Node(
        package='buggy_fleet',
        executable='health_monitor_node',
        name='health_monitor_node',
        output='screen'
    )


    return LaunchDescription([
        declare_rviz_arg,
        mock_gps_node,
        mock_bms_node,
        global_planner_node,
        health_monitor_node,
        static_tf_map_odom,
        static_tf_odom_base,
        static_tf_base_gps,
        rviz_node
    ])
