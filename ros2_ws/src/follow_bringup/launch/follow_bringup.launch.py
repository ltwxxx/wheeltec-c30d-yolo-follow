import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('follow_bringup')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'follow_bridge_params.yaml')),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('start_sensors', default_value='true'),
        DeclareLaunchArgument('start_actuator', default_value='true'),
        Node(
            package='follow_bridges', executable='sensor_bridge', namespace=namespace,
            parameters=[params_file], output='screen',
            condition=IfCondition(LaunchConfiguration('start_sensors'))),
        Node(
            package='follow_bridges', executable='actuator_bridge', namespace=namespace,
            parameters=[params_file], output='screen',
            condition=IfCondition(LaunchConfiguration('start_actuator'))),
    ])
