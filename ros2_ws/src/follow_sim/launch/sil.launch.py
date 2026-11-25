from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        Node(package='follow_sim', executable='sim_node', namespace=namespace, output='screen'),
        Node(package='follow_bridges', executable='actuator_bridge', namespace=namespace,
             parameters=[{'battery_voltage_topic': '/PowerVoltage',
                          'odometry_topic': '/odom', 'command_topic': '/cmd_vel'}],
             output='screen'),
    ])
