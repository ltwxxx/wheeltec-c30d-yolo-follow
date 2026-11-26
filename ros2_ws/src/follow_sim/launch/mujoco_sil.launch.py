"""MuJoCo SIL：世界节点 + 跟随节点，话题契约和真车一致。

Run（容器内、仓库根目录）:
  export MUJOCO_GL=osmesa ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
  ros2 launch follow_sim mujoco_sil.launch.py
  ros2 launch follow_sim mujoco_sil.launch.py person_x:=3.4 person_y:=1.4

world_file 默认自动找仓库根目录的 sim/car.xml；在别处启动时用绝对路径传参。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _repo_sim_xml():
    """包安装在 ros2_ws/install 下时，往上一层层找到仓库根目录的 sim/car.xml。"""
    p = os.path.dirname(get_package_share_directory('follow_sim'))
    for _ in range(6):
        candidate = os.path.join(p, 'sim', 'car.xml')
        if os.path.isfile(candidate):
            return candidate
        p = os.path.dirname(p)
    return 'sim/car.xml'


def _nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    return [
        Node(package='follow_sim', executable='mujoco_world', output='screen',
             parameters=[{'world_file': value('world_file') or _repo_sim_xml(),
                          'person_x': float(value('person_x')),
                          'person_y': float(value('person_y'))}]),
        Node(package='follow_sim', executable='mujoco_follow', output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value='',
                              description='空 = 自动找仓库根目录的 sim/car.xml'),
        DeclareLaunchArgument('person_x', default_value='2.0'),
        DeclareLaunchArgument('person_y', default_value='0.0'),
        OpaqueFunction(function=_nodes),
    ])
