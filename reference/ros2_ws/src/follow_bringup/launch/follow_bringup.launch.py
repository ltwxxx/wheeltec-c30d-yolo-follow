# 对应现在仓库：ops/robot-follow-boot.sh
# 读的时候盯住三件事，那是 bash 脚本给不了的：
#   ① 谁接谁，写在文件里就能看出来（remappings 那一列）
#   ② 参数从 yaml 来，不在代码里写死
#   ③ 话题名/节点名能被外部改写，换话题不用改源码 —— 这正是阶段 A 实测过的安全开关
# 注意：systemd 该留。launch 管"怎么点起一栈节点"，systemd 管"开机就有、崩了重来"，
#       两者是分工不是替代 —— 你现在的 service 文件已经是正确答案。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('follow_bringup')


def generate_launch_description():
    return LaunchDescription([        # 所有可开关的东西一律先在这儿声明，Node 里只管引用。
        # 顺序很重要：参数必须在使用它的实体之前声明。
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(SHARE, 'config', 'follow_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('domain_namespace', default_value='',
                              description='多车同一网络时给话题加前缀，如 /robot2'),
        DeclareLaunchArgument('record', default_value='false',
                              description='true 时边跑边录 rosbag'),
        DeclareLaunchArgument('bag_dir', default_value='/data/follow_bags/latest'),

        # —— 安全层排第一个：语义是"下游依赖它"。
        # 真实项目里它通常另有独立 systemd unit，生命周期不跟 launch 绑，
        # 这样 launch 被 Ctrl-C 掉，车也绝不会变成无人驾驶状态。
        Node(
            package='follow_safety',
            executable='safety_node',
            name='follow_safety',
            namespace=LaunchConfiguration('domain_namespace'),
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('cmd_vel_out', '/cmd_vel')],
        ),
        Node(
            package='follow_perception',
            executable='perception_node',
            name='follow_perception',
            namespace=LaunchConfiguration('domain_namespace'),
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='follow_motion',
            executable='motion_node',
            name='follow_motion',
            namespace=LaunchConfiguration('domain_namespace'),
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            # 关键一行：节点内部只说 "cmd_vel"，出口话题由 launch 决定。
            # 想安全试跑就把这行改成 ('cmd_vel', '/codex_follow_cmd_vel')，
            # 真车那条 /cmd_vel 一个订阅者都不会多 —— 阶段 A 已经验证过这个手法。
            remappings=[('cmd_vel', '/follow/cmd_vel_raw')],
        ),
        Node(
            package='follow_session',
            executable='session_node',
            name='follow_session',
            namespace=LaunchConfiguration('domain_namespace'),
            parameters=[LaunchConfiguration('params_file')],
        ),

        # 录 bag：对应你现在靠 assets/samples + validation/*.json 手工留证据。
        # 差别是 bag 能整段重演，rviz/rosbag play 回放时不用再去现场。
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '--topics', '/follow/tracks', '/scan', '/odom',
                 '/battery_voltage', '/follow/cmd_vel_raw',
                 '--output', LaunchConfiguration('bag_dir')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('record')),
        ),
    ])
