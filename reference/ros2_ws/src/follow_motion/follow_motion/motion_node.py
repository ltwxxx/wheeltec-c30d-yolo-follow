# 对应现在仓库：follow_motion.py（决策）+ follow_sensors.py 的 HTTP 收数部分
# 这是最值得读的一个文件：参数、DDS 订阅、QoS、超时看门狗、速度出口，全在一个节点里。
#
# ⚠️ 本文件只做过语法检查，未编译未运行。逻辑是从你现有实现里"换形态"搬过来的，
#    不是重新设计，所以别把它当结论；把它当"如果走 DDS 会长这样"。

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from follow_msgs.msg import TrackArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, LaserScan


class MotionNode(Node):
    def __init__(self):
        super().__init__('follow_motion')

        # ——— ① 参数：写死在代码里的阈值，到这儿变成"外部可调" ———
        # 每个 declare 都必须有一个默认值，这个默认值就是"yaml 丢了怎么办"的答案。
        self.declare_parameter('desired_distance_m', 1.20)
        self.declare_parameter('distance_deadband_m', 0.20)
        self.declare_parameter('max_forward_mps', 0.45)
        self.declare_parameter('max_yaw_rate_rps', 0.90)
        self.declare_parameter('center_tolerance_norm', 0.06)
        self.declare_parameter('lost_timeout_s', 2.5)
        self.declare_parameter('image_max_age_s', 0.35)
        self.declare_parameter('lidar_max_age_s', 0.50)

        # 参数对象要缓存成局部变量再读：get_parameter() 每次都要走一次 DDS 查询，
        # 放在 20Hz 循环里反复取是常见的性能坑。
        self.p = {k: self.get_parameter(k).value for k in (
            'desired_distance_m', 'distance_deadband_m', 'max_forward_mps',
            'max_yaw_rate_rps', 'center_tolerance_norm',
            'lost_timeout_s', 'image_max_age_s', 'lidar_max_age_s')}

        # 在线改参数时回调刷新缓存 —— 对应你现在"改 json 要不要重启"的痛点
        self.add_on_set_parameters_callback(self._on_param_change)

        # ——— ② 订阅：qos_profile_sensor_data = 尽量新、丢帧不补 ———
        # 传感器数据要的是"低延迟"而不是"不丢"，所以是 best-effort + 小队列。
        # 用默认的 reliable 订雷达，会出现"为了补一帧旧数据把车卡在原地"，这是新手最常踩的 QoS 坑。
        self.create_subscription(TrackArray, 'tracks', self._on_tracks, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)

        # ——— ③ 出口：话题名写 "cmd_vel"，真名字由 launch 的 remappings 决定 ———
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self._lock = threading.Lock()
        self._tracks = None
        self._tracks_ts = None
        self._scan_front_m = None
        self._scan_ts = None
        self._voltage_v = None

        # ——— ④ 定时器：对应你现在的 seq 看门狗，但由 ROS 时钟驱动 ———
        self.create_timer(0.05, self._decide)
        self.get_logger().info('FOLLOW_MOTION_READY')

    def _on_param_change(self, params):
        for p in params:
            self.p[p.name] = p.value
        return []

    def _on_tracks(self, msg):
        with self._lock:
            self._tracks = msg
            self._tracks_ts = self.get_clock().now()

    def _on_scan(self, msg):
        # 只取正前方一小段的最小值：这一步和你现在 lidar_geometry.py 里的做法是同一件事
        n = len(msg.ranges)
        if n == 0:
            return
        center = len(msg.ranges) // 2
        window = max(1, int(n * 0.06))
        seg = [r for r in msg.ranges[center - window:center + window]
               if msg.range_min < r < msg.range_max]
        with self._lock:
            self._scan_front_m = min(seg) if seg else None
            self._scan_ts = self.get_clock().now()

    def _on_battery(self, msg):
        with self._lock:
            self._voltage_v = msg.voltage

    def _stop(self, why):
        self.pub.publish(Twist())
        self.get_logger().warn('STOP: %s' % why)

    def _decide(self):
        with self._lock:
            tracks, tracks_ts = self._tracks, self._tracks_ts
            front, scan_ts = self._scan_front_m, self._scan_ts
            voltage = self._voltage_v

        now = self.get_clock().now()

        # 数据新鲜度检查 —— 三行分别对应你现在的"感知没数据/雷达没数据/电压没数据"
        if tracks is None or (now - tracks_ts).nanoseconds / 1e9 > self.p['image_max_age_s']:
            self._stop('感知数据超时')
            return
        if scan_ts is None or (now - scan_ts).nanoseconds / 1e9 > self.p['lidar_max_age_s']:
            self._stop('雷达数据超时')
            return
        if tracks.primary_track_id < 0:
            if (now - tracks_ts).nanoseconds / 1e9 > self.p['lost_timeout_s']:
                self._stop('目标丢失超时')
            return

        t = next((x for x in tracks.tracks if x.track_id == tracks.primary_track_id), None)
        if t is None:
            return

        cmd = Twist()

        # 纵向：想站到 desired_distance_m，误差进 deadband 就收手
        err = t.distance_m - self.p['desired_distance_m']
        if abs(err) > self.p['distance_deadband_m']:
            cmd.linear.x = max(-0.15, min(self.p['max_forward_mps'], err * 0.8))

        # 横向：靠框中心偏移转过去。这一步在你现在代码里是同一个公式，只是数据来自 dict
        off = t.center_x_ratio - 0.5
        if abs(off) > self.p['center_tolerance_norm']:
            cmd.angular.z = max(-self.p['max_yaw_rate_rps'],
                                min(self.p['max_yaw_rate_rps'], off * 1.6))

        # 只转向不前进：人在侧面时往前拱是危险的，也是观感最差的
        if cmd.linear.x > 0 and abs(cmd.angular.z) > 0.6:
            cmd.linear.x = 0.0

        # 安全兜底：这一层在业界通常仍交给独立安全节点/twist_mux（见 follow_safety），
        # 这里只示范"参数放松了也压不回硬上限"的写法。
        if front is not None and front < 0.6 and cmd.linear.x > 0:
            cmd.linear.x = 0.0
        if voltage is not None and voltage < 11.4 and cmd.linear.x > 0:
            cmd.linear.x = 0.0

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
