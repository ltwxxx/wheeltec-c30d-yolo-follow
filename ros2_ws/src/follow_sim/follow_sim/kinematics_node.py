"""差速小车运动学仿真：用话题名伪装成真车。

sim-to-real 的全部机制只有一句话 —— 仿真节点发布真车发布的、订阅真车订阅的，
名字和类型都对齐（契约见 docs/architecture.md 与 src/follow_actuator.py:32,44,45），
于是 src/ 里的真实代码一行不改就能跑在桌面上。

跑法（先别装 Gazebo，这个零新依赖）：
    ros2 run follow_sim sim_node
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
    ros2 topic echo /odom
"""

import math
import random
import threading
import time

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

BEAMS = 360                  # 360 束 × 0.01745rad ≈ 6.28rad，才能过 follow_sensors.py:114 的"整圈扫描"判据
TAU = 2.0 * math.pi


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('follow_sim')

        # 硬保险：真车底盘在发 /PowerVoltage 时，仿真绝不启动。
        # 两条 /odom、两条 /PowerVoltage 同时活着，比没仿真危险得多 —— 而且不会报错，只会骗人。
        # 位置很关键：必须在创建自己的发布器**之前**查。count_publishers() 把自己也算进去，
        # 上一版把检查放在 main() 里，仿真把你自己当成了真车，永远拒绝启动（实测踩过）。
        if self.count_publishers('/PowerVoltage') > 0:
            raise RuntimeError('检测到真底盘在发 /PowerVoltage，仿真拒绝启动（避免与真实数据抢话题）')

        # 参数用 declare + set_parameters_callback，运行时可以 ros2 param set 挪人，
        # 不用重启 —— 这就是阶段 B 要在真代码里补的那件事，仿真先体验一遍。
        self.declare_parameter('person_x', 2.5)
        self.declare_parameter('person_y', 0.0)
        self.declare_parameter('person_radius', 0.35)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('noise_m', 0.01)
        self._person = [self.get_parameter('person_x').value,
                        self.get_parameter('person_y').value,
                        self.get_parameter('person_radius').value]
        self.add_on_set_parameters_callback(self._on_param)

        # 真车契约：只订 cmd_vel，只发 odom / scan / PowerVoltage
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_volt = self.create_publisher(Float32, '/PowerVoltage', 10)
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 10)

        self._lock = threading.Lock()
        self._v = 0.0
        self._w = 0.0
        self._cmd_at = 0.0
        self._x = self._y = self._yaw = 0.0
        self._volt = 12.4

        self.create_timer(0.05, self._integrate)   # 20Hz 位姿
        self.create_timer(0.10, self._publish_scan)  # 10Hz 雷达，和真车同频
        self.create_timer(1.0, self._publish_voltage)
        self.get_logger().info('FOLLOW_SIM_READY (虚拟车，不是真车)')

    def _on_param(self, params):
        for p in params:
            if p.name in ('person_x', 'person_y', 'person_radius'):
                idx = {'person_x': 0, 'person_y': 1, 'person_radius': 2}[p.name]
                self._person[idx] = p.value
                self.get_logger().info('人挪到 %.2f, %.2f (r=%.2f)' % tuple(self._person))
        # 回调必须返回 SetParametersResult，返回 None 或参数列表会让 set 失败
        return SetParametersResult(successful=True)

    def _on_cmd(self, msg):
        with self._lock:
            # 限速交给真代码里的安全层做，仿真只负责"照命令动"，
            # 否则仿真会替安全层兜底，实车上的漏洞就被掩盖了 —— 这是仿真最容易骗人的地方。
            self._v = msg.linear.x
            self._w = msg.angular.z
            self._cmd_at = time.monotonic()

    def _integrate(self):
        with self._lock:
            if time.monotonic() - self._cmd_at > self.get_parameter('cmd_timeout_s').value:
                self._v = self._w = 0.0          # 命令过期就停：和真车看门狗同语义
            v, w = self._v, self._w
            dt = 0.05
            self._x += v * math.cos(self._yaw) * dt
            self._y += v * math.sin(self._yaw) * dt
            self._yaw = (self._yaw + w * dt + math.pi) % TAU - math.pi
            x, y, yaw = self._x, self._y, self._yaw
            v, w = self._v, self._w

        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        m.twist.twist.linear.x, m.twist.twist.angular.z = v, w
        self.pub_odom.publish(m)

        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = m.pose.pose.orientation.z, m.pose.pose.orientation.w
        # TransformBroadcaster 底层就是往 /tf 发这一条 TFMessage；
        # 直接发消息可以少掉整个 tf2_ros Python 绑定（本镜像里它只有 C++ 头文件，没有 .py）。
        tf_msg = TFMessage()
        tf_msg.transforms = [t]
        self.pub_tf.publish(tf_msg)

    def _publish_voltage(self):
        # 边跑边掉电，且负载越大掉越快 —— 为的是能测真车那段电压锁逻辑
        with self._lock:
            v = abs(self._v)
        self._volt = max(9.0, self._volt - 0.004 - 0.02 * v)
        msg = Float32()
        msg.data = self._volt
        self.pub_volt.publish(msg)

    def _publish_scan(self):
        with self._lock:
            x, y, yaw = self._x, self._y, self._yaw
        px, py, pr = self._person
        noise = self.get_parameter('noise_m').value
        dx, dy = px - x, py - y
        rng = random.Random()

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = 0.0
        scan.angle_max = TAU
        scan.angle_increment = TAU / BEAMS
        scan.scan_time = 0.1
        scan.range_min = 0.12
        scan.range_max = 12.0
        ranges = []
        for i in range(BEAMS):
            phi = yaw + i * scan.angle_increment
            ux, uy = math.cos(phi), math.sin(phi)
            along = dx * ux + dy * uy
            if along <= 0.0 or along > scan.range_max:
                ranges.append(float('inf'))      # 背后/超距：给 inf，让真代码自己过滤
                continue
            perp = math.hypot(dx - along * ux, dy - along * uy)
            if perp >= pr:
                ranges.append(float('inf'))      # 没打中人：真空，不是"很远的一个数"
                continue
            hit = along - math.sqrt(pr * pr - perp * perp)
            ranges.append(max(scan.range_min, hit + rng.gauss(0.0, noise)))
        scan.ranges = ranges
        scan.intensities = []
        self.pub_scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KinematicsNode()
    except RuntimeError as exc:
        get_logger('follow_sim').error(str(exc))
        rclpy.try_shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
