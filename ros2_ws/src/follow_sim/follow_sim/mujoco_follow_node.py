"""跟随节点：只吃话题（/scan + /camera/image_raw），只吐 /cmd_vel。

这是第 6、7 课那套“找人 + 保持距离 + 绕障”搬到 ROS 上的版本。
节点自己不认识 MuJoCo，也不认识车——换个真车，只要话题一样，代码一行不用改。

Run（容器内、仓库根目录）:
  ros2 run follow_sim mujoco_follow   # 需要 mujoco_world 先跑起来

订阅：/scan（LaserScan）、/camera/image_raw（rgb8）
发布：/cmd_vel（Twist）、/follow_state（std_msgs/String，一行 JSON）
"""

import json
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

TARGET = 0.45              # 想保持的间距（米）
KP_SPEED = 3.0             # 距离每差 1 米，补多少轮速
MAX_SPEED = 6.0            # 前进/后退的轮速上限（rad/s）
DEADBAND_DIST = 0.05       # 间距误差小于它就不动（防抖动）
PIXEL_K = 110.0            # 像素数换算距离的标定系数

KP_TURN = 2.0
MAX_TURN = 3.0             # 轮速差上限（rad/s）
DEADBAND_TURN = math.radians(3)
SEARCH_TURN = 2.0          # 找不到人时的原地搜索转速

FAN_HALF = 50              # 用到的激光范围 ±50°
FRONT_HALF = 15            # ±15° 以内算“正前方”
SIDE_IN = 30               # 30°~50° 这段算“侧面”

OBS_MARGIN = 0.45
OBS_RANGE = 1.3
OBS_MIN_EST = 1.0
OBS_TIE = 0.15
SIDE_SAFE = 0.30
CREEP_SPEED = 1.0
STOP_CLEAR = 0.35

MIN_PIXELS = 20            # 少于这么多橙色像素就当没看见


def find_person(img, fy):
    """在摄像头画面里找橙色的人。返回 (方位弧度, 像素数)；看不见时方位为 None。"""
    h, w = img.shape[0], img.shape[1]
    r = img[..., 0].astype(int)
    g = img[..., 1].astype(int)
    mask = (r > 90) & (r - g > 35)
    xs = np.nonzero(mask)[1]
    if xs.size < MIN_PIXELS:
        return None, xs.size
    return -math.atan((xs.mean() - w / 2) / fy), xs.size


def nearest(rays, lo, hi):
    """扇形里 [lo, hi] 这段角度上最近的一根，返回 (角度, 距离)；全打空返回 (0, -1)。"""
    best = None
    for deg, dist in rays:
        if lo <= deg <= hi and dist >= 0.0 and (best is None or dist < best[1]):
            best = (deg, dist)
    return best if best else (0.0, -1.0)


def control(bearing, front_angle, front, left, right, pixels, cutoff):
    """返回 (状态, 前进量, 转向量, 间距, 是否在绕障)。逻辑与第 7 课完全一致。"""
    if bearing is None:
        return "找人", 0.0, SEARCH_TURN, front, False

    est = PIXEL_K / math.sqrt(pixels)
    front_hit = 0.0 <= front < cutoff

    person_ahead = front_hit and abs(front - est) <= OBS_MARGIN
    blocked = (front_hit and not person_ahead
               and front < OBS_RANGE and est > OBS_MIN_EST)
    near_left = 0.0 <= left < SIDE_SAFE
    near_right = 0.0 <= right < SIDE_SAFE

    if blocked or near_left or near_right:
        if blocked:
            left_clear = left if 0.0 <= left < cutoff else cutoff
            right_clear = right if 0.0 <= right < cutoff else cutoff
            if abs(front_angle) >= 4.0:
                away = -1.0 if front_angle > 0 else 1.0
            elif left_clear > right_clear + OBS_TIE:
                away = 1.0
            elif right_clear > left_clear + OBS_TIE:
                away = -1.0
            else:
                away = 1.0 if bearing >= 0 else -1.0
            speed = 0.0
            if front >= STOP_CLEAR:
                speed = CREEP_SPEED * (front - STOP_CLEAR) / (OBS_RANGE - STOP_CLEAR)
        else:
            if near_left and near_right:
                away = -1.0 if left > right else 1.0
            else:
                away = -1.0 if near_left else 1.0
            near = min(left if near_left else 9.9, right if near_right else 9.9)
            speed = 0.0 if near < 0.15 else CREEP_SPEED * 0.5
        return "绕障", speed, away * MAX_TURN, front, True

    gap = front if front_hit else est
    err = gap - TARGET
    if err > DEADBAND_DIST:
        state = "靠近" if front_hit else "往前探"
    elif err < -DEADBAND_DIST:
        if front_hit:
            state = "后退"
        else:
            state, err = "保持", 0.0
    else:
        state, err = "保持", 0.0

    speed = max(-MAX_SPEED, min(MAX_SPEED, KP_SPEED * err))
    speed *= max(0.0, math.cos(bearing))

    turn = 0.0
    if abs(bearing) > DEADBAND_TURN:
        turn = max(-MAX_TURN, min(MAX_TURN, KP_TURN * bearing))
    return state, speed, turn, gap, False


def show(value, cutoff):
    """激光读数打印：打空就显示 --。"""
    return "  -- " if not (0.0 <= value < cutoff) else "%5.2f" % value


class MujocoFollow(Node):

    def __init__(self):
        super().__init__('mujoco_follow')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('state_topic', '/follow_state')
        self.declare_parameter('control_hz', 30.0)
        self.declare_parameter('fov_v_deg', 65.0)        # 和 car.xml 里 eye 相机的 fovy 一致
        self.declare_parameter('laser_cutoff', 6.0)      # 超过它当“打空”
        self.declare_parameter('data_timeout_s', 0.5)    # 数据太旧就先停车
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('half_track', 0.15)

        self.cutoff = self.get_parameter('laser_cutoff').value
        self.data_timeout = self.get_parameter('data_timeout_s').value
        self.fov_v = math.radians(self.get_parameter('fov_v_deg').value)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.half_track = self.get_parameter('half_track').value

        self._rays = []            # [(角度, 距离)]，角度已转成相对车头
        self._scan_at = 0.0
        self._image = None
        self._image_at = 0.0
        self._fy = None

        self.pub_cmd = self.create_publisher(Twist, self.get_parameter('command_topic').value, 10)
        self.pub_state = self.create_publisher(String, self.get_parameter('state_topic').value, 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._on_image, 10)

        self._last_report = 0.0
        self.create_timer(1.0 / self.get_parameter('control_hz').value, self._tick)
        self.get_logger().info('MUJOCO_FOLLOW_READY 等 /scan 和 /camera/image_raw')

    def _on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        center = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        rays = []
        for deg in range(-FAN_HALF, FAN_HALF + 1):
            idx = center + int(round(math.radians(deg) / msg.angle_increment))
            if not (0 <= idx < n):
                continue
            d = msg.ranges[idx]
            if math.isinf(d) or math.isnan(d) or not (msg.range_min <= d < min(self.cutoff, msg.range_max)):
                d = -1.0        # 打空：和仿真课里的约定一样
            rays.append((deg, d))
        self._rays = rays
        self._scan_at = time.monotonic()

    def _on_image(self, msg):
        if msg.encoding != 'rgb8':
            self.get_logger().warn('只认 rgb8，收到的是 %s' % msg.encoding, once=True)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8)
        if img.size != msg.height * msg.width * 3:
            return
        self._image = img.reshape(msg.height, msg.width, 3)
        self._image_at = time.monotonic()
        self._fy = (msg.height / 2.0) / math.tan(self.fov_v / 2.0)

    def _stop(self, state):
        """数据不全或太旧：发零速度，别让世界节点按旧命令继续跑。"""
        self.pub_cmd.publish(Twist())
        self._publish_state(state, None, 0, (0.0, -1.0), -1.0, -1.0, 0.0, 0.0)

    def _tick(self):
        now = time.monotonic()
        if not self._rays or self._image is None:
            self._stop('等数据')
            return
        if now - self._scan_at > self.data_timeout or now - self._image_at > self.data_timeout:
            self._stop('数据过期')
            return

        front_angle, front = nearest(self._rays, -FRONT_HALF, FRONT_HALF)
        left = nearest(self._rays, SIDE_IN, FAN_HALF)[1]
        right = nearest(self._rays, -FAN_HALF, -SIDE_IN)[1]

        bearing, pixels = find_person(self._image, self._fy)
        state, speed, turn, gap, _ = control(
            bearing, front_angle, front, left, right, pixels, self.cutoff)

        left_wheel = speed - turn
        right_wheel = speed + turn
        cmd = Twist()
        cmd.linear.x = (left_wheel + right_wheel) / 2.0 * self.wheel_radius
        cmd.angular.z = turn * self.wheel_radius / self.half_track
        self.pub_cmd.publish(cmd)

        self._publish_state(state, bearing, pixels, (front_angle, front), left, right,
                            left_wheel, right_wheel)

        if now - self._last_report >= 1.0:
            where = "看不见" if bearing is None else "%+4.0f°" % math.degrees(bearing)
            self.get_logger().info(
                "看见 %4d px  方位 %-5s  前 %s  左 %s  右 %s  %-6s  间距 %s  轮速 左%+.1f 右%+.1f"
                % (pixels, where, show(front, self.cutoff), show(left, self.cutoff),
                   show(right, self.cutoff), state, show(gap, self.cutoff),
                   left_wheel, right_wheel))
            self._last_report = now

    def _publish_state(self, state, bearing, pixels, front, left, right, lw, rw):
        msg = String()
        msg.data = json.dumps({
            'state': state,
            'bearing_deg': None if bearing is None else round(math.degrees(bearing), 1),
            'pixels': int(pixels),
            'front': None if front[1] < 0.0 else round(front[1], 3),
            'front_deg': front[0],
            'left': None if left < 0.0 else round(left, 3),
            'right': None if right < 0.0 else round(right, 3),
            'left_wheel': round(lw, 2),
            'right_wheel': round(rw, 2),
        }, ensure_ascii=False)
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoFollow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch 会把同一个 SIGINT 再转发一次，退出过程中再被打断会半途崩掉
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.pub_cmd.publish(Twist())     # 退出前最后一条命令必须是停
        except Exception:
            pass                              # 信号处理已关掉上下文时发不出去，正常
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
