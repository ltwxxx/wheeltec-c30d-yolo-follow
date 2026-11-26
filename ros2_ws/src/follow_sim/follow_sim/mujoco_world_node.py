"""MuJoCo 世界节点：把 sim/car.xml 变成一个会说 ROS 话题的机器人。

Run（容器内、仓库根目录）:
  export MUJOCO_GL=osmesa
  ros2 run follow_sim mujoco_world

话题契约和真车一致：订阅 /cmd_vel，发布 /odom、/tf、/scan、/camera/image_raw。
区别是这里的每个数都是物理世界算出来的：
  /scan  360 根射线真的打在 MuJoCo 场景上（树干、石头、灌木、人都挡得住）；
  /camera/image_raw  车头摄像头真的渲染出来的画面；
  /odom  车身在物理世界里的真实位姿，不是积分出来的。

挪人：ros2 param set /mujoco_world person_x 3.0 或往 /sim/person 发一个 Point。
"""

import math
import signal
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage

try:
    import mujoco
except ImportError as exc:      # 真车没装 MuJoCo，这个开发用节点不该拖垮别的包
    raise SystemExit('mujoco_world 需要 MuJoCo：pip3 install mujoco（%s）' % exc)

BEAMS = 360                     # 一整圈，和真车雷达一样
PERSON_Z = 0.61
CAR_Z = 0.09


def clamp_person(data, per_q, per_v):
    """人只能贴地站着：贴地、站直、不留速度（和仿真课里的钳制一样）。"""
    data.qpos[per_q + 2] = PERSON_Z
    data.qpos[per_q + 3:per_q + 7] = (1.0, 0.0, 0.0, 0.0)
    data.qvel[per_v:per_v + 6] = 0.0


def clamp_car(data, car_q, car_v):
    """车只能在地面上：保留位置和朝向，扶正、贴地。"""
    w, x, y, z = data.qpos[car_q + 3:car_q + 7]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    data.qpos[car_q + 2] = CAR_Z
    data.qpos[car_q + 3] = math.cos(yaw / 2)
    data.qpos[car_q + 4:car_q + 6] = (0.0, 0.0)
    data.qpos[car_q + 6] = math.sin(yaw / 2)
    data.qvel[car_v + 2] = 0.0
    data.qvel[car_v + 3:car_v + 5] = 0.0


class MujocoWorld(Node):

    def __init__(self):
        super().__init__('mujoco_world')

        self.declare_parameter('world_file', 'sim/car.xml')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('person_topic', '/sim/person')
        self.declare_parameter('person_x', 2.0)
        self.declare_parameter('person_y', 0.0)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('scan_every', 3)      # 每 3 帧一次 → 10 Hz，和真车同频
        self.declare_parameter('image_hz', 10.0)     # 渲染单独一条线程，不拖物理
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('half_track', 0.15)
        self.declare_parameter('camera_width', 320)
        self.declare_parameter('camera_height', 240)

        world_file = self.get_parameter('world_file').value
        self.model = mujoco.MjModel.from_xml_path(world_file)
        self.data = mujoco.MjData(self.model)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.half_track = self.get_parameter('half_track').value
        self.cmd_timeout = self.get_parameter('cmd_timeout_s').value
        self.scan_every = int(self.get_parameter('scan_every').value)
        self.image_period = 1.0 / self.get_parameter('image_hz').value
        rate = self.get_parameter('rate_hz').value
        self.substeps = max(1, round((1.0 / rate) / self.model.opt.timestep))

        self.lidar_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'lidar')
        self.chassis = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
        if self.lidar_site < 0 or self.chassis < 0:
            raise RuntimeError('car.xml 里缺 lidar site 或 chassis body，世界文件不对？')
        self._car_bodies = {self.chassis} | {
            b for b in range(self.model.nbody)
            if self.model.body(b).name.startswith('wheel')}
        self._hits = 0
        self._hit_logged = 0.0

        car_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
        self.car_q = self.model.jnt_qposadr[car_joint]
        self.car_v = self.model.jnt_dofadr[car_joint]
        per_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'person_root')
        self.per_q = self.model.jnt_qposadr[per_joint]
        self.per_v = self.model.jnt_dofadr[per_joint]

        self.data.qpos[self.car_q:self.car_q + 3] = (0.0, 0.0, CAR_Z)
        self.data.qpos[self.car_q + 3:self.car_q + 7] = (1.0, 0.0, 0.0, 0.0)
        self._place_person(float(self.get_parameter('person_x').value),
                           float(self.get_parameter('person_y').value))
        mujoco.mj_forward(self.model, self.data)

        # 渲染吃的是 data 的快照：软件渲染（容器里没有 GPU）一帧要上百毫秒，
        # 放进物理循环里会把它拖成半速，所以单开一条线程慢慢画。
        self._snapshot = mujoco.MjData(self.model)
        self._view = mujoco.MjData(self.model)
        mujoco.mj_copyData(self._snapshot, self.model, self.data)

        self._lock = threading.Lock()
        self._v = 0.0
        self._w = 0.0
        self._cmd_at = 0.0
        self._tick_count = 0
        self._shutdown = threading.Event()

        self.pub_odom = self.create_publisher(Odometry, self.get_parameter('odometry_topic').value, 10)
        self.pub_scan = self.create_publisher(LaserScan, self.get_parameter('scan_topic').value, 10)
        self.pub_image = self.create_publisher(Image, self.get_parameter('image_topic').value, 10)
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 10)
        self.create_subscription(Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd, 10)
        self.create_subscription(Point, self.get_parameter('person_topic').value, self._on_person, 10)
        self.add_on_set_parameters_callback(self._on_param)

        self.create_timer(1.0 / rate, self._tick)
        self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
        self._render_thread.start()
        self.get_logger().info(
            'MUJOCO_WORLD_READY world=%s 雷达 %d 束 / 摄像头 %dx%d'
            % (world_file, BEAMS,
               self.get_parameter('camera_width').value,
               self.get_parameter('camera_height').value))

    def _place_person(self, x, y):
        self.data.qpos[self.per_q:self.per_q + 3] = (x, y, PERSON_Z)
        self.data.qpos[self.per_q + 3:self.per_q + 7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qvel[self.per_v:self.per_v + 6] = 0.0

    def _on_person(self, msg):
        with self._lock:
            self._place_person(msg.x, msg.y)
        self.get_logger().info('人挪到 %.2f, %.2f' % (msg.x, msg.y))

    def _on_param(self, params):
        for p in params:
            if p.name in ('person_x', 'person_y'):
                with self._lock:
                    x = p.value if p.name == 'person_x' else self.data.qpos[self.per_q]
                    y = p.value if p.name == 'person_y' else self.data.qpos[self.per_q + 1]
                    self._place_person(float(x), float(y))
                self.get_logger().info('人挪到 %.2f, %.2f' % (x, y))
        return SetParametersResult(successful=True)

    def _on_cmd(self, msg):
        with self._lock:
            self._v = msg.linear.x
            self._w = msg.angular.z
            self._cmd_at = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            if now - self._cmd_at > self.cmd_timeout:
                self._v = self._w = 0.0     # 命令过期就停：和真车看门狗同语义
            v, w = self._v, self._w

            # 车体速度 → 四个轮子的目标转速：vl = (v - w·半轮距)/轮半径
            vl = (v - w * self.half_track) / self.wheel_radius
            vr = (v + w * self.half_track) / self.wheel_radius
            self.data.ctrl[:] = np.clip([vl, vr, vl, vr], -10.0, 10.0)

            for _ in range(self.substeps):
                mujoco.mj_step(self.model, self.data)
                self._count_hits()
            clamp_person(self.data, self.per_q, self.per_v)
            clamp_car(self.data, self.car_q, self.car_v)
            mujoco.mj_forward(self.model, self.data)
            # 给渲染线程留一份快照，物理接着往前走，不等它画完
            mujoco.mj_copyData(self._snapshot, self.model, self.data)

        self._publish_odom(v, w)
        self._tick_count += 1
        if self._tick_count % self.scan_every == 0:
            self._publish_scan()
        if self._hits and now - self._hit_logged > 1.0:
            self.get_logger().warn('碰撞接触步数 %d（车身/轮子碰到别的实体）' % self._hits)
            self._hit_logged = now

    def _count_hits(self):
        """数一数车身和轮子有没有碰到别的东西：绕障没撞上才有底气说“通过”。"""
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            b1 = int(self.model.geom_bodyid[c.geom1])
            b2 = int(self.model.geom_bodyid[c.geom2])
            in1, in2 = b1 in self._car_bodies, b2 in self._car_bodies
            if in1 == in2:
                continue                 # 都不是车（比如人和地面），或者都是车
            other = b2 if in1 else b1
            if other != 0:               # 0 号是世界（地面），压地面不算撞
                self._hits += 1
                return

    def _pose(self):
        q = self.data.qpos[self.car_q:self.car_q + 7]
        yaw = math.atan2(2 * (q[3] * q[6] + q[4] * q[5]), 1 - 2 * (q[5] ** 2 + q[6] ** 2))
        return float(q[0]), float(q[1]), yaw

    def _publish_odom(self, v, w):
        x, y, yaw = self._pose()
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        tf_msg = TFMessage()
        tf_msg.transforms = [t]
        self.pub_tf.publish(tf_msg)

    def _publish_scan(self):
        pos = self.data.site_xpos[self.lidar_site]
        z_axis = self.data.site_xmat[self.lidar_site].reshape(3, 3)[:, 2]
        heading = math.atan2(z_axis[1], z_axis[0])
        inc = 2.0 * math.pi / BEAMS
        ranges = []
        for i in range(BEAMS):
            a = heading - math.pi + i * inc
            vec = np.array([math.cos(a), math.sin(a), 0.0])
            d = mujoco.mj_ray(self.model, self.data, pos, vec, None, True,
                              self.chassis, None, None)
            ranges.append(d if d >= 0.12 else float('inf'))

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - inc
        scan.angle_increment = inc
        scan.scan_time = self.scan_every / self.get_parameter('rate_hz').value
        scan.range_min = 0.12
        scan.range_max = 12.0
        scan.ranges = ranges
        scan.intensities = []
        self.pub_scan.publish(scan)

    def _render_loop(self):
        """单独的渲染线程：拿到快照就画，画多快算多快，绝不让物理等它。"""
        width = self.get_parameter('camera_width').value
        height = self.get_parameter('camera_height').value
        renderer = mujoco.Renderer(self.model, height, width)
        # 激光束是窗口里的调试辅助线，真摄像头拍不到：渲染相机画面时关掉
        cam_opt = mujoco.MjvOption()
        cam_opt.flags[int(mujoco.mjtVisFlag.mjVIS_RANGEFINDER)] = 0
        try:
            while not self._shutdown.is_set():
                started = time.monotonic()
                with self._lock:
                    mujoco.mj_copyData(self._view, self.model, self._snapshot)
                renderer.update_scene(self._view, camera='eye', scene_option=cam_opt)
                img = renderer.render()        # (H, W, 3) uint8，rgb8

                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                msg.height = height
                msg.width = width
                msg.encoding = 'rgb8'
                msg.is_bigendian = 0
                msg.step = width * 3
                msg.data = img.tobytes()
                try:
                    self.pub_image.publish(msg)
                except Exception:
                    break                      # 上下文已关，收工
                self._shutdown.wait(max(0.0, self.image_period - (time.monotonic() - started)))
        finally:
            renderer.close()

    def stop(self):
        self._shutdown.set()
        self._render_thread.join(timeout=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoWorld()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch 会把同一个 SIGINT 再转发一次，退出过程中再被打断会带着渲染线程崩掉
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
