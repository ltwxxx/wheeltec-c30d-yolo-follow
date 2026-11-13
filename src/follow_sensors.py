"""Read-only ROS sensor bridge for the Mac follow preview."""

import base64
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time
import socket

import cv2
cv2.setNumThreads(1)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from lidar_geometry import summarize_points
from follow_settings import VERSION


SCAN_FRAME_WINDOW_S = 0.20
FRONT_HALF_WIDTH_M = 0.23


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def aligned_pair(colors, depths, now):
    colors = [record for record in colors if now - record[0] <= 0.25]
    if not colors:
        raise ValueError("彩色相机数据超时")
    if not depths:
        raise ValueError("未收到深度画面")
    depths = [record for record in depths if now - record[0] <= 0.25]
    if not depths:
        raise ValueError("深度相机数据超时")
    geometry_matched = False
    for color_at, color in sorted(colors, key=lambda record: stamp_ns(record[1]), reverse=True):
        compatible = [(at, depth) for at, depth in depths
                      if depth.header.frame_id == color.header.frame_id
                      and (depth.width, depth.height) == (color.width, color.height)]
        if not compatible:
            continue
        geometry_matched = True
        color_stamp = stamp_ns(color)
        depth_at, depth = min(compatible, key=lambda record: abs(stamp_ns(record[1]) - color_stamp))
        gap_ns = abs(color_stamp - stamp_ns(depth))
        if gap_ns <= 60_000_000:
            return color_at, color, depth_at, depth, gap_ns / 1e9
    if not geometry_matched:
        raise ValueError("深度图未对齐彩色坐标")
    raise ValueError("彩色和深度帧时间未对齐")


def project_scan(scan, transform, info):
    """Project the laser plane into the camera's normalized image coordinates."""
    ranges = np.asarray(scan.ranges)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    beam_indices = np.flatnonzero(valid)
    angles = scan.angle_min + beam_indices * scan.angle_increment
    ranges = ranges[valid]
    points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles), np.zeros(len(ranges))))
    q, t = transform.rotation, transform.translation
    axis = np.array([q.x, q.y, q.z])
    points += 2 * np.cross(axis, np.cross(axis, points) + q.w * points)
    points += (t.x, t.y, t.z)
    return summarize_points(points, beam_indices, info.k[0] / info.width, info.k[4] / info.height,
                            info.k[2] / info.width, info.k[5] / info.height)


class Sensors(Node):
    def __init__(self):
        super().__init__("codex_follow_sensor_reader")
        self.lock = threading.Lock()
        self.colors = deque(maxlen=6)
        self.depths = deque(maxlen=8)
        self.camera_info = None
        self.scan = None
        self.transforms = Buffer()
        self.transform_listener = TransformListener(self.transforms, self)
        for kind, topic, callback in (
            (Image, "/camera/color/image_raw", self.on_color),
            (Image, "/camera/depth/image_raw", self.on_depth),
            (CameraInfo, "/camera/color/camera_info", self.on_camera_info),
            (LaserScan, "/scan", self.on_scan),
        ):
            self.create_subscription(kind, topic, callback, qos_profile_sensor_data)

    def on_color(self, message):
        with self.lock:
            if self.colors and stamp_ns(self.colors[-1][1]) == stamp_ns(message):
                return
            self.colors.append((time.monotonic(), message))

    def on_depth(self, message):
        with self.lock:
            if self.depths and stamp_ns(self.depths[-1][1]) == stamp_ns(message):
                return
            self.depths.append((time.monotonic(), message))

    def on_camera_info(self, message):
        with self.lock:
            self.camera_info = message

    def on_scan(self, scan):
        ranges = np.asarray(scan.ranges)
        valid = ranges[np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)]
        full_scan = len(ranges) >= 100 and abs(scan.angle_increment) * len(ranges) >= 6.0
        closest = float(valid.min()) if full_scan and len(valid) >= 10 else None
        with self.lock:
            if self.scan and stamp_ns(self.scan[2]) == stamp_ns(scan):
                return
            self.scan = (time.monotonic(), closest, scan)

    def scan_status(self):
        with self.lock:
            scan = self.scan
        if scan is None:
            return {"scan_age": None, "closest": None}
        at, closest, _ = scan
        return {"scan_age": time.monotonic() - at, "closest": closest}

    def projected_lidar(self, scan, color_at, info):
        if scan is None or scan[1] is None or abs(scan[0] - color_at) > SCAN_FRAME_WINDOW_S:
            return None
        at, _, message = scan
        try:
            transform = self.transforms.lookup_transform(info.header.frame_id, message.header.frame_id, Time())
        except TransformException:
            return None
        if info.height <= 0 or not all(math.isfinite(info.k[i]) for i in (4, 5)) or info.k[4] <= 0:
            return None
        points, front_m, nearest_m, clusters = project_scan(message, transform.transform, info)
        return {"age": time.monotonic() - at, "points": points, "front_m": front_m,
                "nearest_m": nearest_m, "clusters": clusters, "front_reference": "bumper",
                "sample_at": at, "version": VERSION}

    def health(self):
        now = time.monotonic()
        with self.lock:
            return {"version": VERSION,
                    "color_age_s": now - self.colors[-1][0] if self.colors else None,
                    "depth_age_s": now - self.depths[-1][0] if self.depths else None,
                    "scan_age_s": now - self.scan[0] if self.scan else None}

    def frame(self, *, requested_at=None, after_stamp=None, allow_color_only=False):
        with self.lock:
            colors, depths = list(self.colors), list(self.depths)
            info, scan = self.camera_info, self.scan
        now = time.monotonic()
        if allow_color_only:
            fresh_colors = [record for record in colors if now - record[0] <= 0.25]
            if not fresh_colors:
                raise ValueError("彩色相机数据超时")
            color_at, color = max(fresh_colors, key=lambda record: stamp_ns(record[1]))
            depth_at, depth, gap = color_at, None, None
            compatible = [(at, item) for at, item in depths if now - at <= .25
                          and item.header.frame_id == color.header.frame_id
                          and (item.width, item.height) == (color.width, color.height)
                          and abs(stamp_ns(item) - stamp_ns(color)) <= 60_000_000]
            if compatible:
                depth_at, depth = min(compatible, key=lambda record: abs(stamp_ns(record[1])-stamp_ns(color)))
                gap = abs(stamp_ns(depth) - stamp_ns(color)) / 1e9
        else:
            color_at, color, depth_at, depth, gap = aligned_pair(colors, depths, now)
        if (info is None or info.header.frame_id != color.header.frame_id
                or info.width != color.width or not math.isfinite(info.k[0]) or info.k[0] <= 0
                or not math.isfinite(info.k[2]) or not 0 < info.k[2] < color.width):
            raise ValueError("缺少有效的彩色相机标定")
        if color.encoding != "rgb8":
            raise ValueError("图像格式不支持")
        if depth is not None and depth.encoding != "16UC1":
            if not allow_color_only:
                raise ValueError("图像格式不支持")
            depth_at, depth, gap = color_at, None, None
        stamp = stamp_ns(color)
        if after_stamp == str(stamp):
            return None

        rgb = np.ndarray((color.height, color.width, 3), dtype=np.uint8,
                         buffer=color.data, strides=(color.step, 3, 1))
        preview = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                             (320, 240), interpolation=cv2.INTER_AREA)
        color_ok, jpeg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not color_ok:
            raise ValueError("图像压缩失败")
        encoded_depth = None
        if depth is not None:
            depth_mm = np.ndarray((depth.height, depth.width),
                                  dtype=">u2" if depth.is_bigendian else "<u2",
                                  buffer=depth.data, strides=(depth.step, 2)).astype(np.uint16, copy=False)
            small_depth = cv2.resize(depth_mm, (160, 120), interpolation=cv2.INTER_NEAREST)
            depth_ok, png = cv2.imencode(".png", small_depth, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if not depth_ok and not allow_color_only:
                raise ValueError("图像压缩失败")
            if depth_ok:
                encoded_depth = base64.b64encode(png).decode("ascii")
        now = time.monotonic()
        received_at = color_at if allow_color_only else min(color_at, depth_at)
        return {
            "stamp": stamp,
            "age": now - received_at,
            "request_age": max(0.0, (requested_at if requested_at is not None else now) - received_at),
            "gap_ms": round(gap * 1000, 1) if encoded_depth is not None else None,
            "fx": info.k[0] / color.width,
            "cx": info.k[2] / color.width,
            "scan_age": now - scan[0] if scan is not None else None,
            "closest": scan[1] if scan is not None else None,
            "lidar": self.projected_lidar(scan, color_at, info) if allow_color_only else None,
            "color": base64.b64encode(jpeg).decode("ascii"),
            "depth": encoded_depth,
        }


def handler(sensors):
    class Request(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def setup(self):
            super().setup()
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def do_GET(self):
            requested_at = time.monotonic()
            if self.path not in ("/frame", "/scan", "/health"):
                self.send_error(404)
                return
            try:
                if self.path == "/frame":
                    data = sensors.frame(requested_at=requested_at,
                                         after_stamp=self.headers.get("X-After-Frame"),
                                         allow_color_only=self.headers.get("X-Allow-Color-Only") == "1")
                elif self.path == "/scan":
                    data = sensors.scan_status()
                else:
                    data = sensors.health()
                status = 200
            except (ValueError, cv2.error) as exc:
                data, status = {"error": str(exc)}, 503
            if data is None:
                self.send_response(204)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return
            payload = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_args):
            pass

    return Request


if __name__ == "__main__":
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    sensors = Sensors()
    with ThreadingHTTPServer(("127.0.0.1", 18767), handler(sensors)) as server:
        threading.Thread(target=rclpy.spin, args=(sensors,), daemon=True).start()
        print("READ_ONLY_SENSOR_BRIDGE_READY", flush=True)
        server.serve_forever()
