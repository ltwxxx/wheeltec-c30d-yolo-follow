"""Read-only ROS sensor bridge for the Mac follow preview."""

import base64
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image, LaserScan


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


class Sensors(Node):
    def __init__(self):
        super().__init__("codex_follow_sensor_reader")
        self.lock = threading.Lock()
        self.color = None
        self.depths = deque(maxlen=8)
        self.camera_info = None
        self.scan = None
        for kind, topic, callback in (
            (Image, "/camera/color/image_raw", self.on_color),
            (Image, "/camera/depth/image_raw", self.on_depth),
            (CameraInfo, "/camera/color/camera_info", self.on_camera_info),
            (LaserScan, "/scan", self.on_scan),
        ):
            self.create_subscription(kind, topic, callback, qos_profile_sensor_data)

    def on_color(self, message):
        with self.lock:
            self.color = (time.monotonic(), message)

    def on_depth(self, message):
        with self.lock:
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
            self.scan = (time.monotonic(), closest)

    def scan_status(self):
        with self.lock:
            scan = self.scan
        if scan is None:
            return {"scan_age": None, "closest": None}
        at, closest = scan
        return {"scan_age": time.monotonic() - at, "closest": closest}

    def frame(self):
        with self.lock:
            color_record, depths = self.color, list(self.depths)
            info, scan = self.camera_info, self.scan
        now = time.monotonic()
        if color_record is None or now - color_record[0] > 0.25:
            raise ValueError("彩色相机数据超时")
        color_at, color = color_record
        if not depths:
            raise ValueError("未收到深度画面")
        depth_at, depth = min(depths, key=lambda pair: abs(stamp_ns(pair[1]) - stamp_ns(color)))
        gap = abs(stamp_ns(color) - stamp_ns(depth)) / 1e9
        if gap > 0.06 or now - depth_at > 0.25:
            raise ValueError("彩色和深度帧时间未对齐")
        if (color.header.frame_id != depth.header.frame_id
                or (color.width, color.height) != (depth.width, depth.height)):
            raise ValueError("深度图未对齐彩色坐标")
        if (info is None or info.header.frame_id != color.header.frame_id
                or info.width != color.width or not math.isfinite(info.k[0]) or info.k[0] <= 0
                or not math.isfinite(info.k[2]) or not 0 < info.k[2] < color.width):
            raise ValueError("缺少有效的彩色相机标定")
        if color.encoding != "rgb8" or depth.encoding != "16UC1":
            raise ValueError("图像格式不支持")

        rgb = np.ndarray((color.height, color.width, 3), dtype=np.uint8,
                         buffer=color.data, strides=(color.step, 3, 1))
        depth_mm = np.ndarray((depth.height, depth.width),
                              dtype=">u2" if depth.is_bigendian else "<u2",
                              buffer=depth.data, strides=(depth.step, 2)).astype(np.uint16, copy=False)
        small_depth = cv2.resize(depth_mm, (160, 120), interpolation=cv2.INTER_NEAREST)
        preview = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                             (480, 360), interpolation=cv2.INTER_AREA)
        color_ok, jpeg = cv2.imencode(".jpg", preview,
                               
                                     [cv2.IMWRITE_JPEG_QUALITY, 50])
        depth_ok, png = cv2.imencode(".png", small_depth, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not color_ok or not depth_ok:
            raise ValueError("图像压缩失败")
        now = time.monotonic()
        return {
            "stamp": stamp_ns(color),
            "age": now - min(color_at, depth_at),
            "gap_ms": round(gap * 1000, 1),
            "fx": info.k[0] / color.width,
            "cx": info.k[2] / color.width,
            "scan_age": now - scan[0] if scan is not None else None,
            "closest": scan[1] if scan is not None else None,
            "color": base64.b64encode(jpeg).decode("ascii"),
            "depth": base64.b64encode(png).decode("ascii"),
        }


def handler(sensors):
    class Request(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self):
            if self.path not in ("/frame", "/scan"):
                self.send_error(404)
                return
            try:
                data = sensors.frame() if self.path == "/frame" else sensors.scan_status()
                status = 200
            except (ValueError, cv2.error) as exc:
                data, status = {"error": str(exc)}, 503
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
