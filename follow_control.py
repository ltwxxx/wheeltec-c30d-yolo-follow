"""Calculate a follow plan from a tracked person and time-aligned sensors."""

from __future__ import annotations

import base64
from dataclasses import dataclass
from http.client import HTTPConnection, HTTPException
import json
import math
import threading
import time
from urllib.request import urlopen

import cv2
import numpy as np


SENSOR_URL = "http://127.0.0.1:18767"
FRAME_TIMEOUT = 0.5
SCAN_TIMEOUT = 0.4
MIN_CLEARANCE_M = 0.45
FOLLOW_DISTANCE_M = 1.3
ARRIVAL_DISTANCE_M = FOLLOW_DISTANCE_M + 0.12
MAX_FORWARD_MPS = 0.08
MAX_TURN_RAD_S = 0.25
_frame_http = None
_frame_http_lock = threading.Lock()


@dataclass(frozen=True)
class Sensors:
    depth_mm: np.ndarray
    focal_x: float
    center_x: float
    gap_ms: float


@dataclass(frozen=True)
class Lidar:
    at: float
    closest_m: float | None


@dataclass(frozen=True)
class Observation:
    offset: float
    angle_rad: float
    distance_m: float | None


@dataclass(frozen=True)
class Plan:
    reason: str
    forward_mps: float = 0.0
    turn_rad_s: float = 0.0
    blocked: bool = False


def decode_image(encoded, mode):
    data = np.frombuffer(base64.b64decode(encoded, validate=True), dtype=np.uint8)
    return cv2.imdecode(data, mode)


def fetch_frame():
    started = time.monotonic()
    global _frame_http
    with _frame_http_lock:
        if _frame_http is None:
            _frame_http = HTTPConnection("127.0.0.1", 18767, timeout=0.4)
        try:
            _frame_http.request("GET", "/frame", headers={"Connection": "keep-alive"})
            response = _frame_http.getresponse()
            body = response.read(512_000)
            if response.status != 200:
                raise ValueError("相机数据桥返回错误")
            packet = json.loads(body)
        except (OSError, HTTPException, ValueError):
            _frame_http.close()
            _frame_http = None
            raise
    captured_at = started - packet["age"]
    if time.monotonic() - captured_at > FRAME_TIMEOUT:
        raise ValueError("彩色和深度数据超时")
    image = decode_image(packet["color"], cv2.IMREAD_COLOR)
    depth = decode_image(packet["depth"], cv2.IMREAD_UNCHANGED)
    if image is None or depth is None or depth.shape != (120, 160) or depth.dtype != np.uint16:
        raise ValueError("图像格式错误")
    sensors = Sensors(depth, packet["fx"], packet["cx"], packet["gap_ms"])
    return packet["stamp"], image, captured_at, sensors


def fetch_lidar():
    started = time.monotonic()
    with urlopen(SENSOR_URL + "/scan", timeout=0.35) as response:
        packet = json.loads(response.read(2048))
    if packet["scan_age"] is None:
        return None
    return Lidar(started - packet["scan_age"], packet["closest"])


def observe_person(box, sensors):
    left, top, right, bottom = box
    center = (left + right) / 2
    offset = 2 * center - 1
    if sensors is None:
        return Observation(offset, 0.0, None)
    angle = math.atan2(center - sensors.center_x, sensors.focal_x)
    # The center of the torso is less likely to include background than the full box.
    width, height = right - left, bottom - top
    # Vertical clipping can leave a clean torso sample. Side clipping biases the bearing.
    if width < 0.05 or height < 0.16 or left <= 0.01 or right >= 0.99:
        return Observation(offset, angle, None)
    rows, columns = sensors.depth_mm.shape
    x1, x2 = (int((center + factor * width) * columns) for factor in (-0.2, 0.2))
    y1, y2 = (int((top + factor * height) * rows) for factor in (0.22, 0.55))
    region = sensors.depth_mm[y1:y2, x1:x2]
    valid = region[(region >= 350) & (region <= 6000)]
    if valid.size < max(12, region.size * 0.6):
        return Observation(offset, angle, None)
    near, far = np.percentile(valid, [25, 75])
    distance = float(near) / 1000 if far - near <= 450 else None
    return Observation(offset, angle, distance)


def plan_follow(observation, sensors, lidar, now, *, require_lidar=True,
                min_clearance_m=MIN_CLEARANCE_M):
    if sensors is None:
        return Plan("对齐深度数据不可用", blocked=True)
    if require_lidar:
        if lidar is None or now - lidar.at > SCAN_TIMEOUT or lidar.closest_m is None:
            return Plan("雷达数据无效或超时", blocked=True)
        if lidar.closest_m < min_clearance_m:
            return Plan("雷达检测到近处物体", blocked=True)
    if observation.distance_m is None:
        return Plan("目标距离无法确认", blocked=True)
    if observation.distance_m < 0.85:
        return Plan("目标过近")
    if observation.distance_m <= ARRIVAL_DISTANCE_M:
        return Plan("已到人物附近")
    angle = observation.angle_rad
    turn = float(np.clip(-0.9 * angle, -MAX_TURN_RAD_S, MAX_TURN_RAD_S))
    if abs(angle) <= math.radians(5):
        turn = 0.0
    if abs(angle) > math.radians(12):
        return Plan("先对准人物", turn_rad_s=turn)
    forward = min(MAX_FORWARD_MPS, 0.22 * max(0.0, observation.distance_m - ARRIVAL_DISTANCE_M))
    return Plan("缓慢靠近" if forward else "保持距离", forward, turn)
