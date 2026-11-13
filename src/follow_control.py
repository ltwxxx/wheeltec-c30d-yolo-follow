"""Calculate a follow plan from a tracked person and time-aligned sensors."""

from __future__ import annotations

import base64
from dataclasses import dataclass
from http.client import HTTPConnection, HTTPException
from io import BytesIO
import json
import math
import threading
import time
from urllib.request import urlopen

import numpy as np
from PIL import Image
from follow_settings import SETTINGS


SENSOR_URL = "http://127.0.0.1:18767"
FRAME_TIMEOUT = 0.5
SCAN_TIMEOUT = 0.4
MIN_CLEARANCE_M = 0.45
ARRIVAL_DISTANCE_M = SETTINGS.stop_m
MAX_FORWARD_MPS = 0.10
AUTO_MAX_FORWARD_MPS = 0.25
AUTO_FAST_GAIN = 0.65
VISUAL_CLOSE_WIDTH = 0.38
VISUAL_CLOSE_HEIGHT = 0.90
VISUAL_SLOWDOWN_MARGIN = 0.35
NEAR_OBSTACLE_MM = int((SETTINGS.stop_m + (SETTINGS.camera_to_front_m or 0)) * 1000)
FAST_CLEARANCE_MM = 1000
FAST_DEPTH_RANGE_MM = 2000
MAX_TURN_RAD_S = 0.25
TURN_DEADBAND_RAD = math.radians(5)
FULL_FORWARD_ANGLE_RAD = math.radians(9)
TURN_ONLY_ANGLE_RAD = math.radians(12)
_frame_http = None
_frame_http_lock = threading.Lock()


@dataclass(frozen=True)
class LidarFrame:
    at: float
    points: np.ndarray
    front_m: float | None
    nearest_m: float | None = None
    clusters: tuple = ()


@dataclass(frozen=True)
class Sensors:
    depth_mm: np.ndarray | None
    focal_x: float
    center_x: float
    gap_ms: float | None
    lidar: LidarFrame | None = None

    @property
    def has_depth(self):
        return self.depth_mm is not None


@dataclass(frozen=True)
class Lidar:
    at: float
    closest_m: float | None


@dataclass(frozen=True)
class Observation:
    offset: float
    angle_rad: float
    distance_m: float | None
    width_ratio: float = 0.0
    height_ratio: float = 0.0
    lidar_distance_m: float | None = None
    lidar_quality: float = 0.0
    lidar_candidates_m: tuple = ()


@dataclass(frozen=True)
class Plan:
    reason: str
    forward_mps: float = 0.0
    turn_rad_s: float = 0.0
    blocked: bool = False


def decode_image(encoded):
    return Image.open(BytesIO(base64.b64decode(encoded, validate=True)))


def fetch_frame(*, as_pil=False, after_stamp=None, allow_color_only=False):
    started = time.monotonic()
    headers = {"Connection": "keep-alive"}
    if allow_color_only:
        headers["X-Allow-Color-Only"] = "1"
    if after_stamp is not None:
        headers["X-After-Frame"] = str(after_stamp)
    global _frame_http
    with _frame_http_lock:
        if _frame_http is None:
            _frame_http = HTTPConnection("127.0.0.1", 18767, timeout=0.4)
        try:
            _frame_http.request("GET", "/frame", headers=headers)
            response = _frame_http.getresponse()
            body = response.read(512_000)
            if response.status == 204 and after_stamp is not None:
                return None
            packet = json.loads(body)
            if response.status != 200:
                raise ValueError(packet.get("error", "相机数据桥返回错误"))
        except (OSError, HTTPException, ValueError):
            _frame_http.close()
            _frame_http = None
            raise
    if after_stamp is not None and packet["stamp"] == after_stamp:
        return None
    # The request start is a conservative clock reference even across hosts.
    age = packet.get("request_age", packet["age"])
    if type(age) not in (int, float) or not math.isfinite(age) or age < 0:
        raise ValueError("彩色和深度时间无效")
    captured_at = started - age
    if time.monotonic() - captured_at > FRAME_TIMEOUT:
        raise ValueError("彩色和深度数据超时")
    with decode_image(packet["color"]) as color:
        image = color.convert("RGB")
    depth = None
    if packet["depth"] is not None:
        with decode_image(packet["depth"]) as depth_image:
            if (depth_image.size != (160, 120)
                    or depth_image.mode not in ("I", "I;16", "I;16B", "I;16L")):
                raise ValueError("深度图像格式错误")
            depth = np.asarray(depth_image, dtype=np.uint16)
    elif not allow_color_only:
        raise ValueError("未收到深度画面")
    if not as_pil:
        image = np.asarray(image)[:, :, ::-1].copy()
    lidar = None
    if (scan := packet.get("lidar")) is not None:
        scan_age = scan["age"]
        points = np.asarray(scan["points"], dtype=float).reshape(-1, 3)
        if not math.isfinite(scan_age) or scan_age < 0 or not np.isfinite(points).all():
            raise ValueError("雷达数据无效")
        if len(points) > 4096 or (len(points) and (points[:, 2] <= 0).any()):
            raise ValueError("雷达点集无效")
        offset = SETTINGS.camera_to_front_m or 0
        front = scan["front_m"]
        nearest = scan.get("nearest_m", front)
        if any(value is not None and (type(value) not in (int, float) or not math.isfinite(value))
               for value in (front, nearest)):
            raise ValueError("雷达障碍距离无效")
        clusters = tuple(scan.get("clusters", ()))
        fields = ("u", "v", "u_min", "u_max", "distance_m", "width_m", "count", "spread_m")
        if len(clusters) > 4096 or any(not isinstance(c, dict) or any(
                type(c.get(key)) not in (int, float) or not math.isfinite(c[key]) for key in fields)
                or c["count"] < 3 or c["distance_m"] <= 0 or c["width_m"] < 0 or c["spread_m"] < 0 for c in clusters):
            raise ValueError("雷达候选回波无效")
        if front is not None and scan.get("front_reference") != "bumper":
            front -= offset
            nearest = nearest - offset if nearest is not None else None
        lidar = LidarFrame(started - scan_age, points, front,
                           nearest, clusters)
    if (type(packet["fx"]) not in (int, float) or not math.isfinite(packet["fx"]) or packet["fx"] <= 0
            or type(packet["cx"]) not in (int, float) or not 0 < packet["cx"] < 1):
        raise ValueError("相机内参无效")
    sensors = Sensors(depth, packet["fx"], packet["cx"], packet["gap_ms"], lidar)
    return packet["stamp"], image, captured_at, sensors


def fetch_lidar():
    started = time.monotonic()
    with urlopen(SENSOR_URL + "/scan", timeout=0.35) as response:
        packet = json.loads(response.read(2048))
    if packet["scan_age"] is None:
        return None
    return Lidar(started - packet["scan_age"], packet["closest"])


def lidar_candidates(box, sensors):
    if sensors.lidar is None:
        return []
    left, top, right, bottom = box
    width, center = right-left, (left+right)/2
    candidates = []
    for c in sensors.lidar.clusters:
        if (left <= c["u"] <= right and top <= c["v"] <= bottom
                and abs(c["u"] - center) < .40 * width and c["spread_m"] < .35
                and .025 <= c["width_m"] <= .85
                and .10 <= width * c["distance_m"] / sensors.focal_x <= .95
                and c["u_max"] - c["u_min"] <= width * 1.1):
            candidates.append((abs(c["u"] - center) / width, c["distance_m"]))
    return sorted(candidates)


def target_lidar_distance(box, sensors):
    if sensors.lidar is None or len(sensors.lidar.points) < 3:
        return None
    left, top, right, bottom = box
    width = right - left
    center = (left + right) / 2
    if sensors.lidar.clusters:
        candidates = lidar_candidates(box, sensors)
        if len(candidates) > 1 and abs(candidates[0][1] - candidates[1][1]) > .30:
            return None  # Ambiguous foreground/background: wait for an unambiguous observation.
        return candidates[0][1] if candidates else None
    points = sensors.lidar.points
    points = points[np.argsort(points[:, 0])]
    breaks = np.flatnonzero((np.diff(points[:, 0]) > 0.035) | (np.abs(np.diff(points[:, 2])) > 0.20)) + 1
    distances = []
    for cluster in np.split(points, breaks):
        u, v, z = cluster.T
        inside = (left <= u) & (u <= right) & (top <= v) & (v <= bottom)
        if np.count_nonzero(inside) < max(3, 0.65 * len(cluster)):
            continue
        if abs(float(np.median(u[inside])) - center) > 0.35 * width:
            continue
        distance = float(np.percentile(z[inside], 25))
        # A wall return far behind a large person box implies an implausibly wide person.
        if 0.10 <= width * distance / sensors.focal_x <= 0.95:
            distances.append(distance)
    return min(distances, default=None)


def fresh_lidar(sensors, now):
    if sensors is None or sensors.lidar is None or now - sensors.lidar.at > SCAN_TIMEOUT:
        return None
    return sensors.lidar


def observe_person(box, sensors, *, use_depth=True):
    left, top, right, bottom = box
    center = (left + right) / 2
    offset = 2 * center - 1
    if sensors is None:
        return Observation(offset, 0.0, None)
    angle = math.atan2(center - sensors.center_x, sensors.focal_x)
    lidar_distance = target_lidar_distance(box, sensors)
    candidates = tuple(distance for _, distance in lidar_candidates(box, sensors))
    def observation(distance=None):
        return Observation(offset, angle, distance, right - left, bottom - top, lidar_distance,
                           .8 if candidates else .5 if lidar_distance is not None else 0, candidates)
    # The center of the torso is less likely to include background than the full box.
    width, height = right - left, bottom - top
    if not use_depth:
        return observation()
    if not sensors.has_depth:
        return observation()
    # Vertical clipping can leave a clean torso sample. Side clipping biases the bearing.
    if width < 0.05 or height < 0.16 or left <= 0.01 or right >= 0.99:
        return observation()
    rows, columns = sensors.depth_mm.shape
    x1, x2 = (int((center + factor * width) * columns) for factor in (-0.2, 0.2))
    y1, y2 = (int((top + factor * height) * rows) for factor in (0.22, 0.55))
    region = sensors.depth_mm[y1:y2, x1:x2]
    valid = region[(region >= 350) & (region <= 6000)]
    if valid.size < max(12, region.size * 0.6):
        return observation()
    near, far = np.percentile(valid, [25, 75])
    distance = float(near) / 1000 if far - near <= 450 else None
    return observation(distance)


def front_corridor(sensors):
    depth = sensors.depth_mm
    rows, columns = depth.shape
    return depth[rows * 2 // 5:rows * 4 // 5, columns // 4:columns * 3 // 4]


def front_obstacle(corridor, clearance_mm):
    return np.count_nonzero((corridor > 0) & (corridor <= clearance_mm)) >= 8


def front_depth_covered(corridor, max_range_mm):
    valid = (corridor > 0) & (corridor <= max_range_mm)
    return all(sector.mean() >= 0.5 for sector in np.array_split(valid, 3, axis=1))


def trial_depth_block(sensors):
    """Check the visible front corridor before a limited onboard trial."""
    if sensors is None or not sensors.has_depth:
        return "等待新鲜的对齐深度"
    corridor = front_corridor(sensors)
    if front_obstacle(corridor, NEAR_OBSTACLE_MM):
        return "正前方有近处物体"
    if not front_depth_covered(corridor, 8000):
        return "正前方有效深度不足"
    return None


def plan_visual_follow(observation, sensors, now, *, fast_forward=False):
    """Steer from the person box, preferring matched lidar for approach distance."""
    lidar = fresh_lidar(sensors, now)
    distance = observation.lidar_distance_m if lidar is not None else None
    if distance is not None and distance <= ARRIVAL_DISTANCE_M:
        return Plan("已到人物附近（雷达）")
    if lidar is not None and lidar.front_m is not None and lidar.front_m <= ARRIVAL_DISTANCE_M:
        return Plan("雷达检测到前方近障")
    if sensors is not None and sensors.has_depth and front_obstacle(front_corridor(sensors), NEAR_OBSTACLE_MM):
        return Plan("正前方有近处物体")
    proximity = max(observation.width_ratio / VISUAL_CLOSE_WIDTH,
                    observation.height_ratio / VISUAL_CLOSE_HEIGHT)
    if distance is None and proximity >= 1:
        return Plan("人物已靠近（画面估计）")
    angle = observation.angle_rad
    bearing = abs(angle)
    turn = 0.0 if bearing <= TURN_DEADBAND_RAD else float(np.clip(-0.9 * angle, -0.16, 0.16))
    if bearing >= TURN_ONLY_ANGLE_RAD or abs(observation.offset) + observation.width_ratio >= 0.98:
        return Plan("先对准人物", turn_rad_s=turn)
    steering = min(1.0, (TURN_ONLY_ANGLE_RAD - bearing) / (TURN_ONLY_ANGLE_RAD - FULL_FORWARD_ANGLE_RAD))
    speed_limit = AUTO_MAX_FORWARD_MPS if fast_forward else MAX_FORWARD_MPS
    if distance is not None:
        forward = min(speed_limit, AUTO_FAST_GAIN * (distance - ARRIVAL_DISTANCE_M))
        return Plan("按雷达测距靠近", forward * steering, turn)
    approach = min(1.0, (1 - proximity) / VISUAL_SLOWDOWN_MARGIN)
    return Plan("按画面估计靠近", speed_limit * approach * steering, turn)


def plan_follow(observation, sensors, lidar, now, *, require_lidar=True,
                min_clearance_m=MIN_CLEARANCE_M, smooth_steering=False, fast_forward=False):
    if sensors is None or not sensors.has_depth:
        return Plan("对齐深度数据不可用", blocked=True)
    if require_lidar:
        if lidar is None or now - lidar.at > SCAN_TIMEOUT or lidar.closest_m is None:
            return Plan("雷达数据无效或超时", blocked=True)
        if lidar.closest_m < min_clearance_m:
            return Plan("雷达检测到近处物体", blocked=True)
    if observation.distance_m is None:
        return Plan("目标距离无法确认", blocked=True)
    if observation.distance_m <= ARRIVAL_DISTANCE_M:
        return Plan("已到人物附近")
    speed_limit = MAX_FORWARD_MPS
    gain = 0.25
    if smooth_steering:
        corridor = front_corridor(sensors)
        if front_obstacle(corridor, NEAR_OBSTACLE_MM):
            return Plan("正前方有近处物体")
        if (fast_forward and not front_obstacle(corridor, FAST_CLEARANCE_MM)
                and front_depth_covered(corridor, FAST_DEPTH_RANGE_MM)):
            speed_limit = AUTO_MAX_FORWARD_MPS
            gain = AUTO_FAST_GAIN
    angle = observation.angle_rad
    bearing = abs(angle)
    turn = float(np.clip(-0.9 * angle, -MAX_TURN_RAD_S, MAX_TURN_RAD_S))
    if bearing <= TURN_DEADBAND_RAD:
        turn = 0.0
    elif smooth_steering and bearing < FULL_FORWARD_ANGLE_RAD:
        turn *= (bearing - TURN_DEADBAND_RAD) / (FULL_FORWARD_ANGLE_RAD - TURN_DEADBAND_RAD)
    if bearing > TURN_ONLY_ANGLE_RAD:
        return Plan("先对准人物", turn_rad_s=turn)
    forward = min(speed_limit, gain * (observation.distance_m - ARRIVAL_DISTANCE_M))
    if smooth_steering and bearing > FULL_FORWARD_ANGLE_RAD:
        forward *= (TURN_ONLY_ANGLE_RAD - bearing) / (TURN_ONLY_ANGLE_RAD - FULL_FORWARD_ANGLE_RAD)
    reason = "快速靠近" if forward > MAX_FORWARD_MPS else "缓慢靠近" if forward else "保持距离"
    return Plan(reason, forward, turn)
