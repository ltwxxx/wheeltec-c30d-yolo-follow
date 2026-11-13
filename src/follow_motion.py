"""Stateful lidar-priority distance control, independent of HTTP and ROS."""
from dataclasses import dataclass
import math
from follow_control import (fresh_lidar, front_corridor, front_obstacle, VISUAL_CLOSE_HEIGHT,
                            VISUAL_CLOSE_WIDTH, TURN_DEADBAND_RAD, TURN_ONLY_ANGLE_RAD)
from follow_settings import SETTINGS


@dataclass
class Motion:
    state: str
    reason: str
    forward_mps: float = 0.0
    turn_rad_s: float = 0.0
    distance_m: float | None = None
    source: str = "none"
    limit_mps: float = 0.0
    limit_reason: str = "等待有效观测"
    quality: float = 0.0


class FollowMotion:
    def __init__(self, settings=SETTINGS):
        self.cfg = settings
        self.arrived = False
        self.obstructed = False
        self.clear_since = None
        self.previous_range = None
        self.range_hits = 0
        self.isolated_since = None
        self.isolated_sample_at = None

    def plan(self, obs, sensors, frame_at, now, speed, battery_limit):
        cfg = self.cfg
        if not cfg.geometry_verified or cfg.camera_to_front_m is None:
            return Motion("fault_paused", "等待安装位置标定")
        lidar = fresh_lidar(sensors, now)
        if lidar is None:
            self.clear_since = None
            self.previous_range = None
            self.range_hits = 0
            self.isolated_since = None
            self.isolated_sample_at = None
            return Motion("fault_paused", "雷达数据过期，等待恢复")
        offset = cfg.camera_to_front_m
        distance = None
        measured = obs.lidar_distance_m
        if measured is None and self.previous_range is not None and frame_at - self.previous_range[0] <= .35:
            plausible = [candidate for candidate in obs.lidar_candidates_m
                         if abs(candidate - offset - self.previous_range[1]) <= .20 + max(0, frame_at - self.previous_range[0])]
            if plausible and max(plausible)-min(plausible) <= .20:
                measured = min(plausible, key=lambda candidate: abs(candidate-offset-self.previous_range[1]))
        if measured is not None and obs.lidar_quality >= .5:
            raw = max(0., measured - offset)
            previous = self.previous_range
            if previous is None or frame_at - previous[0] > .5 or abs(raw - previous[1]) > .25 + max(0, frame_at - previous[0]):
                self.range_hits = 1
            elif frame_at > previous[0]:
                self.range_hits += 1
            self.previous_range = (frame_at, raw)
            if self.range_hits >= 2:
                distance = raw
        else:
            self.range_hits = 0
            if self.previous_range is not None and frame_at - self.previous_range[0] > .35:
                self.previous_range = None
        source = "lidar" if distance is not None else "visual"
        quality = obs.lidar_quality if distance is not None else 0.
        latency = min(.6, max(.1, now - frame_at + .1))
        margin = max(0, speed) * latency + max(0, speed)**2 / (2 * cfg.brake_deceleration_mps2)
        stop_threshold = cfg.stop_m + margin
        near = lidar.nearest_m is not None and lidar.nearest_m <= .12
        obstacle = lidar.front_m is not None and lidar.front_m <= stop_threshold
        isolated = lidar.nearest_m is not None and lidar.nearest_m <= stop_threshold
        if isolated:
            if self.isolated_since is None:
                self.isolated_since = now
                self.isolated_sample_at = lidar.at
            # A persistent single-beam chair leg is an obstacle too; one sample is not.
            if now - self.isolated_since >= .15 and lidar.at > self.isolated_sample_at:
                obstacle = True
        else:
            self.isolated_since = self.isolated_sample_at = None
        if sensors.has_depth:
            obstacle |= front_obstacle(front_corridor(sensors), int((stop_threshold + offset) * 1000))
        if distance is not None and distance <= stop_threshold:
            self.arrived = True
        elif distance is not None and distance >= cfg.resume_m:
            self.arrived = False
        if near:
            self.obstructed = True
            self.clear_since = None
            return Motion("waiting_corridor", "极近障碍，立即停车", distance_m=distance, source=source)
        if self.arrived and distance is not None:
            return Motion("arrived", "已到车头停车距离", distance_m=distance, source=source, quality=quality)
        if obstacle:
            self.obstructed = True
            self.clear_since = None
        elif self.obstructed:
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since >= cfg.obstacle_clear_s:
                self.obstructed = False
        if self.obstructed:
            return Motion("waiting_corridor", "等待前方通道连续清空 0.5 秒", distance_m=distance, source=source)
        proximity = max(obs.width_ratio / VISUAL_CLOSE_WIDTH, obs.height_ratio / VISUAL_CLOSE_HEIGHT)
        if distance is None and (proximity >= .85 or self.arrived):
            return Motion("waiting_person", "人物靠近，等待可信雷达测距", source="visual")
        cap = min(cfg.max_speed_mps, battery_limit)
        reason = "电量限速" if battery_limit < cfg.max_speed_mps else "车端速度上限"
        if not cfg.braking_verified and cap > cfg.fallback_speed_mps:
            cap, reason = cfg.fallback_speed_mps, "制动实测待完成"
        if distance is None:
            cap, reason = min(cap, cfg.fallback_speed_mps), "雷达关联待确认，画面辅助"
        else:
            available = max(0, distance - cfg.stop_m)
            brake_cap = cfg.brake_deceleration_mps2 * (
                math.sqrt(latency**2 + 2 * available / cfg.brake_deceleration_mps2) - latency)
            if brake_cap < cap:
                cap, reason = brake_cap, "感知延迟及制动距离"
            if distance < .60 and cap > cfg.approach_speed_mps:
                cap, reason = cfg.approach_speed_mps, "近距离缓慢靠近"
        bearing = abs(obs.angle_rad)
        turn = 0. if bearing <= TURN_DEADBAND_RAD else max(-cfg.max_turn_rad_s, min(cfg.max_turn_rad_s, -.9 * obs.angle_rad))
        steering = max(0., min(1., (TURN_ONLY_ANGLE_RAD - bearing) / math.radians(3)))
        if abs(obs.offset) + obs.width_ratio >= .98:
            steering = 0.
        forward = cap * steering
        if distance is not None:
            forward = min(forward, .65 * max(0., distance - cfg.stop_m))
        else:
            forward *= min(1., max(0., (.85 - proximity) / .35))
        return Motion("following", "雷达优先跟随" if distance is not None else "远距离画面辅助跟随",
                      forward, turn, distance, source, cap, reason, quality)
