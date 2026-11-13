"""Onboard session state. Only the control loop calls update; GET is read-only."""
from collections import deque
import json
import logging
import math
import secrets
from follow_control import FRAME_TIMEOUT
from follow_motion import FollowMotion
from follow_settings import SETTINGS, VERSION


class FollowSession:
    def __init__(self, settings=SETTINGS):
        self.cfg = settings
        self.events = deque(maxlen=100)
        self.motion = FollowMotion(settings)
        self.data = dict(target=None, automatic=False, unlocked=False, session_id=None,
                         state="idle", status="待机，等待人工启动", pause_reason=None,
                         offset=None, distance_m=None, forward_mps=0., turn_rad_s=0.,
                         source="none", limit_mps=0., limit_reason="待机", quality=0.,
                         sent_forward_mps=0., sent_turn_rad_s=0., version=VERSION)
        self.armed = False
        self.missing_since = None
        self.confirmation = None
        self.last_update = None
        self.deadline = None
        self.fault_latched = False

    def start(self, target, now, trial_seconds=None):
        if self.armed:
            raise ValueError("请先停止当前跟随")
        self.motion = FollowMotion(self.cfg)
        self.armed, self.fault_latched = True, False
        self.missing_since, self.confirmation = now, None
        self.deadline = now + trial_seconds if trial_seconds else None
        self.data.update(target=None if target in ("auto", "unlocked") else target,
                         automatic=target in ("auto", "unlocked"), unlocked=target == "unlocked",
                         session_id=secrets.token_hex(12), offset=None, distance_m=None)
        self._state("confirming", "正在确认人物", now)

    def stop(self, now, reason="人工停止"):
        self.armed = False
        self.deadline = None
        self.data.update(target=None, automatic=False, unlocked=False)
        self._state("manual_stop", reason, now)

    def _state(self, state, reason, now, **updates):
        previous = (self.data["state"], self.data["status"])
        self.data.update(state=state, status=reason,
                         pause_reason=reason if state in ("waiting_person", "waiting_corridor", "fault_paused", "confirming") else None,
                         forward_mps=0., turn_rad_s=0., planned_forward_mps=0., planned_turn_rad_s=0.,
                         distance_m=None, source="none", offset=None, quality=0.,
                         limit_mps=0., limit_reason=reason, **updates)
        if previous != (state, reason):
            event = dict(at=now, session_id=self.data["session_id"], state=state, reason=reason,
                         target=self.data["target"], sensors=self.data.get("sensor_ages", {}),
                         sent_forward_mps=self.data["sent_forward_mps"], sent_turn_rad_s=self.data["sent_turn_rad_s"])
            self.events.append(event)
            logging.getLogger("follow.events").warning(json.dumps(event, ensure_ascii=False))

    def _wait(self, state, reason, now):
        if self.missing_since is None:
            self.missing_since = now
        self.confirmation = None
        self._state(state, reason, now)

    def update(self, observations, sensors, frame_at, now, health, enabled=True):
        dt = min(.2, max(0., now - self.last_update)) if self.last_update is not None else .1
        self.last_update = now
        self.data["sensor_ages"] = {"image_s": max(0., now - frame_at) if frame_at else None,
                                    "lidar_s": max(0., now - sensors.lidar.at) if sensors and sensors.lidar else None}
        if not self.armed:
            return
        if self.deadline is not None and now >= self.deadline:
            self.stop(now, "试跑时间已到")
            return
        battery = health.get("battery", {})
        if battery.get("latched"):
            self.fault_latched = True
        if self.fault_latched:
            self._wait("fault_paused", "低电压保护，充电后停止并重新启动", now)
            return
        if not health.get("ready") or not enabled:
            self._wait("fault_paused", "等待底盘或识别服务恢复", now)
            return
        if sensors is None or now - frame_at > FRAME_TIMEOUT:
            self._wait("fault_paused", "等待新的相机观测", now)
            return
        target = self.data["target"]
        obs = observations.get(target)
        if obs is None:
            if self.missing_since is None:
                self.missing_since = now
            if target is not None:
                self.confirmation = None
            if target is not None and self.data["unlocked"] and now - self.missing_since >= self.cfg.target_grace_s:
                self.data["target"] = target = None
                self.motion = FollowMotion(self.cfg)
            if target is None:
                target = min(observations, key=lambda key: abs(observations[key].angle_rad), default=None)
                obs = observations.get(target)
            if obs is None:
                self._state("waiting_person", "等待人物重新出现", now)
                return
        if self.missing_since is not None or self.data["target"] is None:
            previous = self.confirmation
            if previous is None or previous[0] != target or frame_at - previous[1] > FRAME_TIMEOUT:
                self.confirmation = (target, frame_at, obs.angle_rad)
                self._state("confirming", f"正在确认人物 {target}", now)
                return
            if frame_at <= previous[1]:
                return
            if abs(obs.angle_rad - previous[2]) > math.radians(8):
                self.confirmation = (target, frame_at, obs.angle_rad)
                self._state("confirming", f"正在确认人物 {target}", now)
                return
            self.data["target"] = target
            self.missing_since = self.confirmation = None
        battery_limit = health.get("available_forward_mps", 0.)
        if battery_limit <= 0:
            self._wait("fault_paused", battery.get("reason", "等待电池数据"), now)
            return
        odom = health.get("odometry") or {}
        measured_speed = abs(odom.get("forward_mps", 0.)) if odom.get("age_s", 99) < .5 else 0.
        current_speed = max(self.data["sent_forward_mps"], measured_speed)
        plan = self.motion.plan(obs, sensors, frame_at, now, current_speed, battery_limit)
        previous_forward, previous_turn = self.data["sent_forward_mps"], self.data["sent_turn_rad_s"]
        self._state(plan.state, plan.reason, now)
        self.data.update(distance_m=plan.distance_m, source=plan.source, offset=obs.offset,
                         limit_mps=plan.limit_mps, limit_reason=plan.limit_reason, quality=plan.quality,
                         planned_forward_mps=plan.forward_mps, planned_turn_rad_s=plan.turn_rad_s)
        if plan.state == "following":
            forward = min(plan.forward_mps, previous_forward + self.cfg.acceleration_mps2 * dt)
            delta = self.cfg.angular_acceleration_rad_s2 * dt
            turn = max(previous_turn - delta, min(previous_turn + delta, plan.turn_rad_s))
            if self.deadline is not None:
                forward = min(forward, .07)
                turn = max(-.12, min(.12, turn))
            self.data.update(forward_mps=round(forward, 4), turn_rad_s=round(turn, 4))
        elif plan.state == "fault_paused":
            self.missing_since, self.confirmation = now, None

    def sent(self, forward, turn):
        self.data.update(sent_forward_mps=forward, sent_turn_rad_s=turn)

    def snapshot(self):
        return dict(self.data)
