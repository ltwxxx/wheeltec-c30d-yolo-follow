"""Shared startup configuration for the controller, sensor and actuator services."""
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
from runtime_paths import config_path

VERSION = "2.0.1"


@dataclass(frozen=True)
class Settings:
    stop_m: float = 0.30
    resume_m: float = 0.40
    camera_to_front_m: float | None = None
    geometry_verified: bool = False
    brake_deceleration_mps2: float = 0.30
    braking_verified: bool = False
    body_half_width_m: float = 0.116
    corridor_margin_m: float = 0.04
    max_speed_mps: float = 0.25
    fallback_speed_mps: float = 0.10
    approach_speed_mps: float = 0.05
    unlocked_cruise_mps: float = 0.0
    unlocked_clearance_m: float = 1.0
    forward_gain_per_s: float = 0.65
    turn_gain_per_s: float = 0.90
    turn_deadband_deg: float = 5.0
    full_forward_angle_deg: float = 9.0
    turn_only_angle_deg: float = 12.0
    near_slowdown_m: float = 0.60
    acceleration_mps2: float = 0.30
    angular_acceleration_rad_s2: float = 0.60
    max_turn_rad_s: float = 0.25
    obstacle_clear_s: float = 0.50
    target_grace_s: float = 0.50
    battery_slow_v: float = 10.8
    battery_stop_v: float = 10.5
    battery_stop_s: float = 3.0
    battery_recover_s: float = 5.0
    detection_min_confidence: float = 0.10
    tracking_new_confidence: float = 0.35
    detection_nms_iou: float = 0.70
    reid_min_similarity: float = 0.82
    reid_min_margin: float = 0.08
    reid_model: str = "models/osnet_x0_25.onnx"

    @property
    def calibrated(self):
        return self.geometry_verified and self.braking_verified and self.camera_to_front_m is not None


def load_settings(path=None):
    path = Path(path or os.environ.get("WHEELTEC_FOLLOW_CONFIG", config_path("follow_settings.json")))
    values = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    settings = Settings(**values)
    for key, value in asdict(settings).items():
        default = getattr(Settings(), key)
        if isinstance(default, bool):
            if type(value) is not bool:
                raise ValueError(f"{key} must be boolean")
        elif key == "reid_model":
            if not isinstance(value, str):
                raise ValueError("reid_model must be a path")
        elif value is None:
            if key != "camera_to_front_m":
                raise ValueError(f"{key} cannot be null")
        else:
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f"Invalid {key}")
    if not 0.25 <= settings.stop_m < settings.resume_m <= 1.5:
        raise ValueError("Invalid stop/resume distances")
    if not 0 < settings.approach_speed_mps <= settings.fallback_speed_mps <= settings.max_speed_mps <= .25:
        raise ValueError("Invalid speed limits")
    if settings.unlocked_cruise_mps and not settings.fallback_speed_mps <= settings.unlocked_cruise_mps <= .25:
        raise ValueError("Invalid unlocked cruise speed")
    if not 1.0 <= settings.unlocked_clearance_m <= 5.0:
        raise ValueError("Unlocked cruise requires at least 1 m of measured clearance")
    if not (0 < settings.forward_gain_per_s <= 2 and 0 < settings.turn_gain_per_s <= 2
            and 0 <= settings.turn_deadband_deg < settings.full_forward_angle_deg < settings.turn_only_angle_deg <= 25
            and .30 <= settings.near_slowdown_m <= 2):
        raise ValueError("Invalid steering or approach settings")
    if not (.05 <= settings.detection_min_confidence < settings.tracking_new_confidence <= .95
            and .30 <= settings.detection_nms_iou <= .90
            and .70 <= settings.reid_min_similarity <= .98
            and .02 <= settings.reid_min_margin <= .30):
        raise ValueError("Invalid detection or identity thresholds")
    if not 0 < settings.brake_deceleration_mps2 <= 2 or not 0 < settings.acceleration_mps2 <= 1:
        raise ValueError("Invalid acceleration")
    if not 0 < settings.battery_stop_v < settings.battery_slow_v < 12.6:
        raise ValueError("Invalid voltage limits")
    if settings.geometry_verified and settings.camera_to_front_m is None:
        raise ValueError("Measured camera offset is required")
    if not 0.05 <= settings.body_half_width_m <= 0.5 or not 0 < settings.max_turn_rad_s <= .25:
        raise ValueError("Invalid chassis limits")
    if settings.battery_stop_s < 3 or settings.battery_recover_s < 5:
        raise ValueError("Battery protection intervals cannot be shortened")
    return settings


SETTINGS = load_settings()
