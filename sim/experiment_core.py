"""Deterministic scenarios, teaching controller, and metrics (no physics imports)."""
from dataclasses import asdict, dataclass
import math
import random
import re


KINDS = ('static', 'straight', 'turn', 'stop_start', 'dropout')


@dataclass(frozen=True)
class Controller:
    target_m: float = .45
    distance_deadband_m: float = .05
    distance_gain: float = 3.
    turn_gain: float = 2.
    max_wheel_rad_s: float = 6.
    max_turn_rad_s: float = 3.
    turn_deadband_rad: float = math.radians(3)

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('Controller parameters must be finite and positive')
        if self.distance_deadband_m >= self.target_m:
            raise ValueError('Distance deadband must be smaller than target')

    def command(self, bearing, gap):
        # Same P formula as follow_p_control.py; missing observation stops both wheels.
        if bearing is None or gap is None:
            return 'missing_stop', 0., 0.
        error = gap - self.target_m
        if abs(error) <= self.distance_deadband_m:
            error = 0.
        speed = max(-self.max_wheel_rad_s, min(self.max_wheel_rad_s,
                                             self.distance_gain * error))
        speed *= max(0., math.cos(bearing))
        turn = 0. if abs(bearing) <= self.turn_deadband_rad else max(
            -self.max_turn_rad_s, min(self.max_turn_rad_s, self.turn_gain * bearing))
        limit = self.max_wheel_rad_s
        left = max(-limit, min(limit, speed - turn))
        right = max(-limit, min(limit, speed + turn))
        return ('holding' if not speed and not turn else 'following'), left, right


@dataclass(frozen=True)
class Scenario:
    case_id: str
    kind: str
    initial_x_m: float
    initial_y_m: float
    target_speed_mps: float
    turn_rate_rad_s: float
    duration_s: float = 24.

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError('Unknown scenario kind')
        if not re.fullmatch(re.escape(self.kind) + r'-[0-9]+', self.case_id):
            raise ValueError('Case ID must be KIND-NUMBER')
        if any(not math.isfinite(v) for v in (
                self.initial_x_m, self.initial_y_m, self.target_speed_mps,
                self.turn_rate_rad_s, self.duration_s)):
            raise ValueError('Scenario numbers must be finite')
        if self.duration_s < 6 or self.target_speed_mps < 0:
            raise ValueError('Duration must be >= 6 s; speed must be nonnegative')

    @property
    def stop_at_s(self):
        return self.duration_s * .60 if self.kind != 'static' else 0.

    @property
    def dropout_window(self):
        return (self.duration_s * .30, self.duration_s * .40)

    def visible(self, t):
        lo, hi = self.dropout_window
        return not (self.kind == 'dropout' and lo <= t < hi)

    def position(self, t):
        """Scripted, continuous target motion; last 40% is stationary."""
        duration = self.duration_s
        active = max(0., min(t, self.stop_at_s) - duration * .10)
        if self.kind == 'static':
            active = 0.
        elif self.kind == 'stop_start':
            active = (max(0., min(t, duration * .25) - duration * .10)
                      + max(0., min(t, duration * .60) - duration * .40))
        distance = self.target_speed_mps * active
        if self.kind == 'turn' and abs(self.turn_rate_rad_s) > 1e-9:
            angle = self.turn_rate_rad_s * active
            radius = self.target_speed_mps / self.turn_rate_rad_s
            return (self.initial_x_m + radius * math.sin(angle),
                    self.initial_y_m + radius * (1 - math.cos(angle)))
        return self.initial_x_m + distance, self.initial_y_m


def generate_scenarios(seed=42, repeats=6, duration_s=24.):
    if repeats < 1:
        raise ValueError('Repeats must be positive')
    rng = random.Random(seed)
    return [Scenario(f'{kind}-{i + 1:02d}', kind, rng.uniform(1.0, 1.8),
                     rng.uniform(-.35, .35), rng.uniform(.06, .12),
                     rng.choice((-1., 1.)) * rng.uniform(.08, .16), duration_s)
            for kind in KINDS for i in range(repeats)]


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    at = (len(ordered) - 1) * fraction
    lo, hi = math.floor(at), math.ceil(at)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (at - lo)


def summarize(rows, scenario, controller, evaluation, collision_events=0,
              completed=True, termination=None):
    """All scoring uses ground truth; command/observation columns never define truth."""
    if not rows:
        raise ValueError('Cannot score an empty episode')
    low = controller.target_m - controller.distance_deadband_m
    high = controller.target_m + controller.distance_deadband_m
    limit = math.radians(evaluation['bearing_tolerance_deg'])
    dt = evaluation['control_dt_s']
    hold = evaluation['hold_s']

    def good(row):
        return low <= row['true_gap_m'] <= high and abs(row['true_bearing_rad']) <= limit

    def stable_after(start):
        since = None
        for row in rows:
            if row['time_s'] + 1e-9 < start:
                continue
            if good(row):
                if since is None:
                    since = row['time_s']
                if row['time_s'] - since + dt >= hold - 1e-9:
                    return max(0., since - start)
            else:
                since = None
        return None

    errors = [max(low - r['true_gap_m'], 0., r['true_gap_m'] - high) for r in rows]
    tail = [r for r in rows if r['time_s'] >= scenario.duration_s - hold - 1e-9]
    final_ok = (completed and bool(tail) and
                tail[0]['time_s'] <= scenario.duration_s - hold + dt + 1e-9 and
                rows[-1]['time_s'] >= scenario.duration_s - dt - 1e-9 and all(map(good, tail)))
    reasons = []
    if not completed:
        reasons.append(termination or 'incomplete')
    if collision_events:
        reasons.append('collision')
    if not final_ok:
        reasons.append('final_hold_not_met')
    missing = [r for r in rows if not r['observation_valid']]
    if any(abs(r['left_rad_s']) + abs(r['right_rad_s']) > 1e-9 for r in missing):
        reasons.append('command_during_missing_observation')
    stopped = [r for r in rows if r['time_s'] >= scenario.stop_at_s]
    change = sum((abs(b['left_rad_s'] - a['left_rad_s'])
                  + abs(b['right_rad_s'] - a['right_rad_s'])) / 2
                 for a, b in zip(rows, rows[1:]))
    recovery = (stable_after(scenario.dropout_window[1])
                if scenario.kind == 'dropout' else None)
    return dict(
        case_id=scenario.case_id, kind=scenario.kind, success=not reasons,
        failure_reasons=reasons, samples=len(rows), completed=completed,
        mean_band_error_m=sum(errors) / len(errors),
        p95_band_error_m=percentile(errors, .95),
        mean_abs_bearing_deg=sum(abs(math.degrees(r['true_bearing_rad'])) for r in rows) / len(rows),
        in_band_and_aligned_fraction=sum(map(good, rows)) / len(rows),
        minimum_gap_m=min(r['true_gap_m'] for r in rows),
        final_gap_m=rows[-1]['true_gap_m'],
        settle_after_final_stop_s=stable_after(scenario.stop_at_s),
        post_stop_near_band_overshoot_m=max((max(0., low-r['true_gap_m']) for r in stopped), default=0.),
        dropout_recovery_to_band_s=recovery,
        collision_events=collision_events,
        missing_observation_fraction=len(missing) / len(rows),
        mean_wheel_command_variation_rad_s2=change / max(dt, rows[-1]['time_s']),
        final_hold_met=final_ok,
    )
