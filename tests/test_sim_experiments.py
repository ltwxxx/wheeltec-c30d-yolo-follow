"""Metric and reproducibility contracts; MuJoCo tests skip without optional deps."""
import csv
from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

SIM = Path(__file__).resolve().parents[1] / 'sim'
sys.path.insert(0, str(SIM))

from experiment_core import Controller, Scenario, generate_scenarios, summarize


EVALUATION = dict(control_dt_s=.1, hold_s=2., bearing_tolerance_deg=8.)


def perfect_rows(gap=.45):
    return [dict(time_s=i / 10, true_gap_m=gap, true_bearing_rad=0.,
                 observation_valid=True, left_rad_s=0., right_rad_s=0.)
            for i in range(61)]


def case(kind='static'):
    return Scenario(f'{kind}-01', kind, 1.4, .1, .1, .12, duration_s=6.)


def test_seeded_suite_has_stable_ids_and_reproducible_parameters():
    first = generate_scenarios()
    assert len(first) == 30 and len({s.case_id for s in first}) == 30
    assert first == generate_scenarios()
    assert first != generate_scenarios(seed=43)
    assert all(math.hypot(s.initial_x_m, s.initial_y_m) > 1 for s in first)


@pytest.mark.parametrize('kind', ('static', 'straight', 'turn', 'stop_start', 'dropout'))
def test_target_path_is_continuous_and_stops_before_final_hold(kind):
    s = case(kind)
    assert s.position(0.) == (1.4, .1)
    for t in (.6, 1.5, 2.4, 3.6):
        a, b = s.position(t-1e-7), s.position(t+1e-7)
        assert math.dist(a, b) < 1e-6
    assert s.position(4.) == s.position(6.)
    if kind == 'stop_start':
        assert s.position(1.6) == s.position(2.3)


def test_dropout_is_observation_mask_and_does_not_teleport_target():
    s = case('dropout')
    lo, hi = s.dropout_window
    assert s.visible(lo-1e-7) and not s.visible(lo)
    assert not s.visible(hi-1e-7) and s.visible(hi)
    assert s.position(lo) != s.position(hi)


def test_controller_stops_on_missing_input_and_bounds_actual_wheel_commands():
    c = Controller()
    assert c.command(None, 3.) == ('missing_stop', 0., 0.)
    assert c.command(.1, None) == ('missing_stop', 0., 0.)
    for bearing in (-math.pi, -1., 0., 1., math.pi):
        for gap in (-.2, .45, 100.):
            _, left, right = c.command(bearing, gap)
            assert max(abs(left), abs(right)) <= c.max_wheel_rad_s
    assert c.command(0., .45) == ('holding', 0., 0.)


def test_metrics_use_truth_and_count_band_error_not_observed_distance():
    rows = perfect_rows(.7)
    for row in rows:
        row['observed_gap_m'] = .45
    result = summarize(rows, case(), Controller(), EVALUATION)
    assert result['mean_band_error_m'] == pytest.approx(.2)
    assert result['p95_band_error_m'] == pytest.approx(.2)
    assert not result['success'] and result['settle_after_final_stop_s'] is None
    good = summarize(perfect_rows(), case(), Controller(), EVALUATION)
    assert good['success'] and good['mean_band_error_m'] == 0
    assert good['settle_after_final_stop_s'] == 0


def test_collision_missing_command_and_incomplete_runs_fail_independently():
    rows = perfect_rows()
    assert not summarize(rows, case(), Controller(), EVALUATION,
                         collision_events=1)['success']
    assert not summarize(rows, case(), Controller(), EVALUATION,
                         completed=False)['success']
    rows[20].update(observation_valid=False, left_rad_s=1.)
    result = summarize(rows, case(), Controller(), EVALUATION)
    assert 'command_during_missing_observation' in result['failure_reasons']


def test_final_hold_requires_full_coverage_and_resets_on_late_error():
    assert not summarize(perfect_rows()[-1:], case(), Controller(), EVALUATION)['success']
    rows = perfect_rows()
    rows[-5]['true_bearing_rad'] = math.radians(30)
    assert not summarize(rows, case(), Controller(), EVALUATION)['success']
    assert not summarize(rows[:-10], case(), Controller(), EVALUATION)['success']


def test_recovery_time_is_to_a_sustained_good_band_not_first_detection():
    s = case('dropout')
    rows = perfect_rows()
    for r in rows:
        if r['time_s'] < 3.:
            r['true_gap_m'] = 1.
    result = summarize(rows, s, Controller(), EVALUATION)
    assert result['dropout_recovery_to_band_s'] == pytest.approx(.6)


@pytest.mark.parametrize('kwargs', (
    dict(turn_gain=float('nan')), dict(distance_deadband_m=.5), dict(target_m=-1)))
def test_controller_rejects_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        Controller(**kwargs)


def test_scenario_rejects_path_ids_and_nonfinite_duration():
    with pytest.raises(ValueError):
        replace(case(), case_id='../escape')
    with pytest.raises(ValueError):
        replace(case(), duration_s=float('nan'))


def test_mujoco_contact_filter_excludes_ground_and_self_contacts():
    pytest.importorskip('mujoco')
    from run_experiments import collision_active
    model = SimpleNamespace(geom_bodyid=[0, 1, 2, 3])  # floor, car, wheel, person
    def contacts(*pairs):
        return SimpleNamespace(contact=[SimpleNamespace(geom1=a, geom2=b, dist=dist)
                                        for a, b, dist in pairs])
    assert not collision_active(model, contacts((0, 2, -.001), (1, 2, -.001)), {1, 2}, 3)
    assert not collision_active(model, contacts((1, 3, .001)), {1, 2}, 3)
    assert collision_active(model, contacts((3, 2, -.001)), {1, 2}, 3)


def test_mujoco_saved_case_replays_identical_telemetry_and_rejects_tampering(tmp_path):
    pytest.importorskip('mujoco')
    script = SIM/'run_experiments.py'
    first, replay = tmp_path/'first', tmp_path/'replay'
    def run(*args):
        return subprocess.run([sys.executable, '-B', str(script), *map(str, args)],
                              capture_output=True, text=True, timeout=60)
    result = run('--repeats', 1, '--duration', 6, '--case', 'dropout-01', '--output', first)
    assert result.returncode == 0, result.stdout + result.stderr
    result = run('--replay', first/'manifest.json', '--case', 'dropout-01', '--output', replay)
    assert result.returncode == 0, result.stdout + result.stderr
    a, b = first/'dropout-01', replay/'dropout-01'
    assert (a/'telemetry.csv').read_bytes() == (b/'telemetry.csv').read_bytes()
    assert json.loads((a/'summary.json').read_text()) == json.loads((b/'summary.json').read_text())
    with (a/'telemetry.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    missing = [r for r in rows if r['observation_valid'] == 'False']
    assert missing and all(float(r['left_rad_s']) == float(r['right_rad_s']) == 0 for r in missing)
    assert (first/'report.html').is_file()
    # Wheel/floor contacts must not be counted as task collisions.
    assert json.loads((a/'summary.json').read_text())['collision_events'] == 0
    result = run('--replay', first/'manifest.json', '--turn-gain', 5)
    assert result.returncode != 0 and 'saved parameters' in result.stderr
    result = run('--replay', first/'manifest.json', '--case', 'missing-99')
    assert result.returncode != 0 and 'Case ID' in result.stderr
    result = run('--output', first, '--repeats', 1, '--duration', 6)
    assert result.returncode != 0  # no overwrite
    with (first/'model.xml').open('a') as stream:
        stream.write('\n<!-- changed -->')
    result = run('--replay', first/'manifest.json', '--output', tmp_path/'bad')
    assert result.returncode != 0 and 'checksum' in result.stderr
