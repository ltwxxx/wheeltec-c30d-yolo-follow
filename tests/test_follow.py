from dataclasses import replace
import json
import sys
import types
from pathlib import Path
import numpy as np
import pytest
from PIL import Image

from follow_control import Observation, Sensors, LidarFrame, observe_person
from follow_settings import Settings, load_settings
from follow_session import FollowSession
from follow_motion import FollowMotion
from battery_state import BatteryState
from edge_tracking import EdgeTracker
from lidar_geometry import summarize_points

CFG = Settings(camera_to_front_m=.0244, geometry_verified=True, braking_verified=True)
HEALTH = dict(ready=True, available_forward_mps=.25, battery={'latched': False})


def scene(now, distance=1., front=2.):
    sensors = Sensors(None, .89, .5, None, LidarFrame(now, np.empty((0, 3)), front, front))
    obs = Observation(0., 0., None, .15, .5, distance + CFG.camera_to_front_m, .8)
    return {1: obs}, sensors


def tick(session, now, distance=1., front=2., people=None, health=HEALTH):
    obs, sensors = scene(now, distance, front)
    session.update(obs if people is None else people, sensors, now, now, health)
    session.sent(session.data['forward_mps'], session.data['turn_rad_s'])
    return session.data


def running(mode='auto'):
    session = FollowSession(CFG)
    session.start(mode, 1.)
    for now in (1., 1.1, 1.2, 1.3):
        tick(session, now)
    assert session.data['forward_mps'] > 0
    return session


def test_autonomous_session_for_five_minutes_and_restart_idle():
    session = running()
    identifier = session.data['session_id']
    for now in np.arange(1.4, 301.5, .1):
        tick(session, float(now))
    assert session.armed and session.data['session_id'] == identifier
    assert session.data['forward_mps'] == .25
    assert not FollowSession(CFG).armed


def test_missing_locked_person_preserves_identity_and_requires_new_frames():
    session = running()
    tick(session, 1.4, people={})
    assert session.armed and session.data['target'] == 1 and session.data['forward_mps'] == 0
    other = {2: scene(1.5)[0][1]}
    for now in (1.5, 1.6, 1.7, 2.):
        tick(session, now, people=other)
    assert session.data['target'] == 1 and session.data['forward_mps'] == 0
    tick(session, 2.1)
    assert session.data['state'] == 'confirming'
    tick(session, 2.1)
    assert session.data['forward_mps'] == 0
    tick(session, 2.2)
    assert session.data['forward_mps'] > 0


def test_unlocked_grace_then_confirmed_switch():
    session = running('unlocked')
    other = {2: scene(1.4)[0][1]}
    tick(session, 1.4, people=other)
    tick(session, 1.8, people=other)
    assert session.data['target'] == 1 and session.data['forward_mps'] == 0
    tick(session, 2.0, people=other)
    assert session.data['state'] == 'confirming'
    tick(session, 2.1, people=other)
    assert session.data['target'] == 2


def test_arrival_hysteresis_and_near_approach():
    session = running()
    for now, distance in ((1.4, .6), (1.5, .5), (1.6, .4), (1.7, .32), (1.8, .30)):
        tick(session, now, distance)
        if distance < .6:
            assert session.data['forward_mps'] <= .05
    assert session.data['state'] == 'arrived'
    for now in (1.9, 2., 2.1):
        tick(session, now, .39)
        assert session.data['forward_mps'] == 0
    tick(session, 2.2, .41)
    assert session.data['forward_mps'] > 0


def test_corridor_waits_half_second_and_emergency_point_immediate():
    session = running()
    tick(session, 1.4, front=.25)
    assert session.data['state'] == 'waiting_corridor'
    for now in (1.5, 1.6, 1.7, 1.8, 1.9):
        tick(session, now)
        assert session.data['forward_mps'] == 0
    tick(session, 2.01)
    assert session.data['forward_mps'] > 0
    obs, sensors = scene(2.1)
    sensors = replace(sensors, lidar=replace(sensors.lidar, nearest_m=.10))
    session.update(obs, sensors, 2.1, 2.1, HEALTH)
    assert session.data['forward_mps'] == session.data['turn_rad_s'] == 0


def test_stale_frame_scan_and_battery_fault_stop():
    session = running()
    obs, sensors = scene(1.4)
    session.update(obs, sensors, 1.4, 2., HEALTH)
    assert session.data['state'] == 'fault_paused' and session.armed
    session.update(obs, sensors, 2.1, 2.1, HEALTH)
    session.update(obs, sensors, 2.2, 2.2, HEALTH)
    assert session.data['forward_mps'] == 0
    tick(session, 2.3, health=dict(HEALTH, battery={'latched': True}))
    tick(session, 9.)
    assert session.data['state'] == 'fault_paused' and session.data['forward_mps'] == 0


def test_braking_uses_measured_speed_even_after_requested_speed_decreases():
    session = running()
    session.sent(0., 0.)
    health = dict(HEALTH, odometry={'age_s': .1, 'forward_mps': .25, 'turn_rad_s': 0.})
    tick(session, 1.4, distance=.42, health=health)
    tick(session, 1.5, distance=.42, health=health)
    assert session.data['state'] == 'arrived' and session.data['forward_mps'] == 0


def test_visual_fallback_never_fast_or_close():
    planner = FollowMotion(CFG)
    obs, sensors = scene(1.)
    person = replace(obs[1], lidar_distance_m=None)
    assert planner.plan(person, sensors, 1., 1., 0., .25).forward_mps <= .10
    person = replace(person, height_ratio=.95)
    assert planner.plan(person, sensors, 1., 1., 0., .25).forward_mps == 0


def test_persistent_single_beam_obstacle_is_confirmed():
    planner = FollowMotion(CFG)
    obs, sensors = scene(1.)
    sensors = replace(sensors, lidar=replace(sensors.lidar, nearest_m=.2, front_m=None))
    assert planner.plan(obs[1], sensors, 1., 1., .1, .25).state != 'waiting_corridor'
    assert planner.plan(obs[1], sensors, 1., 1.2, .1, .25).state != 'waiting_corridor'
    sensors = replace(sensors, lidar=replace(sensors.lidar, at=1.2))
    assert planner.plan(obs[1], sensors, 1.2, 1.2, .1, .25).state == 'waiting_corridor'


def test_range_continuity_keeps_target_when_background_cluster_appears():
    planner = FollowMotion(CFG)
    obs, sensors = scene(1., 2.5, 3.)
    planner.plan(obs[1], sensors, 1., 1., 0., .25)
    _, sensors = scene(1.1, 2.5, 3.)
    assert planner.plan(obs[1], sensors, 1.1, 1.1, 0., .25).source == 'lidar'
    ambiguous = replace(obs[1], lidar_distance_m=None, lidar_candidates_m=(5.2, 2.5244))
    _, sensors = scene(1.2, 2.5, 3.)
    plan = planner.plan(ambiguous, sensors, 1.2, 1.2, 0., .25)
    assert plan.source == 'lidar' and plan.distance_m == pytest.approx(2.5)
    _, sensors = scene(2., 2.5, 3.)
    assert planner.plan(ambiguous, sensors, 2., 2., 0., .25).source == 'visual'


def test_battery_latch_recovery_and_data_gaps():
    battery = BatteryState(CFG)
    for now in np.arange(0., 3.2, .1):
        battery.observe(10.4, float(now))
    assert battery.status(3.2)['latched']
    for now in np.arange(3.2, 9.5, .1):
        battery.observe(11.2, float(now))
    assert battery.status(9.5)['latched'] and battery.status(9.5)['recoverable']
    assert battery.arm(9.5) and not battery.latched
    assert battery.status(15.)['limit_mps'] == 0
    gap = BatteryState(CFG)
    gap.observe(10.4, 0.)
    gap.observe(10.4, 10.)
    assert not gap.latched
    gap.latched = True
    gap.observe(11.5, 20.)
    gap.observe(11.5, 30.)
    assert not gap.arm(30.)


def test_configuration_rejects_invalid_limits(tmp_path):
    for data in ({'max_speed_mps': .8}, {'battery_stop_s': 1}, {'camera_to_front_m': float('nan')},
                 {'max_speed_mps': None}, {'geometry_verified': 'true'}):
        path = tmp_path / 'cfg.json'
        path.write_text(json.dumps(data))
        with pytest.raises((ValueError, TypeError)):
            load_settings(path)


def test_front_offset_and_spatial_clusters():
    pts = np.array([[-.04, 0, .4244], [-.02, 0, .4244], [0., 0, .4244],
                    [.02, 0, .4244], [.6, 0, .2]])
    projected, front, nearest, clusters = summarize_points(pts, np.arange(5), .89, .89, .5, .5)
    assert front == pytest.approx(.4) and nearest == pytest.approx(.4)
    assert len(clusters) == 1
    sensors = Sensors(None, .89, .5, None, LidarFrame(1., np.array(projected), front, nearest, tuple(clusters)))
    person = observe_person((.3, .1, .7, .9), sensors, use_depth=False)
    assert person.lidar_distance_m == pytest.approx(.4244)


class NoReid:
    status = {'ready': False}
    def poll(self):
        return None
    def submit(self, *args):
        pass


class FakeDetector:
    def __init__(self):
        self.people = []
    def detect(self, image):
        return self.people


def test_low_confidence_maintains_track_but_does_not_create_one():
    detector = FakeDetector()
    tracker = EdgeTracker(detector, NoReid())
    image = Image.new('RGB', (480, 360), 'blue')
    detector.people = [{'xyxy': [80, 30, 180, 330], 'confidence': .9}]
    assert tracker.track(image, 1.) == {}
    assert 1 in tracker.track(image, 1.1)
    detector.people = [{'xyxy': [85, 30, 185, 330], 'confidence': .2},
                       {'xyxy': [300, 30, 400, 330], 'confidence': .2}]
    assert list(tracker.track(image, 1.2)) == [1]
    detector.people = []
    assert tracker.track(image, 1.3) == {}  # Prediction cannot authorize movement.
    detector.people = [{'xyxy': [90, 30, 190, 330], 'confidence': .9}]
    assert tracker.track(image, 1.4) == {}
    assert 1 in tracker.track(image, 1.5)


def test_locked_crossing_waits_for_identity():
    detector = FakeDetector()
    tracker = EdgeTracker(detector, NoReid())
    image = Image.new('RGB', (480, 360), 'blue')
    detector.people = [{'xyxy': [80, 30, 180, 330], 'confidence': .9}]
    tracker.track(image, 1.)
    tracker.track(image, 1.1)
    detector.people.append({'xyxy': [110, 30, 210, 330], 'confidence': .9})
    assert 1 not in tracker.track(image, 1.2, selected_id=1)


def test_low_confidence_duplicate_torso_does_not_block_selected_person():
    detector = FakeDetector()
    tracker = EdgeTracker(detector, NoReid())
    image = Image.new('RGB', (320, 240), 'blue')
    detector.people = [{'xyxy': [130, 82, 176, 167], 'confidence': .4}]
    tracker.track(image, 1.)
    tracker.track(image, 1.1)
    detector.people.append({'xyxy': [132, 83, 175, 115], 'confidence': .2})
    assert 1 in tracker.track(image, 1.2, selected_id=1)
    detector.people[0]['confidence'] = .25
    assert 1 in tracker.track(image, 1.3, selected_id=1)


@pytest.fixture
def viewer(monkeypatch):
    monkeypatch.setenv('WHEELTEC_EDGE_MODEL', 'test-model')
    module = types.ModuleType('edge_detector')
    module.Detector = FakeDetector
    monkeypatch.setitem(sys.modules, 'edge_detector', module)
    sys.modules.pop('camera_viewer', None)
    import camera_viewer as viewer
    viewer.control_link.update(HEALTH)
    viewer.actuator_request = lambda *args, **kwargs: {'ok': True}
    return viewer


def test_status_read_only_start_idempotent_and_stop_cancels_old_requests(viewer):
    client = viewer.app.test_client()
    headers = {'X-Follow-Page': 'a'*36}
    snapshot = client.get('/status').json['start_status']
    payload = {'target': 'unlocked', 'seq': 1, 'start_status': snapshot}
    first = client.post('/follow', json=payload, headers=headers)
    assert first.status_code == 200
    identifier = first.json['session_id']
    before = viewer.session.snapshot()
    for _ in range(20):
        assert client.get('/status').status_code == 200
    assert viewer.session.snapshot() == before
    duplicate = client.post('/follow', json=payload, headers=headers)
    assert duplicate.status_code == 200 and duplicate.json['session_id'] == identifier
    assert client.post('/follow', json={'target': None, 'seq': 2}, headers=headers).status_code == 200
    assert client.post('/follow', json=payload, headers=headers).status_code == 409
    # Old snapshot with a newer sequence must not resurrect a canceled startup.
    assert client.post('/follow', json=dict(payload, seq=3), headers=headers).status_code == 409
    assert not viewer.session.armed


def test_other_page_can_stop_and_origin_is_restricted(viewer):
    client = viewer.app.test_client()
    viewer.session.start('auto', 1.)
    assert client.post('/follow', json={'target': None}, headers={'X-Follow-Page': 'b'*36}).status_code == 200
    assert not viewer.session.armed
    assert client.post('/follow', json={'target': None}, headers={'Origin': 'https://unrelated.example'}).status_code == 403


def test_shutdown_requires_confirmation_and_does_not_run_on_invalid_request(viewer, monkeypatch):
    monkeypatch.setattr(viewer.subprocess, 'run', lambda *a, **kw: pytest.fail('must not shut down'))
    assert viewer.app.test_client().post('/shutdown', json={}).status_code == 400


def test_shutdown_only_after_continuously_stopped_odometry(viewer, monkeypatch):
    calls = []
    now = [100.]
    monkeypatch.setattr(viewer.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(viewer.time, 'sleep', lambda seconds: now.__setitem__(0, now[0]+seconds))
    monkeypatch.setattr(viewer.subprocess, 'run', lambda argv, **kw: calls.append(argv))
    health = dict(active=False, sent={'forward_mps': 0., 'turn_rad_s': 0.},
                  odometry={'age_s': .01, 'forward_mps': 0., 'turn_rad_s': 0.})
    viewer.actuator_request = lambda method, path, *args: health if path == '/health' else {'ok': True}
    response = viewer.app.test_client().post('/shutdown', json={'confirm': 'shutdown-host', 'boot_id': viewer.START_SESSION})
    assert response.status_code == 200 and now[0] >= 100.3
    assert calls == [['sudo', '-n', 'systemctl', 'poweroff']]


def test_shutdown_rejects_moving_or_stale_odometry(viewer, monkeypatch):
    now = [100.]
    monkeypatch.setattr(viewer.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(viewer.time, 'sleep', lambda seconds: now.__setitem__(0, now[0]+seconds))
    monkeypatch.setattr(viewer.subprocess, 'run', lambda *a, **kw: pytest.fail('must not shut down'))
    health = dict(active=False, sent={'forward_mps': 0., 'turn_rad_s': 0.},
                  odometry={'age_s': 3., 'forward_mps': .1, 'turn_rad_s': 0.})
    viewer.actuator_request = lambda method, path, *args: health if path == '/health' else {'ok': True}
    response = viewer.app.test_client().post('/shutdown', json={'confirm': 'shutdown-host', 'boot_id': viewer.START_SESSION})
    assert response.status_code == 503 and not viewer.shutting_down
