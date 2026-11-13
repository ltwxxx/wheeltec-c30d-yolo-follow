"""Actual-data liveness checks and per-service exponential restart backoff.

Invoked by the existing boot supervisor. Successful checks do not change follow state.
"""
import json
from pathlib import Path
import subprocess
import time
from urllib.request import urlopen, Request

STATE = Path('/home/wheeltec/.local/share/wheeltec-follow-edge/boot-health.json')


def read(port, path):
    try:
        with urlopen(f'http://127.0.0.1:{port}{path}', timeout=1) as response:
            return json.load(response)
    except Exception:
        return {}


def fresh(value, limit):
    return isinstance(value, (int, float)) and 0 <= value <= limit


def stop_bridge():
    try:
        with urlopen(Request('http://127.0.0.1:18768/stop', b'{}', {'Content-Type': 'application/json'}), timeout=1):
            pass
    except Exception:
        pass


def kill(pattern, signal='-TERM'):
    # Only the named service executable; the existing launch/supervisor recreates it.
    subprocess.run(['docker', 'exec', 'codex-camera-live', 'pkill', signal, '-f', pattern], timeout=5)


def main():
    now = time.monotonic()
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    try:
        state = json.loads(STATE.read_text())
        if now < state.get('checked', now) or state.get('boot_id') != boot_id:
            state = {}
    except (OSError, ValueError):
        state = {}
    sensors, actuator, viewer = read(18767, '/health'), read(18768, '/health'), read(18769, '/healthz')
    checks = {
        'sensors': bool(sensors),
        'camera': bool(sensors) and fresh(sensors.get('color_age_s'), 1),
        'lidar': bool(sensors) and fresh(sensors.get('scan_age_s'), 1),
        'actuator': bool(actuator),
        'base': bool(actuator) and fresh((actuator.get('odometry') or {}).get('age_s'), 1)
                and actuator.get('battery_voltage') is not None,
        'viewer': bool(viewer) and fresh(viewer.get('control_age_s'), 1)
                  and fresh(viewer.get('image_age_s'), 2)
                  and (not viewer.get('detection_enabled', True) or fresh(viewer.get('inference_age_s'), 2)),
    }
    # Do not restart upstream producers just because their reader is unavailable.
    if not sensors:
        for key in ('camera', 'lidar'):
            checks.pop(key)
    if not actuator:
        checks.pop('base')
    patterns = {
        'sensors': '^python3 -u /tmp/follow_sensors.py$',
        'actuator': '^python3 -u /tmp/follow_actuator.py$',
        'camera': '^/home/wheeltec/wheeltec_ros2/install/astra_camera/lib/astra_camera/astra_camera_node ',
        'lidar': '^/home/wheeltec/wheeltec_ros2/install/lslidar_driver/lib/lslidar_driver/lslidar_driver_node ',
        'base': '^/home/wheeltec/wheeltec_ros2/install/turn_on_wheeltec_robot/lib/turn_on_wheeltec_robot/wheeltec_robot_node ',
    }
    for name, healthy in checks.items():
        entry = state.setdefault(name, {'failures': 0, 'next': now + 60, 'backoff': 20, 'good': 0})
        if healthy:
            entry['failures'] = 0
            entry['good'] += 1
            if entry['good'] >= 8:
                entry['backoff'] = 20
            continue
        entry['good'] = 0
        entry['failures'] += 1
        if entry['failures'] < 3 or now < entry['next']:
            continue
        stop_bridge()
        if name == 'viewer':
            subprocess.run(['systemctl', '--user', 'restart', 'wheeltec-follow-edge.service'], timeout=10)
        else:
            kill(patterns[name])
            # Launchers do not reliably exit when a child dies. Retire that launch group.
            launches = {'camera': 'astra_camera astra.launch.xml ',
                        'lidar': 'turn_on_wheeltec_robot wheeltec_lidar.launch.py$',
                        'base': 'turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py$'}
            if name in launches:
                kill('^(.*python3 )?.*/ros2 launch ' + launches[name], '-INT')
        entry['next'] = now + entry['backoff']
        entry['backoff'] = min(180, entry['backoff'] * 2)
        entry['failures'] = 0
        print(json.dumps({'service': name, 'restart_at': now, 'retry_after_s': entry['backoff']}), flush=True)
    state['checked'] = now
    state['boot_id'] = boot_id
    state['depth_ready'] = fresh(sensors.get('depth_age_s'), 1)
    STATE.parent.mkdir(parents=True, exist_ok=True)
    temporary = STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(state))
    temporary.replace(STATE)


if __name__ == '__main__':
    main()
