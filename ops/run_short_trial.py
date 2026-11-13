"""One explicitly supervised, onboard-expiring 10-second low-speed trial."""
import argparse
import json
from pathlib import Path
import time
import uuid
from urllib.request import Request, urlopen

p = argparse.ArgumentParser()
p.add_argument('--target', type=int, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--confirm-bounded-motion', action='store_true', required=True)
args = p.parse_args()
base = 'http://127.0.0.1:18769'
owner = str(uuid.uuid4())
sequence = int(time.time()*1000)


def read(url):
    with urlopen(url, timeout=1) as response:
        return json.load(response)


def post(payload):
    req = Request(base+'/follow', json.dumps(payload).encode(),
                  {'Content-Type': 'application/json', 'X-Follow-Page': owner})
    with urlopen(req, timeout=2) as response:
        return json.load(response)


status = read(base+'/status')
assert not status['follow']['automatic'] and status['follow']['target'] is None, 'An existing session is active'
assert status['control_ready'] and status['sensors']['lidar'] and args.target in status['ids'], 'Target/sensors not ready'
assert status['battery']['limit_mps'] > 0 and not status['battery']['latched'], 'Battery protection active'
req = Request('http://127.0.0.1:18767/frame', headers={'X-Allow-Color-Only': '1'})
with urlopen(req, timeout=1) as response:
    lidar = json.load(response)['lidar']
assert lidar and lidar['age'] < .3 and lidar['nearest_m'] is not None and lidar['nearest_m'] > .8, 'Initial corridor is not clear'
rows = []
started = time.monotonic()
previous = None
try:
    post({'target': args.target, 'local_trial': True, 'seq': sequence, 'start_status': status['start_status']})
    while time.monotonic() - started < 11:
        data = read(base+'/status')
        follow = data['follow']
        row = dict(elapsed_s=time.monotonic()-started, follow=follow, actuator=data['actuator'],
                   battery=data['battery'], sensors=data['sensors'], reid=data['reid'])
        rows.append(row)
        label = (follow['state'], follow['status'])
        if label != previous:
            print(json.dumps({'elapsed_s': round(row['elapsed_s'], 2), 'state': label,
                              'distance_m': follow['distance_m']}, ensure_ascii=False), flush=True)
            previous = label
        sent = data['actuator']['sent']
        assert abs(sent['forward_mps']) <= .071 and abs(sent['turn_rad_s']) <= .121, 'Trial speed cap exceeded'
        if follow['state'] == 'manual_stop':
            break
        if data['battery']['latched']:
            break
        time.sleep(.1)
finally:
    post({'target': None, 'seq': sequence+1})
    # Query the bridge directly: /status intentionally caches health for up to 0.5 s.
    stationary_since = None
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        bridge = read('http://127.0.0.1:18768/health')
        odo = bridge.get('odometry') or {}
        stopped = (not bridge['active'] and bridge['sent']['forward_mps'] == 0
                   and bridge['sent']['turn_rad_s'] == 0 and odo.get('age_s', 99) < .2
                   and abs(odo.get('forward_mps', 1)) < .01 and abs(odo.get('turn_rad_s', 1)) < .02)
        if stopped:
            stationary_since = stationary_since or time.monotonic()
            if time.monotonic() - stationary_since >= .2:
                break
        else:
            stationary_since = None
        time.sleep(.05)
    else:
        raise RuntimeError('Zero command was sent, but stationary chassis feedback was not confirmed')
    final = read(base+'/status')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('\n'.join(json.dumps(row, ensure_ascii=False) for row in rows)+'\n')
    sent = final['actuator']['sent']
    assert sent['forward_mps'] == 0 and sent['turn_rad_s'] == 0
    summary = {'samples': len(rows), 'max_sent_mps': max((r['actuator']['sent']['forward_mps'] for r in rows), default=0),
               'max_odometry_mps': max((abs((r['actuator']['odometry'] or {}).get('forward_mps', 0)) for r in rows), default=0),
               'states': sorted(set(r['follow']['state'] for r in rows)),
               'lidar_associated_samples': sum(r['follow']['source']=='lidar' for r in rows),
               'final_state': final['follow']['state'], 'final_sent': sent,
               'final_odometry': bridge['odometry']}
    args.output.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)
