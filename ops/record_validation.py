"""Read-only telemetry recording; never sends a motion/start/shutdown request."""
import argparse
import json
from pathlib import Path
import statistics
import subprocess
import time
from urllib.request import urlopen

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=60)
p.add_argument('--url', default='http://127.0.0.1:18769')
p.add_argument('--output', type=Path, required=True)
p.add_argument('--service', default='wheeltec-follow-edge.service')
args = p.parse_args()
started = time.monotonic()
args.output.parent.mkdir(parents=True, exist_ok=True)
samples, errors = [], 0
with args.output.open('w', encoding='utf-8') as stream:
    while time.monotonic() - started < args.seconds:
        tick = time.monotonic()
        try:
            with urlopen(args.url + '/status', timeout=2) as response:
                data = json.load(response)
            row = {'elapsed_s': tick-started, 'follow': data['follow'], 'sensors': data['sensors'],
                   'performance': data.get('performance'), 'battery': data.get('battery'),
                   'actuator': data.get('actuator'), 'version': data.get('version')}
            try:
                pid = subprocess.check_output(['systemctl', '--user', 'show', args.service, '-p', 'MainPID', '--value'], timeout=1, text=True).strip()
                memory = (Path('/proc') / pid / 'status').read_text()
                row['pid'] = int(pid)
                row['rss_kib'] = int(next(line for line in memory.splitlines() if line.startswith('VmRSS:')).split()[1])
            except (OSError, ValueError, StopIteration, subprocess.SubprocessError):
                pass
            samples.append(row)
        except Exception as exc:
            row = {'elapsed_s': tick-started, 'error': str(exc)}
            errors += 1
        stream.write(json.dumps(row, ensure_ascii=False) + '\n')
        stream.flush()
        time.sleep(max(0, .5 - (time.monotonic()-tick)))
fps = [s['performance']['fps'] for s in samples if s['performance']]
latency = [s['performance']['latency_p95_ms'] for s in samples if s['performance'] and s['performance']['latency_p95_ms'] is not None]
summary = {'seconds': time.monotonic()-started, 'samples': len(samples), 'http_errors': errors,
           'median_reported_fps': statistics.median(fps) if fps else None,
           'max_reported_latency_p95_ms': max(latency, default=None),
           'motion_states': sorted(set(s['follow'].get('state', 'legacy') for s in samples)),
           'scope': 'read-only service telemetry, not a physical following/stop-distance test'}
memory = [s['rss_kib'] for s in samples if 'rss_kib' in s]
summary.update(rss_first_kib=memory[0] if memory else None, rss_last_kib=memory[-1] if memory else None,
               rss_peak_kib=max(memory, default=None), pids=sorted(set(s['pid'] for s in samples if 'pid' in s)))
args.output.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
print(json.dumps(summary, indent=2))
