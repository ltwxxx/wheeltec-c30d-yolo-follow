"""Map the source layout to the existing robot layout and build a verified archive."""
import argparse
import gzip
import hashlib
import io
from pathlib import Path
import sys
import tarfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from follow_settings import VERSION

SOURCE_MODULES = (
    'camera_viewer', 'follow_control', 'follow_actuator', 'follow_sensors',
    'follow_settings', 'follow_session', 'follow_motion', 'battery_state',
    'lidar_geometry', 'edge_detector', 'edge_tracking', 'edge_reid', 'runtime_paths',
)
RELEASE_FILES = {f'src/{name}.py': f'{name}.py' for name in SOURCE_MODULES}
RELEASE_FILES.update({
    'config/follow_settings.json': 'follow_settings.json',
    'config/calibration.json': 'calibration.json',
    'config/botsort_reid.yaml': 'botsort_reid.yaml',
    'web/templates/follow.html': 'templates/follow.html',
})
for name in (
    'models/osnet_x0_25.onnx', 'models/osnet_x0_25.json', 'models/OSNET-LICENSE',
    'ops/deploy-on-robot.sh', 'ops/robot-follow-boot.sh', 'ops/boot_health.py',
    'ops/wheeltec-follow-edge.service', 'ops/record_validation.py',
):
    RELEASE_FILES[name] = name


def build_release(output_dir=None):
    output_dir = Path(output_dir) if output_dir is not None else ROOT / 'releases'
    # Read every input before creating the output so a missing file fails early.
    contents = {}
    for source, target in RELEASE_FILES.items():
        data = (ROOT / source).read_bytes()
        if source.endswith(('.py', '.sh', '.service', '.json', '.yaml', '.html')):
            data = data.replace(b'\r\n', b'\n')
        contents[target] = data
    checksums = [f'{hashlib.sha256(data).hexdigest()}  {name}' for name, data in contents.items()]
    contents['SHA256SUMS'] = ('\n'.join(checksums) + '\n').encode()
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f'wheeltec-{VERSION}.tar.gz'
    # Fixed archive timestamps make identical source snapshots reproducible.
    with path.open('wb') as raw, gzip.GzipFile(filename='', fileobj=raw, mode='wb', mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode='w') as archive:
            for name, data in contents.items():
                info = tarfile.TarInfo(name)
                info.size = len(data)
                info.mode = 0o755 if name.endswith('.sh') else 0o644
                archive.addfile(info, io.BytesIO(data))
    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, help='Override the default releases/ directory')
    args = parser.parse_args()
    path = build_release(args.output_dir)
    print(path)
    print(hashlib.sha256(path.read_bytes()).hexdigest())
