"""Build a checksummed robot release without credentials or training weights."""
from pathlib import Path
import hashlib
import io
import tarfile

root = Path(__file__).resolve().parents[1]
files = ['camera_viewer.py', 'follow_control.py', 'follow_actuator.py', 'follow_sensors.py',
         'follow_settings.py', 'follow_settings.json', 'follow_session.py', 'follow_motion.py',
         'battery_state.py', 'lidar_geometry.py', 'edge_detector.py', 'edge_tracking.py', 'edge_reid.py',
         'calibration.json', 'templates/follow.html', 'models/osnet_x0_25.onnx', 'models/osnet_x0_25.json',
         'models/OSNET-LICENSE', 'ops/deploy-on-robot.sh', 'ops/robot-follow-boot.sh', 'ops/boot_health.py',
         'ops/wheeltec-follow-edge.service', 'ops/record_validation.py']
out = root / 'releases'
out.mkdir(exist_ok=True)
path = out / 'wheeltec-2.0.0.tar.gz'
checksums = []
with tarfile.open(path, 'w:gz') as archive:
    for name in files:
        data = (root / name).read_bytes()
        if name.endswith(('.sh', '.service')):
            data = data.replace(b'\r\n', b'\n')
        info = tarfile.TarInfo(name)
        info.size = len(data)
        info.mode = 0o755 if name.endswith('.sh') else 0o644
        archive.addfile(info, io.BytesIO(data))
        checksums.append(f'{hashlib.sha256(data).hexdigest()}  {name}')
    data = ('\n'.join(checksums)+'\n').encode()
    info = tarfile.TarInfo('SHA256SUMS')
    info.size = len(data)
    archive.addfile(info, io.BytesIO(data))
print(path)
print(hashlib.sha256(path.read_bytes()).hexdigest())
