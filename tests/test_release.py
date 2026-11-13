"""Check real checkout and extracted-release resources without connecting to a robot."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile

from runtime_paths import RESOURCE_ROOT, TEMPLATES_DIR, config_path

ROOT = Path(__file__).resolve().parents[1]


def test_checkout_resources_and_model_checksum():
    assert RESOURCE_ROOT == ROOT
    assert config_path('follow_settings.json').is_file()
    assert config_path('botsort_reid.yaml').is_file()
    assert (TEMPLATES_DIR / 'follow.html').is_file()
    metadata = json.loads((ROOT / 'models/osnet_x0_25.json').read_text())
    assert hashlib.sha256((ROOT / 'models/osnet_x0_25.onnx').read_bytes()).hexdigest() == metadata['onnx_sha256']
    assert hashlib.sha256((ROOT / 'models/osnet_export_source.py').read_bytes()).hexdigest() == metadata['source_sha256']


def test_release_checksums_and_standalone_runtime(tmp_path):
    spec = importlib.util.spec_from_file_location('build_release', ROOT / 'ops/build_release.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    first = builder.build_release(tmp_path / 'first')
    second = builder.build_release(tmp_path / 'second')
    assert first.read_bytes() == second.read_bytes()
    runtime = tmp_path / 'runtime'
    runtime.mkdir()
    with tarfile.open(first, 'r:gz') as archive:
        members = archive.getmembers()
        assert all(member.isfile() and not Path(member.name).is_absolute() and '..' not in Path(member.name).parts
                   for member in members)
        for member in members:
            target = runtime / member.name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.extractfile(member).read())
    for line in (runtime / 'SHA256SUMS').read_text().splitlines():
        digest, name = line.split('  ', 1)
        assert hashlib.sha256((runtime / name).read_bytes()).hexdigest() == digest
    # Exercise the standalone deployment from an unrelated current directory.
    # The fake detector prevents any model load; importing the viewer starts no control loop.
    code = '''
import json, os, sys, types
from pathlib import Path
sys.path.insert(0, sys.argv[1])
os.environ['WHEELTEC_EDGE_MODEL'] = 'stub'
detector = types.ModuleType('edge_detector')
detector.Detector = object
sys.modules['edge_detector'] = detector
from follow_settings import SETTINGS
from runtime_paths import RESOURCE_ROOT, config_path
from camera_viewer import app
root = Path(sys.argv[1])
assert RESOURCE_ROOT == root
assert config_path('follow_settings.json') == root / 'follow_settings.json'
assert SETTINGS.camera_to_front_m == json.loads((root / 'follow_settings.json').read_text())['camera_to_front_m']
assert (root / SETTINGS.reid_model).is_file()
response = app.test_client().get('/')
assert response.status_code == 200
assert 'html' in response.get_data(as_text=True).lower()
print('standalone runtime resources loaded')
'''
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    env.pop('WHEELTEC_FOLLOW_CONFIG', None)
    result = subprocess.run([sys.executable, '-B', '-c', code, str(runtime)],
                            cwd=tmp_path, env=env, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
