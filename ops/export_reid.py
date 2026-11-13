"""Export official Market1501 OSNet-x0.25 weights; run in Ubuntu-F venv."""
from pathlib import Path
import hashlib
import importlib.util
import json
import urllib.request
import gdown
import numpy as np
import onnxruntime as ort
import torch

root = Path(__file__).resolve().parents[1]
models = root / 'models'
models.mkdir(exist_ok=True)
cache = Path.home() / '.cache/wheeltec-osnet'
cache.mkdir(exist_ok=True)
source = models / 'osnet_export_source.py'
url = 'https://raw.githubusercontent.com/KaiyangZhou/deep-person-reid/master/torchreid/models/osnet.py'
if not source.exists():
    with urllib.request.urlopen(url, timeout=30) as response:
        source.write_bytes(response.read())
spec = importlib.util.spec_from_file_location('osnet', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
weights = models / 'osnet_market_source.pth'
if not weights.exists():
    gdown.download(id='1z1UghYvOTtjx7kEoRfmqSMu-z62J6MAj', output=str(weights), quiet=False)
checkpoint = torch.load(weights, map_location='cpu', weights_only=True)
state = checkpoint.get('state_dict', checkpoint)
state = {key.removeprefix('module.'): value for key, value in state.items()}
net = module.osnet_x0_25(num_classes=751, pretrained=False).eval()
net.load_state_dict(state, strict=True)
torch.set_num_threads(2)
torch.manual_seed(31)
sample = torch.rand(1, 3, 256, 128)
output = models / 'osnet_x0_25.onnx'
torch.onnx.export(net, sample, str(output), input_names=['images'], output_names=['features'],
                  opset_version=17, dynamo=False, dynamic_axes={'images': {0: 'batch'}, 'features': {0: 'batch'}})
with torch.no_grad():
    expected = net(sample).numpy()
actual = ort.InferenceSession(str(output), providers=['CPUExecutionProvider']).run(None, {'images': sample.numpy()})[0]
np.testing.assert_allclose(actual, expected, rtol=2e-4, atol=2e-5)
metadata = {'architecture': 'osnet_x0_25', 'training': 'Market1501', 'input': [1, 3, 256, 128],
            'source': url, 'weights': 'https://drive.google.com/file/d/1z1UghYvOTtjx7kEoRfmqSMu-z62J6MAj/view',
            'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'weights_sha256': hashlib.sha256(weights.read_bytes()).hexdigest(),
            'onnx_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
            'export_max_absolute_error': float(np.max(np.abs(actual - expected))),
            'torch': torch.__version__, 'onnxruntime': ort.__version__}
(models / 'osnet_x0_25.json').write_text(json.dumps(metadata, indent=2))
print(json.dumps(metadata, indent=2))
