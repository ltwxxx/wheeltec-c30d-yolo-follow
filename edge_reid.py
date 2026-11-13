"""Bounded, asynchronous OSNet feature extraction. No image queue accumulates."""
from collections import deque
import atexit
from pathlib import Path
import queue
import threading
import time
import numpy as np
from PIL import Image
from follow_settings import SETTINGS


class ReIdentifier:
    def __init__(self, model=None):
        path = Path(model or SETTINGS.reid_model)
        if not path.is_absolute():
            path = Path(__file__).parent / path
        self.jobs = queue.Queue(maxsize=1)
        self.results = queue.Queue(maxsize=1)
        self.next_at = 0.
        self.thread = None
        self.status = {'ready': False, 'ms': None, 'error': None, 'interval_s': .5}
        if not path.is_file():
            self.status['error'] = 'Re-ID 模型未安装'
            return
        self.path = path
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()
        atexit.register(self.close)

    def close(self):
        if self.thread is None or not self.thread.is_alive():
            return
        try:
            self.jobs.get_nowait()
        except queue.Empty:
            pass
        self.jobs.put_nowait(None)
        self.thread.join(timeout=3)

    def _worker(self):
        try:
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = options.inter_op_num_threads = 1
            options.add_session_config_entry('session.intra_op.allow_spinning', '0')
            model = ort.InferenceSession(str(self.path), options, providers=['CPUExecutionProvider'])
            name = model.get_inputs()[0].name
            self.status['ready'] = True
            while True:
                job = self.jobs.get()
                if job is None:
                    return
                generation, entries = job
                started = time.monotonic()
                inputs = []
                for _, _, _, patch in entries:
                    data = np.asarray(patch.resize((128, 256), Image.Resampling.BILINEAR), dtype=np.float32) / 255.
                    data = (data - np.array([.485, .456, .406], np.float32)) / np.array([.229, .224, .225], np.float32)
                    inputs.append(data.transpose(2, 0, 1))
                vectors = model.run(None, {name: np.ascontiguousarray(np.stack(inputs))})[0]
                results = []
                for (key, at, box, _), vector in zip(entries, vectors):
                    vector = vector / max(float(np.linalg.norm(vector)), 1e-8)
                    results.append((key, at, box, vector))
                elapsed = time.monotonic() - started
                self.status.update(ms=round(elapsed * 1000, 1), interval_s=max(.5, min(3., elapsed * 10)))
                if self.results.full():
                    self.results.get_nowait()
                self.results.put((generation, results))
        except Exception as exc:
            self.status.update(ready=False, error=str(exc))

    def submit(self, image, entries, generation):
        now = time.monotonic()
        if not self.status['ready'] or now < self.next_at or self.jobs.full():
            return
        self.next_at = now + self.status['interval_s']
        # At most two crops per scheduled extraction, selected person first.
        crops = [(key, at, box, image.crop(tuple(map(round, box)))) for key, at, box in entries[:2]]
        if crops:
            self.jobs.put_nowait((generation, crops))

    def poll(self):
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None
