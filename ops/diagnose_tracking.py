"""Read-only perception diagnostic; no actuator, session or service mutations."""
import json
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from edge_detector import Detector
from edge_tracking import EdgeTracker
from follow_control import fetch_frame, observe_person

class Tap:
    def __init__(self):
        self.detector = Detector(str(Path(__file__).resolve().parents[1] / 'follow-yolo11n.onnx'))
        self.rows = []
    def detect(self, image):
        self.rows = self.detector.detect(image)
        return self.rows

detector = Tap()
tracker = EdgeTracker(detector)
selected = None
for i in range(30):
    try:
        _, im, at, sensors = fetch_frame(as_pil=True, allow_color_only=True)
    except ValueError:
        continue
    out = tracker.track(im, at, selected_id=selected)
    if selected is None and out:
        selected = next(iter(out))
    track = tracker.tracks.get(selected)
    ranges = {key: observe_person((box[0]/im.width, box[1]/im.height, box[2]/im.width, box[3]/im.height),
                                  sensors, use_depth=False).lidar_distance_m for key, box in out.items()}
    print(json.dumps({'i': i, 'raw': [[round(d['confidence'], 2), [round(v) for v in d['xyxy']]] for d in detector.rows],
                      'out': list(out), 'selected': selected, 'hits': track.hits if track else None,
                      'gate': track.needs_reid_at if track else None, 'reid': tracker.reid_status,
                      'ranges': ranges}, ensure_ascii=False), flush=True)
    time.sleep(.03)
