"""ByteTrack-style high/low association and Kalman prediction with OSNet identity gates.

Predictions are for association only. Only current detections can command motion.
"""
from dataclasses import dataclass, field
from collections import deque
import numpy as np
from scipy.optimize import linear_sum_assignment
from PIL import ImageDraw
from edge_reid import ReIdentifier


def iou(a, b):
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = max(0, a[2]-a[0]) * max(0, a[3]-a[1]) + max(0, b[2]-b[0]) * max(0, b[3]-b[1]) - intersection
    return intersection / union if union > 0 else 0.


def torso_colors(image, box):
    x1, y1, x2, y2 = box
    w, h = x2-x1, y2-y1
    rgb = np.asarray(image.crop((round(x1+.25*w), round(y1+.22*h), round(x2-.25*w), round(y1+.57*h))).resize((20, 20)), dtype=np.uint8) // 64
    bins = rgb[:, :, 0]*16 + rgb[:, :, 1]*4 + rgb[:, :, 2]
    return np.bincount(bins.ravel(), minlength=64) / bins.size


def measurement(box):
    x1, y1, x2, y2 = box
    return np.array([(x1+x2)/2, (y1+y2)/2, x2-x1, y2-y1])


@dataclass
class Track:
    box: tuple
    colors: np.ndarray
    last_seen: float
    hits: int = 1
    confirmed: bool = False
    mean: np.ndarray = field(default_factory=lambda: np.zeros(8))
    covariance: np.ndarray = field(default_factory=lambda: np.eye(8)*100)
    feature: np.ndarray | None = None
    feature_at: float = 0.
    needs_reid_at: float | None = None

    def __post_init__(self):
        self.mean[:4] = measurement(self.box)

    def predict(self, dt):
        transition = np.eye(8)
        transition[:4, 4:] = np.eye(4) * dt
        self.mean = transition @ self.mean
        self.covariance = transition @ self.covariance @ transition.T + np.eye(8) * max(1., dt*30)
        x, y, w, h = self.mean[:4]
        w, h = max(8., w), max(12., h)
        return (x-w/2, y-h/2, x+w/2, y+h/2)

    def observe(self, box, colors, now):
        residual = measurement(box) - self.mean[:4]
        innovation = self.covariance[:4, :4] + np.eye(4)*9
        gain = np.linalg.solve(innovation, self.covariance[:4, :]).T
        self.mean += gain @ residual
        self.covariance -= gain @ self.covariance[:4, :]
        self.box, self.colors = tuple(box), colors
        self.hits = self.hits + 1 if now - self.last_seen <= .5 else 1
        self.last_seen = now
        self.confirmed |= self.hits >= 2


class EdgeTracker:
    def __init__(self, detector, reid=None):
        self.detector = detector
        self.reid = reid if reid is not None else ReIdentifier()
        self.tracks = {}
        self.next_id = 1
        self.last_frame_at = None
        self.generation = 0
        self.selected_id = None
        self.gallery = deque(maxlen=6)
        self.gallery_at = 0.
        self.anchor = None

    def reset(self):
        self.tracks.clear()
        self.last_frame_at = None
        self.generation += 1
        self.gallery.clear()
        self.anchor = None
        self.selected_id = None

    @property
    def reid_status(self):
        return dict(self.reid.status, gallery_size=len(self.gallery))

    @property
    def known_ids(self):
        return [key for key, track in self.tracks.items() if track.confirmed]

    @property
    def visible_boxes(self):
        return {key: track.box for key, track in self.tracks.items() if track.last_seen == self.last_frame_at}

    def similarity(self, vector):
        refs = list(self.gallery) + ([self.anchor] if self.anchor is not None else [])
        return max((float(vector @ ref) for ref in refs), default=0.)

    def track(self, image, captured_at, selected_id=None, locked=True):
        if self.last_frame_at is not None and captured_at <= self.last_frame_at:
            return {}
        if selected_id != self.selected_id or not locked:
            self.gallery.clear()
            self.anchor = None
            self.selected_id = selected_id if locked else None
        result = self.reid.poll()
        if result and result[0] == self.generation:
            for key, at, box, vector in result[1]:
                track = self.tracks.get(key)
                if track and captured_at - at < 1.2 and iou(track.box, box) > .3:
                    track.feature, track.feature_at = vector, at
        dt = min(.5, captured_at - self.last_frame_at) if self.last_frame_at is not None else .1
        self.last_frame_at = captured_at
        self.tracks = {key: track for key, track in self.tracks.items()
                       if captured_at - track.last_seen < 1.5 or key == self.selected_id}
        predictions = {key: track.predict(dt) for key, track in self.tracks.items()}
        detections = [p for p in self.detector.detect(image) if p['confidence'] >= .1
                      and p['xyxy'][2] - p['xyxy'][0] >= 8 and p['xyxy'][3] - p['xyxy'][1] >= 12]
        boxes = [tuple(p['xyxy']) for p in detections]
        # Low-score torso duplicates maintain a track; they are not new identities.
        high_indices = {i for i, person in enumerate(detections) if person['confidence'] >= .35}
        ambiguous_boxes = {i for i in high_indices
                           if any(i != j and iou(boxes[i], boxes[j]) > .25 for j in high_indices)}
        colors = [torso_colors(image, b) for b in boxes]
        high = [i for i, p in enumerate(detections) if p['confidence'] >= .35]
        low = [i for i, p in enumerate(detections) if .1 <= p['confidence'] < .35]
        remaining = set(self.tracks)
        matched_detections = set()
        for indices, low_stage in ((high, False), (low, True)):
            keys = [key for key in remaining if captured_at - self.tracks[key].last_seen <= (.5 if low_stage else 1.5)]
            if not keys or not indices:
                continue
            cost = np.full((len(keys), len(indices)), 1e6)
            for row, key in enumerate(keys):
                for column, index in enumerate(indices):
                    overlap = iou(predictions[key], boxes[index])
                    similarity = float(np.minimum(self.tracks[key].colors, colors[index]).sum())
                    if overlap >= (.20 if low_stage else .15) and similarity >= .35:
                        cost[row, column] = 1-overlap + .15*(1-similarity)
            rows, columns = linear_sum_assignment(cost)
            for row, column in zip(rows, columns):
                if cost[row, column] >= 1e5:
                    continue
                key, index = keys[row], indices[column]
                track = self.tracks[key]
                if key == self.selected_id:
                    ambiguous = index in ambiguous_boxes
                    if ambiguous or captured_at - track.last_seen > .35:
                        if track.needs_reid_at is None:
                            track.needs_reid_at = captured_at
                track.observe(boxes[index], colors[index], captured_at)
                remaining.remove(key)
                matched_detections.add(index)
        for index in high:
            if index not in matched_detections:
                self.tracks[self.next_id] = Track(boxes[index], colors[index], captured_at)
                self.next_id += 1
        for key in remaining:
            self.tracks[key].hits = 0
        selected = self.tracks.get(self.selected_id)
        if selected and selected.feature is not None and selected.needs_reid_at is None and selected.last_seen == captured_at:
            if self.anchor is None:
                self.anchor = selected.feature.copy()
            if not self.gallery or (selected.feature_at > self.gallery_at and self.similarity(selected.feature) >= .80
                                    and float(selected.feature @ self.anchor) >= .75):
                self.gallery.append(selected.feature.copy())
                self.gallery_at = selected.feature_at
        if selected and selected.needs_reid_at is not None:
            if (selected.feature is not None and selected.feature_at >= selected.needs_reid_at
                    and self.similarity(selected.feature) >= .82
                    and not any(iou(selected.box, boxes[i]) > .25 for i in ambiguous_boxes)):
                selected.needs_reid_at = None
        if selected and self.gallery and (selected.last_seen != captured_at or selected.needs_reid_at is not None):
            candidates = sorted([(self.similarity(t.feature), key) for key, t in self.tracks.items()
                                 if t.feature is not None and t.last_seen == captured_at
                                 and captured_at - t.feature_at < .65 and t.hits >= 2
                                 and t.feature_at >= (selected.needs_reid_at if selected.needs_reid_at is not None else selected.last_seen)
                                 and not any(iou(t.box, boxes[i]) > .25 for i in ambiguous_boxes)], reverse=True)
            if candidates and candidates[0][0] >= .82 and (len(candidates) == 1 or candidates[0][0]-candidates[1][0] >= .08):
                recovered_id = candidates[0][1]
                if recovered_id != self.selected_id:
                    recovered = self.tracks.pop(recovered_id)
                    recovered.hits = 1
                    recovered.needs_reid_at = None
                    self.tracks[self.selected_id] = recovered
        visible = self.visible_boxes
        if locked and self.selected_id is not None:
            selected = self.tracks.get(self.selected_id)
            keys = ([self.selected_id] if selected and self.selected_id in visible and selected.needs_reid_at is None
                    else list(visible))
            keys.sort(key=lambda key: (key != self.selected_id, self.tracks[key].feature_at))
            self.reid.submit(image, [(key, captured_at, visible[key]) for key in keys], self.generation)
        return {key: track.box for key, track in self.tracks.items() if track.last_seen == captured_at
                and track.hits >= 2 and (key != self.selected_id or track.needs_reid_at is None)}


def draw_tracks(image, boxes, target):
    draw = ImageDraw.Draw(image)
    for track_id, box in boxes.items():
        color = '#ffd08a' if track_id == target else '#65e7c4'
        draw.rectangle(box, outline=color, width=2)
        left, top = box[:2]
        draw.text((left+3, max(0, top-12)), f'ID {track_id}', fill=color, stroke_width=1)
    return image
