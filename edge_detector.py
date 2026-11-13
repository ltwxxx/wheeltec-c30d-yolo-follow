"""YOLO11 ONNX person detector; retain low confidence boxes for ByteTrack association."""
import numpy as np
from PIL import Image
import onnxruntime as ort


class Detector:
    def __init__(self, model):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        self.session = ort.InferenceSession(str(model), sess_options=options, providers=['CPUExecutionProvider'])
        info = self.session.get_inputs()[0]
        self.input = info.name
        self.height, self.width = info.shape[-2:]

    def detect(self, image):
        scale = min(self.width / image.width, self.height / image.height)
        size = round(image.width * scale), round(image.height * scale)
        left, top = (self.width - size[0]) // 2, (self.height - size[1]) // 2
        canvas = Image.new('RGB', (self.width, self.height), (114, 114, 114))
        canvas.paste(image.convert('RGB').resize(size, Image.Resampling.BILINEAR), (left, top))
        blob = np.ascontiguousarray(np.asarray(canvas, dtype=np.float32).transpose(2, 0, 1)[None] / 255.)
        output = self.session.run(None, {self.input: blob})[0][0].T
        if output.ndim != 2 or output.shape[1] != 84:
            raise ValueError(f'Expected YOLO11 COCO output, got {output.shape}')
        scores = output[:, 4]
        keep = (scores >= .10) & (output[:, 4:].argmax(axis=1) == 0)
        scores = scores[keep]
        centers, radii = output[keep, :2], output[keep, 2:4] / 2
        boxes = np.concatenate((centers - radii, centers + radii), axis=1)
        areas = (boxes[:, 2:] - boxes[:, :2]).prod(axis=1)
        order = np.argsort(-scores)
        people = []
        while order.size and len(people) < 50:
            best, rest = order[0], order[1:]
            box = (boxes[best] - [left, top, left, top]) / scale
            box = np.clip(box, 0, [image.width, image.height, image.width, image.height])
            people.append({'confidence': float(scores[best]), 'xyxy': box.tolist()})
            overlap = np.maximum(0, np.minimum(boxes[best, 2:], boxes[rest, 2:]) -
                                 np.maximum(boxes[best, :2], boxes[rest, :2])).prod(axis=1)
            iou = overlap / np.maximum(areas[best] + areas[rest] - overlap, 1e-6)
            order = rest[iou <= .7]
        return people
