#!/usr/bin/env python3
"""ROS-free CPU worker. One JPEG request/JSON response per line; no downloads."""
import base64
import json
import sys
import time
import cv2
import numpy as np
import onnxruntime as ort


def mask_color(image, mask):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    valid = mask & (hsv[:, :, 1] >= 90) & (hsv[:, :, 2] >= 50)
    hue = hsv[:, :, 0]
    counts = dict(red=int(np.count_nonzero(valid & ((hue <= 10) | (hue >= 170)))),
                  yellow=int(np.count_nonzero(valid & (hue >= 18) & (hue <= 38))),
                  green=int(np.count_nonzero(valid & (hue >= 39) & (hue <= 85))),
                  blue=int(np.count_nonzero(valid & (hue >= 90) & (hue <= 135))))
    total = int(np.count_nonzero(valid))
    color = max(counts, key=counts.get)
    if counts[color] < 8 or counts[color] < total * .5:
        color = None
    return color, counts


class Detector:
    def __init__(self, model):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(model, options, providers=['CPUExecutionProvider'])

    def detect(self, image):
        started = time.monotonic()
        height, width = image.shape[:2]
        ratio = min(640 / width, 640 / height)
        nw, nh = round(width * ratio), round(height * ratio)
        left, top = (640 - nw) // 2, (640 - nh) // 2
        canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
        canvas[top:top + nh, left:left + nw] = cv2.resize(image, (nw, nh))
        tensor = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.
        prediction, proto = self.session.run(None, {self.session.get_inputs()[0].name: tensor})
        rows = prediction[0].T
        scores = rows[:, 4:6].max(axis=1)
        rows, scores = rows[scores >= .25], scores[scores >= .25]
        xywh = rows[:, :4].copy()
        xywh[:, :2] -= xywh[:, 2:] / 2
        keep = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), .25, .45)
        detections = []
        for index in np.asarray(keep).reshape(-1):
            row, box = rows[index], xywh[index]
            x1, y1, bw, bh = box
            logits = row[6:] @ proto[0].reshape(32, -1)
            mask = cv2.resize(logits.reshape(160, 160), (640, 640), interpolation=cv2.INTER_LINEAR) > 0
            yy, xx = np.ogrid[:640, :640]
            mask &= (xx >= x1) & (xx < x1 + bw) & (yy >= y1) & (yy < y1 + bh)
            mask = cv2.resize(mask[top:top + nh, left:left + nw].astype(np.uint8),
                              (width, height), interpolation=cv2.INTER_NEAREST).astype(bool)
            color, counts = mask_color(image, mask)
            detections.append(dict(color=color, color_pixels=counts, confidence=float(scores[index]),
                                   label=['box', 'cube shaped object'][int(row[4:6].argmax())],
                                   bbox=[float((x1 - left) / ratio), float((y1 - top) / ratio),
                                         float(bw / ratio), float(bh / ratio)],
                                   mask_pixels=int(mask.sum())))
        return dict(detections=detections, inference_seconds=time.monotonic() - started)


def main():
    detector = Detector(sys.argv[1])
    for line in sys.stdin:
        try:
            request = json.loads(line)
            image = cv2.imdecode(np.frombuffer(base64.b64decode(request['jpeg']), np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Invalid JPEG')
            result = detector.detect(image)
        except Exception as exc:
            result = dict(error=str(exc))
        print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
