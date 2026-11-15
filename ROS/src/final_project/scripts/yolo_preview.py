#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Mode-2 YOLO observer. Publishes images only; never commands the mission."""
from __future__ import division
import base64
import json
import select
import subprocess
import time
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


def draw_preview(frame, result, target_color, status=None):
    canvas = frame.copy()
    palette = {'red': (0, 0, 255), 'green': (0, 255, 0),
               'blue': (255, 100, 0), 'yellow': (0, 255, 255)}
    height, width = canvas.shape[:2]
    for item in result.get('detections', []):
        x, y, w, h = item['bbox']
        x1, y1 = max(0, min(width - 1, int(x))), max(0, min(height - 1, int(y)))
        x2, y2 = max(0, min(width - 1, int(x + w))), max(0, min(height - 1, int(y + h)))
        color = item.get('color') or 'unknown'
        ink = palette.get(color, (180, 180, 180))
        cv2.rectangle(canvas, (x1, y1), (x2, y2), ink, 2)
        label = '%s %s %.2f%s' % (item['label'], color, item['confidence'],
                                  ' TARGET' if color == target_color else '')
        cv2.putText(canvas, label, (x1, max(70, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, ink, 2)
    cv2.rectangle(canvas, (0, 0), (width - 1, 55), (20, 20, 20), -1)
    cv2.putText(canvas, 'YOLOE-26n | target: %s | observation only' % target_color,
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
    detail = status or '%d detections | inference %.3f s' % (
        len(result.get('detections', [])), result.get('inference_seconds', 0.))
    cv2.putText(canvas, detail, (10, 46), cv2.FONT_HERSHEY_SIMPLEX,
                .55, (255, 255, 255), 1)
    return canvas


class YoloPreview:
    def __init__(self):
        if rospy.get_param('/final_project/operation_mode') != 2:
            raise RuntimeError('YOLO preview requires mode 2')
        self.bridge = CvBridge()
        self.latest = None
        self.worker = None
        self.target_color = rospy.get_param('/final_project/target_color')
        self.output = rospy.Publisher('/final_project/vision/image_annotated',
                                      Image, queue_size=1, latch=True)
        self.observation_pub = rospy.Publisher('/final_project/embodied/observation',
                                               String, queue_size=1, latch=True)
        self.observation_index = 0
        # ponytail: keep only the newest frame; do not build an inference backlog.
        self.camera = rospy.Subscriber('/inspection/image_raw', Image, self.image_cb,
                                       queue_size=1, buff_size=2 ** 24)

    def image_cb(self, message):
        self.latest = message

    def publish(self, frame, header=None, result=None, status=None):
        canvas = draw_preview(frame, result or {}, self.target_color, status)
        message = self.bridge.cv2_to_imgmsg(canvas, 'bgr8')
        if header is not None:
            message.header = header
        self.output.publish(message)

    def publish_observation(self, frame, result):
        canvas = draw_preview(frame, result, self.target_color)
        ok, encoded = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return
        self.observation_index += 1
        candidates = []
        for index, item in enumerate(result.get('detections', []), 1):
            candidate = dict(item)
            candidate['candidate_id'] = 'obj_%d' % index
            candidates.append(candidate)
        payload = dict(observation_id='obs_%d' % self.observation_index,
                       stamp=rospy.Time.now().to_sec(),
                       image_jpeg=base64.b64encode(encoded.tostring()).decode('ascii'),
                       candidates=candidates)
        self.observation_pub.publish(String(json.dumps(payload, ensure_ascii=True,
                                                       separators=(',', ':'))))

    def infer(self, frame):
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            raise RuntimeError('Cannot encode preview image')
        request = dict(jpeg=base64.b64encode(jpeg.tostring()).decode('ascii'))
        self.worker.stdin.write((json.dumps(request) + '\n').encode('ascii'))
        self.worker.stdin.flush()
        deadline = time.time() + 40.
        while not rospy.is_shutdown():
            if self.worker.poll() is not None:
                raise RuntimeError('YOLO preview worker exited')
            if select.select([self.worker.stdout], [], [], .05)[0]:
                result = json.loads(self.worker.stdout.readline())
                if 'error' in result:
                    raise RuntimeError(result['error'])
                return result
            if time.time() >= deadline:
                raise RuntimeError('YOLO preview inference timed out')
        return None

    def run(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.publish(frame, status='Loading model / waiting for inspection camera')
        try:
            self.worker = subprocess.Popen(
                [rospy.get_param('/final_project/vision_python'),
                 rospy.get_param('/final_project/vision_worker'),
                 rospy.get_param('/final_project/vision_model')],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
            previous = None
            while not rospy.is_shutdown():
                image = self.latest
                if image is None:
                    time.sleep(.05)
                    continue
                if not 0 <= (rospy.Time.now() - image.header.stamp).to_sec() < 1.:
                    self.publish(frame, status='Waiting for a fresh inspection image')
                    time.sleep(.2)
                    continue
                if image is previous:
                    time.sleep(.05)
                    continue
                previous = image
                frame = self.bridge.imgmsg_to_cv2(image, 'bgr8')
                result = self.infer(frame)
                if result is not None:
                    self.publish(frame, image.header, result)
                    self.publish_observation(frame, result)
        except Exception as exc:
            if not rospy.is_shutdown():
                rospy.logerr('YOLO preview stopped: %s', exc)
                self.publish(frame, status='Preview error: see launch terminal')
        finally:
            if self.worker is not None:
                if self.worker.poll() is None:
                    self.worker.terminate()
                    deadline = time.time() + 1.
                    while self.worker.poll() is None and time.time() < deadline:
                        time.sleep(.05)
                    if self.worker.poll() is None:
                        self.worker.kill()
                self.worker.wait()
                self.worker.stdin.close()
                self.worker.stdout.close()


if __name__ == '__main__':
    rospy.init_node('yolo_preview')
    YoloPreview().run()
