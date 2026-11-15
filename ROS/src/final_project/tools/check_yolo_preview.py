#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Small source/overlay check; no ROS master, camera, model or GUI is started."""
from __future__ import print_function
import ast
import io
import os
import xml.etree.ElementTree as ET
import cv2
import numpy as np


root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(relative):
    with io.open(os.path.join(root, relative), encoding='utf-8') as stream:
        return stream.read()


source = read('scripts/yolo_preview.py')
assert '\r' not in source
tree = ast.parse(source.encode('utf-8'))
function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name == 'draw_preview')
module = ast.parse('')
module.body = [function]
namespace = dict(cv2=cv2)
eval(compile(module, '<draw_preview>', 'exec'), namespace)
frame = np.zeros((480, 640, 3), dtype=np.uint8)
result = dict(detections=[dict(bbox=[-5, 100, 120, 100], color='red',
                              confidence=.9, label='cube')], inference_seconds=.2)
canvas = namespace['draw_preview'](frame, result, 'red')
assert canvas.shape == frame.shape and not frame.any()
assert tuple(canvas[150, 114]) == (0, 0, 255)
assert namespace['draw_preview'](frame, {}, 'red', 'Waiting').any()
assert 'SimpleActionClient' not in source and 'ServiceProxy' not in source
assert "rospy.Publisher('/final_project/vision/image_annotated'" in source
launch = ET.fromstring(read('launch/simulation.launch').encode('utf-8'))
assert launch.find("arg[@name='yolo_view']").get('default') == 'true'
mode2 = next(group for group in launch.findall('group')
             if group.get('if') == "$(eval int(arg('mode')) == 2)")
preview = mode2.find("group[@if='$(arg yolo_view)']")
assert preview.find("node[@name='yolo_preview']").get('type') == 'yolo_preview.py'
viewer = preview.find("node[@name='yolo_detection_view']")
assert viewer.get('pkg') == 'image_view'
assert viewer.find("remap[@from='image']").get('to') == '/final_project/vision/image_annotated'
assert 'scripts/yolo_preview.py' in read('CMakeLists.txt')
assert 'image_view' in [item.text for item in ET.fromstring(
    read('package.xml').encode('utf-8')).findall('exec_depend')]
print('PASS: overlay drawing, mode-2 launch/dependency wiring; no model or simulation run')
