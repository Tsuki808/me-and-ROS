#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""One compact check of source encoding, ROS XML references and command controls."""
from __future__ import print_function, unicode_literals
import ast
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

if '--static-only' in sys.argv:
    from static_checks import main
    main()
    sys.exit(0)

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(root, 'src'))
from final_project_lib.control import horizontal_ik
sys.path.insert(0, os.path.join(root, 'scripts'))
from voice_control import parse_voice_command

count = 0
python3_files = []
for directory, dirs, files in os.walk(root):
    dirs[:] = [name for name in dirs if name not in ['logs', '__pycache__', '.venv']]
    for name in files:
        path = os.path.join(directory, name)
        if name.endswith(('.png', '.pgm', '.pyc', '.onnx', '.pt', '.ts')):
            continue
        with open(path, 'rb') as stream:
            raw = stream.read()
        text = raw.decode('utf-8')
        assert b'\r' not in raw, 'Non-LF line endings: ' + path
        if name.endswith(('.yaml', '.rviz')):
            yaml.safe_load(text)
        if name.endswith(('.xml', '.launch', '.srdf', '.world', '.xacro')):
            ET.fromstring(raw)
            for relative in re.findall(r'\$\(find final_project\)/([^\s"<>]+)', text):
                if 'optenv' not in relative and ':' not in relative and '$(' not in relative:
                    assert os.path.exists(os.path.join(root, relative)), 'Missing reference: ' + relative
        if name.endswith('.py'):
            if raw.startswith(b'#!/usr/bin/env python3'):
                python3_files.append(path)
            else:
                ast.parse(text.encode('utf-8'), filename=path)
        count += 1
assert parse_voice_command('小车前进。') == ('move', (1., 0.))
assert parse_voice_command('小车微调后退') == ('step', (-1., 0.))
assert parse_voice_command('小车选择 a 台') == ('station', 'A')
assert parse_voice_command('小车准备') == ('action', 'prepare')
assert parse_voice_command('小车完成') == ('action', 'finish')
for utterance in ['前进', '小车不要前进', '小车前进然后抓取', '小车停止，抓取', '小车', '小车去A台']:
    try:
        parse_voice_command(utterance)
        raise AssertionError('Unsafe/unknown voice text accepted: ' + utterance)
    except ValueError:
        pass
robot_xml = subprocess.check_output([sys.executable, os.path.join(root, 'tools', 'render_robot.py')])
robot = ET.fromstring(robot_xml)
sensor = robot.find("gazebo[@reference='camera_rgb_frame']/sensor")
assert sensor.get('type') == 'depth' and sensor.findtext('update_rate') == '10'
for path, value in [('horizontal_fov', '1.0471975512'), ('image/width', '640'),
                    ('image/height', '480'), ('clip/near', '0.05'), ('clip/far', '8')]:
    assert sensor.findtext('camera/' + path) == value
plugin = sensor.find('plugin')
assert plugin.get('filename') == 'libgazebo_ros_openni_kinect.so'
for tag, value in [('cameraName', 'kinect'), ('robotNamespace', '/'),
                   ('frameName', 'camera_rgb_optical_frame'), ('updateRate', '10'), ('hackBaseline', '0'),
                   ('imageTopicName', 'rgb/image_raw'), ('cameraInfoTopicName', 'rgb/camera_info'),
                   ('depthImageTopicName', 'depth/image_raw'), ('depthImageCameraInfoTopicName', 'depth/camera_info'),
                   ('pointCloudTopicName', 'depth/points'), ('pointCloudCutoff', '0.4'), ('pointCloudCutoffMax', '8')]:
    assert plugin.findtext(tag) == value, tag
assert len(robot.findall("link[@name='camera_rgb_optical_frame']")) == 1
assert os.path.isfile('/opt/ros/melodic/lib/' + plugin.get('filename'))
assert robot.find("gazebo[@reference='base_scan']/sensor/plugin/frameName").text == 'base_scan'
assert robot.find("gazebo[@reference='inspection_camera_link']/sensor/camera/image/width").text == '1280'
rviz = yaml.safe_load(open(os.path.join(root, 'config/delivery.rviz')).read())['Visualization Manager']
assert rviz['Global Options']['Frame Rate'] == 15
topics = [display.get('Topic', display.get('Image Topic')) for display in rviz['Displays'] if display['Enabled']]
assert all(topic in topics for topic in ['/odom', '/scan', '/inspection/image_raw', '/kinect/rgb/image_raw', '/kinect/depth/image_raw', '/kinect/depth/points'])
assert ET.parse(os.path.join(root, 'launch/simulation.launch')).find("arg[@name='rviz']").get('default') == 'true'
simulation = ET.parse(os.path.join(root, 'launch/simulation.launch'))
assert simulation.find("node[@name='command_router']") is None
assert simulation.find("node[@name='slam_navigation']").get('if') == "$(eval int(arg('mode')) == 2)"
assert simulation.find("node[@name='mission_server']").get('if') == "$(eval int(arg('mode')) != 2)"
navigation = ET.parse(os.path.join(root, 'launch/navigation.launch'))
assert navigation.find("node[@pkg='gmapping']").get('if') == '$(arg slam)'
assert navigation.find("group[@unless='$(arg slam)']/node[@name='amcl']") is not None
assert navigation.find("node[@name='move_base']/remap[@from='cmd_vel']").get('to') == '/final_project/navigation_cmd_vel'
assert '/move_base_simple/goal' in [tool.get('Topic') for tool in rviz['Tools']]
for obsolete in ['scripts/task.py', 'scripts/command_router.py', 'scripts/send_text.py', 'tools/acceptance.py']:
    assert not os.path.exists(os.path.join(root, obsolete))
world = ET.parse(os.path.join(root, 'worlds/delivery.world'))
boxes, cylinders = [], []
for model in world.findall('world/model'):
    if model.findtext('static') != 'true':
        continue
    link = model.find('link')
    collision = link.find('collision/geometry')
    assert ET.tostring(collision) == ET.tostring(link.find('visual/geometry'))
    x, y = [float(v) for v in model.findtext('pose').split()[:2]]
    if collision.find('box') is not None:
        sx, sy, _ = [float(v) for v in collision.findtext('box/size').split()]
        boxes.append((x, y, sx, sy))
    else:
        cylinders.append((x, y, float(collision.findtext('cylinder/radius'))))
assert len(boxes) == 9 and len(cylinders) == 4
map_cfg = yaml.safe_load(open(os.path.join(root, 'maps/delivery.yaml')).read())
assert map_cfg['origin'] == [-4.5, -3.5, 0.] and map_cfg['resolution'] == .025
with open(os.path.join(root, 'maps', map_cfg['image']), 'rb') as stream:
    magic, dimensions, maximum, pixels = stream.read().split(b'\n', 3)
assert (magic, dimensions, maximum) == (b'P5', b'360 280', b'255')
assert len(pixels) == 360 * 280
for index, pixel in enumerate(bytearray(pixels)):
    x = -4.5 + (index % 360 + .5) * .025
    y = -3.5 + (280 - index // 360 - .5) * .025
    occupied = (any(abs(x - bx) <= sx / 2 and abs(y - by) <= sy / 2 for bx, by, sx, sy in boxes)
                or any((x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2 for cx, cy, radius in cylinders))
    assert pixel == (0 if occupied else 254), 'World/map mismatch at cell %d' % index
assert len(horizontal_ik([.275, 0., .205], robot_xml)) == 4
try:
    horizontal_ik([5., 0., 0.], robot_xml)
    raise AssertionError('Unreachable target was accepted')
except ValueError:
    pass
subprocess.check_call(['python3', '-c', 'import ast,pathlib,sys; [ast.parse(pathlib.Path(p).read_text(encoding="utf-8")) for p in sys.argv[1:]]'] + python3_files)
print('PASS: %d text files, Python 2/3 syntax, XML/references, RGB-D/RViz interfaces, world/map geometry, voice rejection, SLAM launch isolation and IK limits' % count)
