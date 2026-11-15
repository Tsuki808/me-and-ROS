#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One offline source check; intentionally does not pretend to build ROS/C++."""
from __future__ import division, print_function
import ast
import io
import os
import re
import sys
import tempfile
import threading
import types
import zipfile
import xml.etree.ElementTree as ET
import yaml


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, os.path.join(root, 'src'))
    from final_project_lib.autonomy import Completion, scene_anchor, transform_scene, sample_map, validate_saved_map, residual_regions
    from final_project_lib.control import PID

    def read(relative):
        with io.open(os.path.join(root, relative), encoding='utf-8') as stream:
            return stream.read()

    def xml(relative):
        return ET.fromstring(read(relative))

    count = 0
    for directory, dirs, files in os.walk(root):
        dirs[:] = [name for name in dirs if name not in ['logs', '__pycache__', '.venv']]
        for name in files:
            if name.endswith(('.png', '.pgm', '.pyc', '.onnx', '.pt', '.ts')):
                continue
            path = os.path.join(directory, name)
            with open(path, 'rb') as stream:
                raw = stream.read()
            text = raw.decode('utf-8')
            assert b'\r' not in raw, 'Non-LF: ' + path
            if name.endswith(('.yaml', '.rviz')):
                yaml.safe_load(text)
            if name.endswith(('.xml', '.launch', '.srdf', '.world', '.xacro')):
                ET.fromstring(raw)
                for relative in re.findall(r'\$\(find final_project\)/([^\s"<>]+)', text):
                    if '$(' not in relative and ':' not in relative and relative not in ['../../.venv-voice/bin/python', '../../.venv-vision/bin/python']:
                        assert os.path.exists(os.path.join(root, relative)), relative
            if name.endswith('.py'):
                tree = ast.parse(raw, filename=path)
                if not raw.startswith(b'#!/usr/bin/env python3'):
                    forbidden = tuple(getattr(ast, name) for name in ['JoinedStr', 'AsyncFunctionDef', 'Await', 'AnnAssign', 'NamedExpr'] if hasattr(ast, name))
                    assert not any(isinstance(node, forbidden) for node in ast.walk(tree)), path
                    for node in ast.walk(tree):
                        if isinstance(node, (ast.FunctionDef, ast.Lambda)):
                            assert not node.args.kwonlyargs and not getattr(node.args, 'posonlyargs', []), path
                            assert not any(arg.annotation is not None for arg in node.args.args), path
                        if isinstance(node, ast.FunctionDef):
                            assert node.returns is None, path
            count += 1
    course_readme = os.path.abspath(os.path.join(root, '../../../README.md'))
    with open(course_readme, 'rb') as stream:
        course_raw = stream.read()
    course_raw.decode('utf-8')
    assert b'\r' not in course_raw, 'Non-LF: ' + course_readme

    simulation = xml('launch/simulation.launch')
    assert simulation.find("arg[@name='mode']").get('default') == '2'
    assert simulation.find("node[@name='slam_navigation']").get('if') == "$(eval int(arg('mode')) == 2)"
    assert simulation.find("node[@name='mission_server']").get('if') == "$(eval int(arg('mode')) != 2)"
    include = simulation.find("include[@file='$(find final_project)/launch/navigation.launch']")
    assert include.get('unless') == "$(eval int(arg('mode')) == 2)"
    navigation = xml('launch/navigation.launch')
    assert navigation.find("node[@pkg='gmapping']").get('if') == '$(arg slam)'
    assert navigation.find("group[@unless='$(arg slam)']/node[@name='amcl']") is not None
    assert navigation.find("arg[@name='map_file']").get('default') == '$(find final_project)/maps/delivery.yaml'
    assert [navigation.find("arg[@name='%s']" % name).get('default') for name in ['initial_x', 'initial_y', 'initial_yaw']] == ['0.0', '-1.2', '1.57079632679']
    for phase in ['mapping', 'localization']:
        launch = xml('launch/mode2_' + phase + '.launch')
        assert launch.find("include/arg[@name='simple_goal_topic']").get('value') == '/final_project/mode2/unused_simple_goal'
    auto_rviz = yaml.safe_load(read('config/automatic.rviz'))['Visualization Manager']
    assert not any(tool['Class'] in ['rviz/SetGoal', 'rviz/SetInitialPose'] for tool in auto_rviz['Tools'])
    assert '/final_project/exploration/frontiers' in [item.get('Marker Topic') for item in auto_rviz['Displays']]
    topics = [item.get('Topic', item.get('Image Topic')) for item in auto_rviz['Displays'] if item['Enabled']]
    assert all(topic in topics for topic in ['/odom', '/scan', '/inspection/image_raw', '/kinect/rgb/image_raw', '/kinect/depth/image_raw', '/kinect/depth/points'])
    for obsolete in ['scripts/task.py', 'scripts/command_router.py', 'scripts/send_text.py', 'tools/acceptance.py']:
        assert not os.path.exists(os.path.join(root, obsolete))
    assert read('action/Delivery.action').splitlines() == ['string object_id', 'string source', 'string destination', '---',
                'bool success', 'string message', 'float64 elapsed_seconds', '---', 'string stage', 'string detail']
    cmake = read('CMakeLists.txt')
    assert 'Manual.action Delivery.action' in cmake
    for filename in re.findall(r'(?:scripts|tools)/[A-Za-z_]+\.py|src/[A-Za-z_/.]+\.cpp', cmake):
        assert os.path.isfile(os.path.join(root, filename)), filename
    dependencies = [item.text for item in xml('package.xml') if item.tag.endswith('depend')]
    assert all(name in dependencies for name in ['nav_msgs', 'costmap_2d', 'visualization_msgs', 'tf', 'roslaunch', 'rosnode', 'map_server', 'amcl'])

    # Coordinate alignment survives translation, rotation, and SLAM map resizing.
    anchor = scene_anchor([0., -1., 0.], [3., 2., 1.5707963267948966])
    point = transform_scene([0., -1., 0.], anchor, pose=True)
    assert max(abs(a-b) for a, b in zip(point, [3., 2., 1.5707963267948966])) < 1e-9
    point = transform_scene([1., -1., 0.], anchor, pose=True)
    assert max(abs(a-b) for a, b in zip(point, [3., 3., 1.5707963267948966])) < 1e-9
    first = dict(width=2, height=1, resolution=1., origin=[0., 0., 0.], data=[0, 100])
    second = dict(width=3, height=1, resolution=1., origin=[-1., 0., 0.], data=[-1, 0, 100])
    assert sample_map(first, [0., 0., 0.], [0., 0., 2., 1.], 1.) == sample_map(second, [0., 0., 0.], [0., 0., 2., 1.], 1.) == [0, 1]
    completion = Completion(10., .001)
    assert not completion.update(0., 'BLOCKED', 1, True, [0, 1])
    assert not completion.update(1., 'EXHAUSTED', 1, True, [0, 1])
    assert not completion.update(2., 'EXHAUSTED', 0, False, [0, 1])
    assert not completion.update(3., 'EXHAUSTED', 0, True, [0, 1])
    assert not completion.update(12., 'EXHAUSTED', 0, True, [0, 1])
    assert completion.update(13., 'EXHAUSTED', 0, True, [0, 1])
    assert not completion.update(14., 'EXHAUSTED', 0, True, [1, 1])
    assert not completion.update(30., 'EXHAUSTED', 0, True, [1, 1], cancelled=True)
    assert not completion.update(31., 'EXHAUSTED', 0, True, [1, 1])
    baseline = [0] * 1000
    stable = Completion(10., .001)
    assert not stable.update(0., 'EXHAUSTED', 0, True, baseline)
    changed = list(baseline)
    changed[0] = 1
    assert not stable.update(5., 'EXHAUSTED', 0, True, changed)
    assert not stable.update(6., 'EXHAUSTED', 0, True, baseline)
    changed = list(baseline)
    changed[1] = 1
    assert not stable.update(9., 'EXHAUSTED', 0, True, changed)
    assert not stable.update(10., 'EXHAUSTED', 0, True, changed)
    # Small residual holes are explicit exceptions; large/unbounded failures stay failures.
    small = dict(width=9, height=5, resolution=.1, origin=[0., 0., 0.], data=[0] * 45)
    for index in [20, 21, 29, 30, 24, 25, 33, 34]:
        small['data'][index] = -1
    before = list(small['data'])
    points = [[.25, .25], [.35, .35], [.65, .25]]
    residuals = residual_regions(small, points, [0., 0., 0.], [0., 0., .9, .5], .10, .30)
    assert residuals['accepted'] and len(residuals['component_areas_m2']) == 2
    assert abs(residuals['total_area_m2'] - .08) < 1e-9 and small['data'] == before
    assert not residual_regions(small, points, [0., 0., 0.], [0., 0., .9, .5], .03, .30)['accepted']
    assert not residual_regions(small, points, [0., 0., 0.], [0., 0., .9, .5], .10, .07)['accepted']
    edge = dict(small, data=list(small['data']))
    edge['data'][18] = edge['data'][19] = -1
    assert not residual_regions(edge, points, [0., 0., 0.], [0., 0., .9, .5], .10, .30)['accepted']
    assert not residual_regions(small, [], [0., 0., 0.], [0., 0., .9, .5], .10, .30)['accepted']
    relaxed = Completion(10., .001)
    assert not relaxed.update(0., 'BLOCKED', 1, True, [0, 1], residuals=residuals)
    assert relaxed.update(10., 'BLOCKED', 1, True, [0, 1], residuals=residuals)
    assert not relaxed.update(11., 'BLOCKED', 1, False, [0, 1], residuals=residuals)
    assert not relaxed.update(12., 'BLOCKED', 1, True, [0, 1], cancelled=True, residuals=residuals)
    assert not relaxed.update(30., 'ERROR', 1, True, [0, 1], residuals=residuals)
    early = Completion(10., .001)
    assert not early.update(0., 'ACTIVE', 0, True, [0, 1], residuals=residuals)
    assert early.update(10., 'PAUSED', 0, True, [0, 1], residuals=residuals)
    assert not early.update(11., 'PAUSED', 0, True, [0, 1], residuals={'accepted': False})
    coordinator = ast.parse(read('scripts/slam_navigation.py').encode('utf-8'))
    cls = next(node for node in coordinator.body if isinstance(node, ast.ClassDef) and node.name == 'AutomaticRun')
    # No-return rays clear free space for GMapping; raw scan and invalid values stay intact.
    scan_cb = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == 'scan_cb')
    import copy
    import math
    scan_namespace = dict(copy=copy, math=math)
    exec(compile(ast.Module(body=[scan_cb], type_ignores=[]), '<mapping scan>', 'exec'), scan_namespace)
    published = []
    subject = types.SimpleNamespace(mapping_scan=types.SimpleNamespace(publish=published.append))
    original = types.SimpleNamespace(range_max=3.5, ranges=[1., float('inf'), float('-inf'), float('nan'), .05])
    scan_namespace['scan_cb'](subject, original)
    assert subject.scan is original and math.isinf(original.ranges[1])
    assert published[0] is not original and published[0].ranges[:3] == [1., 3.5, float('-inf')]
    assert math.isnan(published[0].ranges[3]) and published[0].ranges[4] == .05
    assert navigation.find("arg[@name='slam_scan_topic']").get('default') == '/scan'
    assert xml('launch/mode2_mapping.launch').find("include/arg[@name='slam_scan_topic']").get('value') == '/final_project/mode2/slam_scan'
    assert xml('launch/mode2_mapping.launch').find("param[@name='/move_base/DWAPlannerROS/min_vel_x']").get('value') == '0.0'
    assert yaml.safe_load(read('config/navigation.yaml'))['DWAPlannerROS']['min_vel_x'] == -.06
    for name in ['global_costmap', 'local_costmap']:
        assert xml('launch/mode2_mapping.launch').find("param[@name='/move_base/%s/inflation/inflation_radius']" % name).get('value') == '0.50'
        assert yaml.safe_load(read('config/navigation.yaml'))[name]['inflation'] == dict(inflation_radius=.35, cost_scaling_factor=8.)
    assert xml('launch/mode2_mapping.launch').find("param[@name='/move_base/local_costmap/resolution']").get('value') == '0.025'
    assert yaml.safe_load(read('config/navigation.yaml'))['local_costmap']['resolution'] == .05
    for key, value in [('oscillation_timeout', '15.0'), ('oscillation_distance', '0.10'), ('clearing_rotation_allowed', 'true')]:
        assert xml('launch/mode2_mapping.launch').find("param[@name='/move_base/%s']" % key).get('value') == value
    for key, value in [('oscillation_timeout', '0.0'), ('oscillation_distance', '0.5')]:
        assert xml('launch/mode2_localization.launch').find("param[@name='/move_base/%s']" % key).get('value') == value
    envelopes = [yaml.safe_load(xml('launch/mode2_mapping.launch').find(
        "rosparam[@param='/move_base/%s/footprint']" % name).text) for name in ['global_costmap', 'local_costmap']]
    radius = yaml.safe_load(read('config/automatic.yaml'))['safe_radius']
    assert radius == .35
    assert envelopes[0] == envelopes[1] and len(envelopes[0]) == 16
    for a, b in zip(envelopes[0], envelopes[0][1:] + envelopes[0][:1]):
        assert abs(a[0]*b[1] - a[1]*b[0]) / math.hypot(b[0]-a[0], b[1]-a[1]) >= radius - 1e-6
    # A late-started node must receive /clock before creating action deadlines.
    delivery_module = ast.parse(read('scripts/automatic_delivery.py'))
    entry = next(node for node in delivery_module.body if isinstance(node, ast.If))
    startup = next(node for node in entry.body if isinstance(node, ast.Try))
    clock = [0., 0., 857.]
    started = []
    def start_mission():
        assert clock[0] > 0., 'Action initialization before the first /clock sample'
        started.append(True)
    startup_env = dict(AutomaticMission=start_mission,
        rospy=types.SimpleNamespace(is_shutdown=lambda: False, spin=lambda: None,
            Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: clock[0]))),
        time=types.SimpleNamespace(sleep=lambda unused: clock.pop(0)))
    exec(compile(ast.Module(body=startup.body, type_ignores=[]), '<delivery clock startup>', 'exec'), startup_env)
    assert started == [True]
    for name in ['stage', 'start_child']:
        method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)
        assert isinstance(method.body[0], ast.Expr) and method.body[0].value.func.attr == 'check'
    # Execute only the three transition guards, with ROS/process bodies unreachable.
    guarded = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in ['check', 'stage', 'start_child']]
    class OfflineCancelled(Exception):
        pass
    namespace = dict(Cancelled=OfflineCancelled, rospy=types.SimpleNamespace(is_shutdown=lambda: False))
    exec(compile(ast.Module(body=guarded, type_ignores=[]), '<transition guards>', 'exec'), namespace)
    subject = type('OfflineTransitions', (), dict((node.name, namespace[node.name]) for node in guarded))()
    subject.cancelled = threading.Event()
    subject.cancelled.set()
    for callback, args in [(subject.stage, ('SHOULD_NOT_ADVANCE', 'cancelled')), (subject.start_child, ('must_not_launch',))]:
        try:
            callback(*args)
        except OfflineCancelled:
            pass
        else:
            raise AssertionError('Cancelled run advanced or started a process')
    pid = PID(dict(kp=1., ki=1., kd=0., limit=.1, integral_limit=.2))
    assert pid.update(1., .1) == .1 and pid.integral == 0.
    with tempfile.TemporaryDirectory(prefix='ros_map_check_') as directory:
        image = os.path.join(directory, 'map.pgm')
        with open(image, 'wb') as stream:
            stream.write(b'P5\n# map snapshot\n2 2\n255\n' + bytes(bytearray([205, 205, 254, 0])))
        metadata = dict(image='map.pgm', resolution=1., origin=[0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)
        snapshot = dict(first, height=2, data=[0, 100, -1, 50])
        validate_saved_map(os.path.join(directory, 'map.yaml'), metadata, snapshot)
        with open(image, 'r+b') as stream:
            stream.seek(-4, os.SEEK_END)
            stream.write(b'\x00')
        try:
            validate_saved_map(os.path.join(directory, 'map.yaml'), metadata, snapshot)
        except ValueError:
            pass
        else:
            raise AssertionError('Corrupted map snapshot accepted')

    # Direct content comparisons, including every original shared method body.
    backup = os.path.abspath(os.path.join(root, '../../backups/before_mode2_auto_mapping_20261020/final_project_before_auto_mapping.zip'))
    with zipfile.ZipFile(backup) as archive:
        def methods(text):
            return dict((m.group(1), m.group(0)) for m in re.finditer(r'^    def ([a-z_]+)\(.*?(?=^    def |^if __name__|\Z)', text, re.M | re.S))
        old, current = methods(archive.read('final_project/scripts/mission_server.py').decode('utf-8')), methods(read('scripts/mission_server.py'))
        shared = [name for name in old if name != '__init__']
        for name in shared:
            expected = old[name]
            if name in ['pick', 'place']:
                expected = expected.replace("self.model('delivery_box', 'world')", "self.model(self.object_model, 'world')")
            assert expected == current[name], 'Shared algorithm changed: ' + name
        def functions(text):
            lines = text.splitlines()
            return dict((node.name, '\n'.join(lines[node.lineno - 1:node.end_lineno]))
                        for node in ast.parse(text).body if isinstance(node, ast.FunctionDef))
        original_control = functions(archive.read('final_project/src/final_project_lib/control.py').decode('utf-8'))
        current_control = functions(read('src/final_project_lib/control.py'))
        assert all(original_control[name] == current_control[name] for name in ['clip', 'horizontal_ik'])
        frozen = ['scripts/keyboard_control.py', 'scripts/voice_control.py', 'voice/xfyun_input.py',
                  'tools/render_robot.py', 'config/mission.yaml', 'config/navigation.yaml', 'config/delivery.rviz', 'action/Manual.action']
        frozen += [name[len('final_project/'):] for name in archive.namelist()
                   if name.startswith(('final_project/maps/', 'final_project/models/', 'final_project/urdf/')) and not name.endswith('/') and not name.endswith('marker.material')]
        for name in frozen:
            with open(os.path.join(root, name), 'rb') as stream:
                assert archive.read('final_project/' + name) == stream.read(), name
        old_world = ET.fromstring(archive.read('final_project/worlds/delivery.world'))
        new_world = xml('worlds/delivery.world')
        models = new_world.find('world')
        for color in ['red', 'green', 'blue']:
            models.remove(models.find("model[@name='delivery_box_%s']" % color))
        assert ET.tostring(old_world) == ET.tostring(new_world), 'Original scene changed'
    colors = yaml.safe_load(read('config/colored_boxes.yaml'))['colored_boxes']
    assert set(colors) == set(['red', 'green', 'blue', 'yellow'])
    assert len(set(item['marker_id'] for item in colors.values())) == 4
    assert len(set(item['model'] for item in colors.values())) == 4
    assert xml('launch/simulation.launch').find("arg[@name='target_color']").get('default') == 'red'
    assert read('srv/SetObjectGrasp.srv').splitlines() == ['string model_name', 'bool attach', '---', 'bool success', 'string message']
    assert 'add_service_files(FILES SetObjectGrasp.srv)' in cmake
    print('PASS: %d text files; UTF-8/LF, Python syntax/compatibility rules, XML/YAML/references, mode isolation and build declarations' % count)
    print('PASS: completion/failure/cancellation rules, shifted map coordinates, frozen-map pixels and restored PID anti-windup')
    print('PASS: direct comparison: %d shared algorithms (only pick/place model identity parameterized), clip/horizontal_ik, %d frozen files and original world unchanged' % (len(shared), len(frozen)))
    print('NOT RUN: VM .venv-voice interpreter, Python 2 execution, ROS imports, xacro/URDF expansion, plugin loading, C++/catkin build or simulation')


if __name__ == '__main__':
    main()
