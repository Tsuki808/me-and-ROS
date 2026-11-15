#!/usr/bin/env python3
"""Copy attributed ROBOTIS resources and generate the matching world/map."""
import argparse
import pathlib
import shutil
import xml.etree.ElementTree as ET
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference', required=True)
    args = parser.parse_args()
    root = pathlib.Path(__file__).resolve().parents[1]
    reference = pathlib.Path(args.reference)
    upstream = reference / 'turtlebot3_manipulation-melodic-devel'
    urdf = root / 'urdf' / 'upstream'
    urdf.mkdir(parents=True, exist_ok=True)
    for name in ['open_manipulator_with_tb3_waffle.urdf.xacro', 'open_manipulator_with_tb3.gazebo.xacro']:
        text = (upstream / 'open_manipulator_with_tb3_description' / 'urdf' / name).read_text(encoding='utf-8')
        text = text.replace('$(find open_manipulator_with_tb3_description)/urdf/', '$(find final_project)/urdf/upstream/')
        (urdf / name).write_bytes(text.replace('\r\n', '\n').encode('utf-8'))
    semantic = ET.parse(str(upstream / 'open_manipulator_with_tb3_waffle_moveit' / 'config' / 'om_with_tb3.srdf'))
    for joint in semantic.getroot().findall('virtual_joint'):
        semantic.getroot().remove(joint)
    semantic_bytes = ET.tostring(semantic.getroot(), encoding='utf-8')
    (root / 'config' / 'robot.srdf').write_bytes(semantic_bytes.replace(b'\r\n', b'\n') + b'\n')
    shutil.copyfile(str(upstream / 'LICENSE'), str(urdf / 'LICENSE.ROBOTIS'))
    texture_dir = root / 'models' / 'delivery_marker' / 'materials' / 'textures'
    texture_dir.mkdir(parents=True, exist_ok=True)
    simulation = reference / 'turtlebot3_manipulation_simulations-melodic-devel'
    shutil.copyfile(str(simulation / 'open_manipulator_with_tb3_gazebo' / 'models' / 'turtlebot3_rooms' / 'ar_marker' / 'materials' / 'textures' / 'Marker0.png'), str(texture_dir / 'Marker0.png'))
    cfg = yaml.safe_load((root / 'config' / 'mission.yaml').read_text(encoding='utf-8'))
    rectangles = [('north_wall', 0, 3, 8.1, .1, 1), ('south_wall', 0, -3, 8.1, .1, 1),
                  ('west_wall', -4, 0, .1, 6.1, 1), ('east_wall', 4, 0, .1, 6.1, 1),
                  ('left_partition', -2.65, -.4, 2.6, .12, 1),
                  ('right_partition', 2.65, -.4, 2.6, .12, 1),
                  ('central_partition', 0, .7, .12, 2.2, 1)]
    cylinders = [('pillar_%d' % index, x, y, .18, .8)
                 for index, (x, y) in enumerate([(-2.7, -1.7), (2.7, -1.7),
                                                (-2.7, 1.5), (2.7, 1.5)], 1)]
    for name, station in sorted(cfg['stations'].items()):
        rectangles.append(('table_' + name, station['table'][0], station['table'][1], .45, .45, cfg['table_height']))
    parts = ['<?xml version="1.0"?><sdf version="1.6"><world name="delivery">',
             '<include><uri>model://ground_plane</uri></include><include><uri>model://sun</uri></include>',
             '<physics type="ode"><max_step_size>0.002</max_step_size><real_time_update_rate>500</real_time_update_rate></physics>']
    for name, x, y, sx, sy, sz in rectangles:
        shape = '<geometry><box><size>%s %s %s</size></box></geometry>' % (sx, sy, sz)
        color = '.2 .5 .8 1' if name.startswith('table') else '.65 .65 .65 1'
        parts.append('<model name="%s"><static>true</static><pose>%s %s %s 0 0 0</pose><link name="link"><collision name="collision">%s</collision><visual name="visual">%s<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual></link></model>' % (name, x, y, sz / 2, shape, shape, color, color))
    for name, x, y, radius, height in cylinders:
        shape = '<geometry><cylinder><radius>%s</radius><length>%s</length></cylinder></geometry>' % (radius, height)
        color = '.8 .35 .15 1'
        parts.append('<model name="%s"><static>true</static><pose>%s %s %s 0 0 0</pose><link name="link"><collision name="collision">%s</collision><visual name="visual">%s<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual></link></model>' % (name, x, y, height / 2, shape, shape, color, color))
    x, y, z = cfg['stations']['A']['object']
    size = cfg['box_size']
    inertia = .025 * size * size / 6
    parts.append('''<model name="delivery_box"><pose>{x} {y} {z} 0 0 0</pose><link name="box_link">
      <inertial><mass>0.025</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz></inertia></inertial>
      <collision name="box"><geometry><box><size>{s} {s} {s}</size></box></geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>
      <visual name="box"><geometry><box><size>{s} {s} {s}</size></box></geometry><material><ambient>1 .65 .05 1</ambient><diffuse>1 .65 .05 1</diffuse></material></visual>
      <visual name="marker"><pose>0 0 {top} 0 0 0</pose><geometry><plane><normal>0 0 1</normal><size>.05 .05</size></plane></geometry>
      <material><script><uri>model://delivery_marker/materials/scripts</uri><uri>model://delivery_marker/materials/textures</uri><name>Delivery/Marker0</name></script></material></visual>
      </link></model>'''.format(x=x, y=y, z=z + .002, s=size, i=inertia, top=size / 2 + .0003))
    original_box = parts[-1]
    colored = yaml.safe_load((root / 'config' / 'colored_boxes.yaml').read_text(encoding='utf-8'))['colored_boxes']
    for color in ['red', 'green', 'blue']:
        box = colored[color]
        x, y, z = box['position']
        model = original_box.replace('name="delivery_box"', 'name="%s"' % box['model'])
        start, end = model.index('<pose>'), model.index('</pose>')
        model = model[:start] + '<pose>%s %s %s 0 0 0' % (x, y, z + .002) + model[end:]
        model = model.replace('1 .65 .05 1', box['rgba'])
        model = model.replace('Delivery/Marker0', 'Delivery/Marker%d' % box['marker_id'])
        parts.append(model)
    parts.append('</world></sdf>')
    (root / 'worlds').mkdir(exist_ok=True)
    (root / 'worlds' / 'delivery.world').write_bytes(('\n'.join(parts) + '\n').encode('utf-8'))
    resolution, width, height, ox, oy = .025, 360, 280, -4.5, -3.5
    pixels = bytearray()
    for row in range(height):
        y = oy + (height - row - .5) * resolution
        for col in range(width):
            x = ox + (col + .5) * resolution
            occupied = any(abs(x - rx) <= sx / 2 and abs(y - ry) <= sy / 2 for _, rx, ry, sx, sy, _ in rectangles)
            occupied = occupied or any((x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2
                                       for _, cx, cy, radius, _ in cylinders)
            pixels.append(0 if occupied else 254)
    (root / 'maps').mkdir(exist_ok=True)
    (root / 'maps' / 'delivery.pgm').write_bytes(('P5\n%d %d\n255\n' % (width, height)).encode('ascii') + pixels)
    (root / 'maps' / 'delivery.yaml').write_bytes(('image: delivery.pgm\nresolution: %s\norigin: [%s, %s, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n' % (resolution, ox, oy)).encode('ascii'))
    print('Prepared ROBOTIS resources, scene and matching map:', root)


if __name__ == '__main__':
    main()
