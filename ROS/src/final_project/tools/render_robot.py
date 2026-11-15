#!/usr/bin/env python
"""Resolve the model, enable Kinect-compatible RGB-D and add the inspection camera."""
from __future__ import print_function
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
import rospkg


def main():
    root = rospkg.RosPack().get_path('final_project')
    if '--cleanup-ros' in sys.argv[1:]:
        # Parameter commands run before roslaunch starts the new master/nodes.
        subprocess.check_call([sys.executable, os.path.join(root, 'scripts/cleanup_ros.py')], stdout=sys.stderr)
    raw = subprocess.check_output(['rosrun', 'xacro', 'xacro', os.path.join(root, 'urdf/upstream/open_manipulator_with_tb3_waffle.urdf.xacro')])
    robot = ET.fromstring(raw)
    for gazebo in list(robot.findall('gazebo')):
        if gazebo.get('reference') == 'camera_rgb_frame':
            # Reuse the R200 housing and its existing ROS optical frame.
            sensor = gazebo.find('sensor')
            sensor.set('type', 'depth')
            sensor.set('name', 'kinect_compatible_r200')
            ET.SubElement(sensor, 'update_rate').text = '10'
            camera = sensor.find('camera')
            for path, value in [('horizontal_fov', '1.0471975512'),
                                ('image/width', '640'), ('image/height', '480'),
                                ('clip/near', '0.05'), ('clip/far', '8')]:
                camera.find(path).text = value
            plugin = sensor.find('plugin')
            plugin.set('filename', 'libgazebo_ros_openni_kinect.so')
            plugin.set('name', 'kinect_controller')
            plugin.find('cameraName').text = 'kinect'
            plugin.find('updateRate').text = '10'
            # RGB and registered depth share one optical frame, with no stereo offset.
            plugin.find('hackBaseline').text = '0'
            ET.SubElement(plugin, 'robotNamespace').text = '/'
            for tag, value in [('depthImageTopicName', 'depth/image_raw'),
                               ('depthImageCameraInfoTopicName', 'depth/camera_info'),
                               ('pointCloudTopicName', 'depth/points'),
                               ('pointCloudCutoff', '0.4')]:
                ET.SubElement(plugin, tag).text = value
            ET.SubElement(plugin, 'pointCloudCutoffMax').text = '8'
        for plugin in gazebo.findall('plugin'):
            filename = plugin.get('filename', '')
            if 'diff_drive' in filename:
                plugin.find('commandTopic').text = '/om_with_tb3/cmd_vel'
                ET.SubElement(plugin, 'robotNamespace').text = '/'
            if 'gazebo_ros_control' in filename:
                ET.SubElement(plugin, 'robotParam').text = '/robot_description'
                plugin.find('controlPeriod').text = '0.002'
    extra = ET.fromstring('''<robot>
      <link name="inspection_camera_link"/>
      <joint name="inspection_camera_joint" type="fixed"><parent link="base_link"/><child link="inspection_camera_link"/><origin xyz="0.12 0 0.55" rpy="0 1.0 0"/></joint>
      <link name="inspection_camera_optical_frame"/>
      <joint name="inspection_optical_joint" type="fixed"><parent link="inspection_camera_link"/><child link="inspection_camera_optical_frame"/><origin rpy="-1.57079632679 0 -1.57079632679"/></joint>
      <gazebo reference="inspection_camera_link"><sensor name="inspection" type="camera"><always_on>true</always_on><update_rate>10</update_rate>
        <camera><horizontal_fov>1.2</horizontal_fov><image><width>1280</width><height>960</height><format>R8G8B8</format></image><clip><near>0.03</near><far>8</far></clip></camera>
        <plugin name="inspection_camera" filename="libgazebo_ros_camera.so"><cameraName>inspection</cameraName><frameName>inspection_camera_optical_frame</frameName><imageTopicName>image_raw</imageTopicName><cameraInfoTopicName>camera_info</cameraInfoTopicName><updateRate>10</updateRate></plugin>
      </sensor></gazebo>
      <gazebo reference="end_effector_joint"><preserveFixedJoint>true</preserveFixedJoint></gazebo>
      <gazebo><plugin name="delivery_grasp" filename="libfinal_project_grasp.so"/></gazebo>
    </robot>''')
    for child in extra:
        robot.append(child)
    document = ET.tostring(robot, encoding='utf-8')
    sys.stdout.write(document if sys.version_info[0] == 2 else document.decode('utf-8'))


if __name__ == '__main__':
    main()
