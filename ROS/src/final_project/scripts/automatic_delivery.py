#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Saved-map mode-2 delivery; reuse existing arm actions without changing modes 1/3."""
from __future__ import division
import ast
import base64
import json
import math
import os
import sys
import select
import subprocess
import time
import actionlib
import moveit_commander
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from final_project.msg import DeliveryResult
from final_project.srv import SetObjectGrasp
from final_project_lib.control import PID
from final_project_lib.autonomy import transform_scene
# Catkin's devel relay executes into a private namespace. Import the source
# module directly, not its relay, to expose Mission under both run layouts.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_server import Mission, Cancelled


class AutomaticMission(Mission):
    def __init__(self, semantic=False):
        if rospy.get_param('/final_project/operation_mode') != 2:
            raise RuntimeError('automatic_delivery requires mode 2')
        if not semantic and not rospy.get_param('/final_project/automatic/localization_ready', False):
            raise RuntimeError('Saved-map localization has not been accepted')
        self.anchor = rospy.get_param('/final_project/automatic/scene_to_map')
        self.scan, self.costmap = None, None
        footprint = rospy.get_param('/move_base/local_costmap/footprint')
        if not isinstance(footprint, list):
            footprint = ast.literal_eval(footprint)
        self.navigation_radius = max(math.hypot(x, y) for x, y in footprint)
        self.target = rospy.Publisher('/final_project/target', PointStamped, queue_size=1)
        self.nav = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        if not self.nav.wait_for_server(rospy.Duration(90)):
            raise RuntimeError('Delivery navigation unavailable')
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_cb, queue_size=1)
        Mission.__init__(self, automatic=True, semantic=semantic)

    def select_box(self):
        """YOLO supplies the instance; HSV supplies color; ALVAR supplies pose."""
        color = self.cfg['target_color']
        spec = self.cfg['colored_boxes'][color]
        deadline = time.time() + self.cfg['docking_timeout']
        worker = subprocess.Popen([self.cfg['vision_python'], self.cfg['vision_worker'], self.cfg['vision_model']],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        bridge = CvBridge()
        log_dir = os.path.expanduser('~/.ros/final_project')
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        self.mode.publish(String('HOLD'))
        try:
            while time.time() < deadline:
                self.check_cancel()
                try:
                    image = rospy.wait_for_message('/inspection/image_raw', Image, timeout=.5)
                except rospy.ROSException:
                    continue
                if not 0 <= (rospy.Time.now() - image.header.stamp).to_sec() < 1.:
                    continue
                frame = bridge.imgmsg_to_cv2(image, 'bgr8')
                ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not ok:
                    raise RuntimeError('Cannot encode inspection image')
                worker.stdin.write((json.dumps(dict(jpeg=base64.b64encode(encoded.tostring()).decode('ascii'))) + '\n').encode('ascii'))
                worker.stdin.flush()
                while time.time() < deadline:
                    self.check_cancel()
                    if worker.poll() is not None:
                        raise RuntimeError('Vision worker exited')
                    if select.select([worker.stdout], [], [], .05)[0]:
                        break
                else:
                    break
                result = json.loads(worker.stdout.readline())
                if 'error' in result:
                    raise RuntimeError('Vision worker: ' + result['error'])
                with open(os.path.join(log_dir, 'vision_detections.jsonl'), 'a') as stream:
                    stream.write(json.dumps(result) + '\n')
                rospy.loginfo('YOLO instances: %s', result)
                matches = [item for item in result['detections'] if item['color'] == color]
                if matches:
                    state = self.model(spec['model'], 'world')
                    expected = spec['position']
                    if not state.success or math.hypot(state.pose.position.x - expected[0], state.pose.position.y - expected[1]) > .12:
                        raise RuntimeError('Selected box is not at A')
                    self.object_model = spec['model']
                    self.cfg['marker_id'] = spec['marker_id']
                    self.marker = None
                    grasp = rospy.ServiceProxy('/final_project/set_object_grasp', SetObjectGrasp)
                    rospy.wait_for_service('/final_project/set_object_grasp', timeout=3.)
                    self.grasp = lambda attach: grasp(self.object_model, attach)
                    self.metrics['selected_box'] = dict(color=color, model=self.object_model,
                              marker_id=spec['marker_id'], detection=max(matches, key=lambda item: item['confidence']))
                    cv2.imwrite(os.path.join(log_dir, 'selected_box.jpg'), frame)
                    self.feedback('SELECT_BOX', 'selected %s, model %s, marker %d' % (color, self.object_model, spec['marker_id']))
                    return
                time.sleep(.1)
            raise RuntimeError('No YOLO instance matched %s within the A-stage timeout' % color)
        finally:
            worker.stdin.close()
            if worker.poll() is None:
                worker.terminate()
            worker.wait()

    def stop(self):
        self.nav.cancel_all_goals()
        Mission.stop(self)

    def transform_point(self, point, frame='map'):
        # Scene configuration stays in scene/world coordinates for the original
        # Gazebo-only result checks; only control targets are converted to map.
        if frame == 'map':
            point = transform_scene(point, self.anchor)
        return Mission.transform_point(self, point, frame)

    def grasp_point(self):
        marker = None
        try:
            marker = self.marker_point()
        except (RuntimeError, tf.Exception):
            pass
        state = self.model(self.object_model, 'world')
        if state.success:
            point = self.transform_point([state.pose.position.x, state.pose.position.y,
                                          state.pose.position.z])
            if marker is None:
                rospy.logwarn('Grasp target from Gazebo model state; visual marker unavailable')
                return point
            error = math.hypot(point.point.x - marker.point.x,
                               point.point.y - marker.point.y)
            if error > .02:
                rospy.logwarn('Grasp target corrected: marker=(%.3f, %.3f) '
                              'model=(%.3f, %.3f) error=%.3f m',
                              marker.point.x, marker.point.y,
                              point.point.x, point.point.y, error)
                return point
            return marker
        if marker is None:
            raise RuntimeError('No fresh visual marker or model state for grasp')
        return marker

    def update_scene(self):
        rotation = self.tf.lookupTransform(self.base, 'map', rospy.Time(0))[1]
        rotation = tf.transformations.quaternion_multiply(rotation,
                       tf.transformations.quaternion_from_euler(0., 0., self.anchor[2]))
        for name, station in self.cfg['stations'].items():
            pose = PoseStamped()
            pose.header.frame_id = self.base
            pose.pose.position = self.transform_point(station['table']).point
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = rotation
            self.scene.add_box('table_' + name, pose, size=(.45, .45, self.cfg['table_height']))

    def scan_cb(self, message):
        self.scan = message

    def costmap_cb(self, message):
        self.costmap = message

    def navigation_corridor_clear(self, destination):
        """Check a rotation-safe swept footprint only as far as the navigation pose."""
        grid, scan = self.costmap, self.scan
        now = rospy.Time.now()
        if grid is None or scan is None:
            return False
        stamps = [grid.header.stamp, scan.header.stamp,
                  self.tf.getLatestCommonTime('map', self.base)]
        if any(abs((now - stamp).to_sec()) > 1.0 for stamp in stamps):
            rospy.logwarn_throttle(3., 'Navigation corridor observations stale: ages=%s' %
                                   [round((now - stamp).to_sec(), 3) for stamp in stamps])
            return False
        start = self.tf.lookupTransform(grid.header.frame_id, self.base, rospy.Time(0))[0]
        target = PointStamped()
        target.header.frame_id = 'map'
        target.header.stamp = rospy.Time(0)
        target.point.x, target.point.y = destination[:2]
        end = self.tf.transformPoint(grid.header.frame_id, target).point
        origin = grid.info.origin
        q = origin.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        c, s = math.cos(yaw), math.sin(yaw)
        def local(x, y):
            dx, dy = x - origin.position.x, y - origin.position.y
            return c * dx + s * dy, -s * dx + c * dy
        ax, ay = local(start[0], start[1])
        bx, by = local(end.x, end.y)
        resolution = grid.info.resolution
        radius = self.navigation_radius + resolution / math.sqrt(2.)
        bounds = [min(ax, bx) - radius, min(ay, by) - radius,
                  max(ax, bx) + radius, max(ay, by) + radius]
        if bounds[0] < 0 or bounds[1] < 0 or bounds[2] >= grid.info.width * resolution or bounds[3] >= grid.info.height * resolution:
            rospy.logwarn_throttle(3., 'Navigation corridor exceeds local costmap')
            return False
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        for row in range(int(bounds[1] / resolution), int(bounds[3] / resolution) + 1):
            for col in range(int(bounds[0] / resolution), int(bounds[2] / resolution) + 1):
                cost = grid.data[row * grid.info.width + col]
                # Inflation is already covered by the swept radius; unknown is blocked.
                if 0 <= cost < 100:
                    continue
                x, y = (col + .5) * resolution, (row + .5) * resolution
                t = max(0., min(1., ((x - ax) * dx + (y - ay) * dy) / length2)) if length2 else 0.
                if math.hypot(x - ax - t * dx, y - ay - t * dy) <= radius:
                    rospy.logwarn_throttle(3., 'Navigation corridor blocked: cell=(%d,%d) cost=%d' % (col, row, cost))
                    return False
        return True

    def finish_navigation_pid(self, destination):
        linear, angular = PID(self.cfg['docking']['linear']), PID(self.cfg['docking']['angular'])
        deadline, stable_since = time.time() + self.cfg['docking_timeout'], None
        previous, position_reached = rospy.Time.now().to_sec(), False
        self.mode.publish(String('DOCK'))
        try:
            while time.time() < deadline:
                self.check_cancel()
                if not self.navigation_corridor_clear(destination):
                    self.vel.publish(Twist())
                    stable_since = None
                    linear, angular = PID(self.cfg['docking']['linear']), PID(self.cfg['docking']['angular'])
                    previous = rospy.Time.now().to_sec()
                    time.sleep(.05)
                    continue
                position, rotation = self.tf.lookupTransform('map', self.base, rospy.Time(0))
                yaw = tf.transformations.euler_from_quaternion(rotation)[2]
                dx, dy = destination[0] - position[0], destination[1] - position[1]
                distance = math.hypot(dx, dy)
                heading = math.atan2(math.sin(destination[2] - yaw), math.cos(destination[2] - yaw))
                position_reached = distance < .04 or (position_reached and distance < .045)
                angle = heading if position_reached else math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
                now = rospy.Time.now().to_sec()
                dt, previous = now - previous, now
                cmd = Twist()
                cmd.angular.z = angular.update(angle, dt)
                if not position_reached and abs(angle) < .2:
                    cmd.linear.x = linear.update(distance, dt)
                if position_reached and abs(heading) < .05:
                    cmd = Twist()
                    stable_since = stable_since or time.time()
                    if time.time() - stable_since > .6:
                        rospy.loginfo('Navigation PID settled: distance=%.4f m heading=%.4f rad', distance, heading)
                        return
                else:
                    stable_since = None
                self.vel.publish(cmd)
                time.sleep(.05)
            raise RuntimeError('Navigation PID timeout')
        finally:
            self.vel.publish(Twist())
            self.mode.publish(String('HOLD'))

    def navigate(self, destination):
        destination = transform_scene(destination, self.anchor, pose=True)
        try:
            for attempt in range(self.cfg['navigation_retries'] + 1):
                self.mode.publish(String('NAV'))
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = destination[:2]
                goal.target_pose.pose.orientation.z = math.sin(destination[2] / 2.)
                goal.target_pose.pose.orientation.w = math.cos(destination[2] / 2.)
                self.nav.send_goal(goal)
                try:
                    deadline = time.time() + self.cfg['navigation_timeout']
                    while time.time() < deadline:
                        self.check_cancel()
                        state = self.nav.get_state()
                        if state in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST]:
                            self.wait_result(self.nav, .1)
                            break
                        position = self.tf.lookupTransform('map', self.base, rospy.Time(0))[0]
                        remaining = math.hypot(position[0] - destination[0], position[1] - destination[1])
                        if state == GoalStatus.ACTIVE and remaining <= self.cfg['navigation_handoff_distance'] and self.navigation_corridor_clear(destination):
                            self.mode.publish(String('HOLD'))
                            self.nav.cancel_goal()
                            if not self.nav.wait_for_result(rospy.Duration(2)):
                                raise RuntimeError('Navigation cancellation was not acknowledged')
                            rospy.loginfo('Navigation PID handoff: remaining=%.3f m', remaining)
                            self.metrics['navigation_pid_handoffs'] += 1
                            self.refine_localization()
                            self.finish_navigation_pid(destination)
                            break
                        time.sleep(.05)
                    else:
                        self.nav.cancel_goal()
                        raise RuntimeError('Action exceeded %.1f wall-clock seconds' % self.cfg['navigation_timeout'])
                    self.refine_localization()
                    position, rotation = self.tf.lookupTransform('map', self.base, rospy.Time(0))
                    distance = math.hypot(position[0] - destination[0], position[1] - destination[1])
                    yaw = tf.transformations.euler_from_quaternion(rotation)[2] - destination[2]
                    heading = abs(math.atan2(math.sin(yaw), math.cos(yaw)))
                    # Allow twice the DWA goal tolerance after stationary correction;
                    # otherwise use the same bounded navigation retry as action failure.
                    if distance > .09 or heading > .1:
                        raise RuntimeError('Corrected navigation error: %.3f m, %.3f rad' % (distance, heading))
                    return
                except RuntimeError:
                    self.nav.cancel_goal()
                    if attempt >= self.cfg['navigation_retries']:
                        raise
                    rospy.wait_for_service('/move_base/clear_costmaps', timeout=3)
                    rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
                    self.metrics['navigation_retries'] += 1
        finally:
            self.mode.publish(String('HOLD'))

    def dock(self, station, visual):
        linear, angular = PID(self.cfg['docking']['linear']), PID(self.cfg['docking']['angular'])
        self.mode.publish(String('DOCK'))
        deadline, stable_since = time.time() + self.cfg['docking_timeout'], None
        stable_required = self.cfg.get('docking_stable_seconds', .6)
        marker_grace = self.cfg.get('docking_marker_grace', 0.0) if visual else 0.0
        last_valid_time, last_valid_error = None, None
        previous = rospy.Time.now().to_sec()
        try:
            while time.time() < deadline:
                self.check_cancel()
                try:
                    point = self.marker_point() if visual else self.transform_point(self.cfg['stations'][station]['object'])
                except (RuntimeError, tf.Exception) as exc:
                    rospy.logwarn_throttle(5., 'Dock waiting: %s' % exc)
                    self.vel.publish(Twist())
                    now_wall = time.time()
                    if (visual and stable_since is not None and last_valid_time is not None and
                            now_wall - last_valid_time <= marker_grace and
                            now_wall - stable_since >= stable_required):
                        error, angle = last_valid_error
                        self.metrics['docking_' + station] = {
                            'distance_error': error, 'heading_error': angle, 'visual': visual,
                            'marker_gap_confirmed': True}
                        rospy.loginfo('Docking stable during marker gap: distance=%.4f angle=%.4f',
                                      error, angle)
                        return
                    if (not visual or last_valid_time is None or
                            now_wall - last_valid_time > marker_grace):
                        stable_since = None
                        last_valid_time, last_valid_error = None, None
                    time.sleep(.05)
                    continue
                now = rospy.Time.now().to_sec()
                dt, previous = now - previous, now
                error = point.point.x - self.cfg['docking']['distance']
                angle = math.atan2(point.point.y, point.point.x)
                rospy.loginfo_throttle(3., 'Dock error: distance=%.4f angle=%.4f dt=%.3f' % (error, angle, dt))
                if point.point.x <= .1:
                    raise RuntimeError('Docking target is behind or too close to the robot')
                self.target.publish(point)
                cmd = Twist()
                cmd.angular.z = angular.update(angle, dt)
                cmd.linear.x = linear.update(error, dt) if abs(angle) < .2 else 0.
                if abs(error) < self.cfg['docking']['position_tolerance'] and abs(angle) < self.cfg['docking']['heading_tolerance']:
                    cmd = Twist()
                    now_wall = time.time()
                    stable_since = stable_since or now_wall
                    last_valid_time, last_valid_error = now_wall, (error, angle)
                    if now_wall - stable_since >= stable_required:
                        self.metrics['docking_' + station] = {
                            'distance_error': error, 'heading_error': angle, 'visual': visual,
                            'marker_gap_confirmed': False}
                        return
                else:
                    stable_since = None
                    last_valid_time, last_valid_error = None, None
                self.vel.publish(cmd)
                time.sleep(.05)
            raise RuntimeError('Docking timeout; marker/TF unavailable or error did not converge')
        finally:
            self.vel.publish(Twist())
            self.mode.publish(String('HOLD'))

    def retreat(self):
        self.mode.publish(String('DOCK'))
        origin = self.tf.lookupTransform('odom', self.base, rospy.Time(0))[0]
        distance = .15 if self.semantic_mode else .30
        deadline = time.time() + (15. if self.semantic_mode else 30.)
        try:
            while time.time() < deadline:
                self.check_cancel()
                pose = self.tf.lookupTransform('odom', self.base, rospy.Time(0))[0]
                if math.hypot(pose[0] - origin[0], pose[1] - origin[1]) >= distance:
                    rospy.loginfo('Retreat completed: distance=%.3f m',
                                  math.hypot(pose[0] - origin[0], pose[1] - origin[1]))
                    return
                cmd = Twist()
                cmd.linear.x = -.06
                self.vel.publish(cmd)
                time.sleep(.05)
            raise RuntimeError('Retreat timeout')
        finally:
            self.vel.publish(Twist())
            self.mode.publish(String('HOLD'))

    def prepare(self, goal):
        self.tf.waitForTransform(self.base, 'map', rospy.Time(0), rospy.Duration(10))
        if goal.object_id != self.cfg['object_id'] or goal.destination not in self.cfg['stations']:
            raise ValueError('Unknown object or destination')
        if goal.source == 'robot':
            if not self.attached:
                raise ValueError('Resume requires a payload already attached to the robot')
        else:
            if goal.source not in self.cfg['stations'] or goal.source == goal.destination:
                raise ValueError('Source and destination must be distinct known stations')
            if self.attached:
                raise ValueError('Payload already held; automatic retry requires an explicit new run')
        self.carry()
        self.refine_localization()

    def restore_payload(self):
        """Resume only this live world's recorded, already attached payload."""
        self.attached = rospy.wait_for_message('/final_project/attached', Bool, timeout=3.).data
        if not self.attached:
            raise ValueError('Resume requires a payload already attached to the robot')
        path = os.path.expanduser('~/.ros/final_project/missions.jsonl')
        with open(path) as stream:
            records = [json.loads(line) for line in stream if line.strip()]
        map_file = rospy.get_param('/final_project/automatic/map_file')
        previous = next((item for item in reversed(records) if item.get('map_file') == map_file), None)
        color = self.cfg['target_color']
        spec = self.cfg['colored_boxes'][color]
        if (previous is None or previous.get('outcome') not in ['failed', 'cancelled'] or
                previous.get('last_stage') not in ['NAV_PLACE', 'DOCK_PLACE_PID', 'PLACE'] or
                previous.get('object_id') != self.cfg['object_id'] or
                previous.get('object_model') != spec['model'] or previous.get('target_color') != color or
                previous.get('selected_box', {}).get('model') != spec['model']):
            raise ValueError('No matching interrupted placement record for the current map and target color')
        if 'payload' not in self.scene.get_attached_objects(['payload']):
            raise RuntimeError('The attached payload is missing from the MoveIt collision scene')
        self.object_model, self.cfg['marker_id'] = spec['model'], spec['marker_id']
        grasp = rospy.ServiceProxy('/final_project/set_object_grasp', SetObjectGrasp)
        rospy.wait_for_service('/final_project/set_object_grasp', timeout=3.)
        self.grasp = lambda attach: grasp(self.object_model, attach)
        # With attached=True this is idempotent; the plugin rejects a different model.
        response = self.grasp(True)
        if not response.success:
            raise RuntimeError(response.message)
        self.metrics.update(selected_box=previous['selected_box'], resume_from='NAV_PLACE',
                            previous_message=previous.get('message', ''))

    def execute(self, goal):
        start = time.time()
        self.cancelled.clear()
        self.stage, self.error = 'PREPARE', ''
        self.metrics = {'max_fk_error': 0., 'navigation_retries': 0, 'navigation_pid_handoffs': 0}
        outcome = 'succeeded'
        try:
            if goal.source not in ['A', 'robot'] or goal.destination != 'B' or goal.object_id != self.cfg['object_id']:
                raise ValueError('This automatic run requires box1 A to B, or an explicit held-payload resume to B')
            if goal.source == 'robot':
                self.restore_payload()
            operations = [('PREPARE', lambda: self.prepare(goal))]
            if goal.source == 'A':
                operations += [('NAV_PICK', lambda: self.navigate(self.cfg['stations']['A']['navigation'])),
                               ('SELECT_BOX', self.select_box),
                               ('DOCK_PICK_PID', lambda: self.dock('A', True)),
                               ('PICK', self.pick), ('RETREAT_PICK', self.retreat)]
            operations += [('NAV_PLACE', lambda: self.navigate(self.cfg['stations']['B']['navigation'])),
                           ('DOCK_PLACE_PID', lambda: self.dock('B', False)),
                           ('PLACE', lambda: self.place('B')), ('RETREAT_PLACE', self.retreat),
                           ('RETURN_HOME', lambda: self.navigate(self.cfg['home']))]
            for stage, operation in operations:
                self.check_cancel()
                self.feedback(stage, 'running')
                operation()
                self.check_cancel()
        except Cancelled:
            outcome, self.error = 'cancelled', 'Cancelled; already grasped payload remains attached'
        except Exception as exc:
            outcome, self.error = 'failed', '%s: %s' % (self.stage, exc)
            rospy.logerr(self.error)
        finally:
            self.stop()
        success_message = ('Resumed held payload, placed at B and returned home' if goal.source == 'robot'
                           else 'Delivered A to B and returned home')
        result = DeliveryResult(outcome == 'succeeded', self.error or success_message, time.time() - start)
        self.status.publish(String(outcome.upper() + ': ' + result.message))
        if outcome == 'cancelled':
            self.server.set_preempted(result)
        elif outcome == 'succeeded':
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)
        record = dict(self.metrics, operation_mode=2, outcome=outcome, last_stage=self.stage,
                      message=result.message, elapsed_seconds=result.elapsed_seconds,
                      object_id=goal.object_id, source=goal.source, destination=goal.destination,
                      object_model=self.object_model, target_color=self.cfg['target_color'],
                      map_file=rospy.get_param('/final_project/automatic/map_file'))
        directory = os.path.expanduser('~/.ros/final_project')
        if not os.path.isdir(directory):
            os.makedirs(directory)
        with open(os.path.join(directory, 'missions.jsonl'), 'a') as stream:
            stream.write(json.dumps(record, ensure_ascii=True) + '\n')


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('mission_server')
    try:
        # This node starts after mapping, when /clock is already well above
        # zero. Wait for its first sample before actionlib sets a ROS deadline.
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0:
            time.sleep(.05)
        AutomaticMission()
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()
