#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Mode 2: exploration -> saved-map AMCL -> one A-B-home delivery."""
from __future__ import division
import copy
import datetime
import json
import math
import os
import subprocess
import sys
import threading
import time
import actionlib
import moveit_commander
import roslaunch
import rosnode
import rospkg
import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from final_project.msg import DeliveryAction, DeliveryGoal
from final_project_lib.control import horizontal_ik
from final_project_lib.autonomy import Completion, scene_anchor, transform_scene, sample_map, validate_saved_map, residual_regions


class Cancelled(Exception):
    pass


class AutomaticRun(object):
    def __init__(self):
        if rospy.get_param('/final_project/operation_mode') != 2:
            raise RuntimeError('AutomaticRun requires mode 2')
        self.cfg = rospy.get_param('/final_project')
        if self.cfg['target_color'] not in self.cfg['colored_boxes']:
            raise ValueError('target_color must be red, green, blue or yellow')
        self.auto = self.cfg['automatic']
        self.root = rospkg.RosPack().get_path('final_project')
        self.cancelled, self.finished = threading.Event(), False
        self.children, self.arm = [], None
        self.grid, self.scan, self.odom, self.localized = None, None, None, None
        self.exploration, self.exploration_received = None, 0.
        self.mode = rospy.Publisher('/final_project/control_mode', String, queue_size=1, latch=True)
        self.status = rospy.Publisher('/final_project/status', String, queue_size=5, latch=True)
        self.snapshot = rospy.Publisher('/final_project/mode2/map_snapshot', OccupancyGrid, queue_size=1, latch=True)
        self.mapping_scan = rospy.Publisher('/final_project/mode2/slam_scan', LaserScan, queue_size=1)
        self.initial = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.tf = tf.TransformListener()
        self.nav = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.execution = actionlib.SimpleActionClient('/execute_trajectory', ExecuteTrajectoryAction)
        self.delivery = actionlib.SimpleActionClient('/final_project/delivery', DeliveryAction)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        for topic, kind, field in [('/map', OccupancyGrid, 'grid'),
                                    ('/odom', Odometry, 'odom'), ('/amcl_pose', PoseWithCovarianceStamped, 'localized')]:
            rospy.Subscriber(topic, kind, lambda msg, name=field: setattr(self, name, msg), queue_size=1)
        rospy.Subscriber('/final_project/exploration/status', String, self.exploration_cb, queue_size=1)
        self.cancel_service = rospy.Service('/final_project/mode2/cancel', Trigger, self.cancel)
        rospy.on_shutdown(self.stop)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory = os.path.expanduser('~/.ros/final_project/maps/' + stamp)
        os.makedirs(self.directory)
        self.record = {'operation_mode': 2, 'outcome': 'running', 'stages': [], 'map_file': None}
        self.record['target_color'] = self.cfg['target_color']
        self.mode.publish(String('HOLD'))
        rospy.set_param('/final_project/automatic/localization_ready', False)

    def scan_cb(self, message):
        self.scan = message
        # GMapping discards +inf entirely. A max-range no-return ray clears up
        # to maxUrange (3.45 m), without marking an obstacle at range_max (3.5 m).
        scan = copy.copy(message)
        scan.ranges = [message.range_max if math.isinf(value) and value > 0 else value
                       for value in message.ranges]
        self.mapping_scan.publish(scan)

    def exploration_cb(self, msg):
        try:
            self.exploration = json.loads(msg.data)
            self.exploration_received = rospy.Time.now().to_sec()
        except (ValueError, TypeError):
            self.exploration = None

    def cancel(self, unused):
        if self.finished:
            return TriggerResponse(False, 'This run has already finished')
        self.cancelled.set()
        self.stop()
        return TriggerResponse(True, 'Cancellation requested; no automatic resume')

    def check(self):
        if self.cancelled.is_set() or rospy.is_shutdown():
            raise Cancelled()

    def stage(self, name, detail):
        self.check()
        self.record['stages'].append(dict(stage=name, sim_time=rospy.Time.now().to_sec(), detail=detail))
        self.status.publish(String(name + ': ' + detail))
        rospy.loginfo('%s: %s', name, detail)

    def stop(self):
        try:
            self.mode.publish(String('HOLD'))
            for client in [self.nav, self.delivery, self.execution]:
                client.cancel_all_goals()
            if self.arm is not None:
                self.arm.stop()
        except rospy.ROSException:
            pass

    def start_child(self, filename, arguments=None):
        self.check()
        parent = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'),
                    [(os.path.join(self.root, 'launch', filename), arguments or [])])
        self.children.append(parent)
        parent.start()  # Main thread only: roslaunch owns signal handlers.
        self.check()
        return parent

    def stop_child(self, parent):
        parent.shutdown()
        self.children.remove(parent)

    def wait(self, predicate, seconds, description):
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.check()
            try:
                if predicate():
                    return
            except (tf.Exception, rospy.ROSException):
                pass
            time.sleep(.05)
        raise RuntimeError('Timed out: ' + description)

    def fresh(self, map_required=True):
        now = rospy.Time.now()
        scan = self.scan
        if any(msg is None or abs((now - msg.header.stamp).to_sec()) > self.auto['sensor_timeout']
               for msg in [scan, self.odom]):
            return False
        if not any(not math.isnan(value) and not math.isinf(value) and
                   scan.range_min <= value <= scan.range_max for value in scan.ranges):
            return False
        if map_required and (self.grid is None or abs((now - self.grid.header.stamp).to_sec()) > 3.):
            return False
        try:
            # The scan can arrive one odometry publication before its TF.
            self.tf.waitForTransform('map', scan.header.frame_id, scan.header.stamp, rospy.Duration(.1))
            return abs((rospy.Time.now() - self.tf.getLatestCommonTime('map', self.cfg['base_frame'])).to_sec()) <= self.auto['sensor_timeout']
        except tf.Exception:
            return False

    def robot_pose(self):
        point, rotation = self.tf.lookupTransform('map', self.cfg['base_frame'], rospy.Time(0))
        return [point[0], point[1], tf.transformations.euler_from_quaternion(rotation)[2]]

    def stow(self):
        for client in [self.nav, self.execution]:
            self.wait(lambda: client.wait_for_server(rospy.Duration(.05)), 90., 'navigation/arm startup')
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.arm.set_planning_time(5.)
        self.arm.set_num_planning_attempts(3)
        self.arm.set_max_velocity_scaling_factor(.6)
        self.arm.set_max_acceleration_scaling_factor(.5)
        joints = horizontal_ik([.16, 0., .34], rospy.get_param('/robot_description'))
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(joints)
        plan = self.arm.plan()
        if not plan.joint_trajectory.points:
            raise RuntimeError('No collision-free arm stow trajectory')
        goal = ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self.execution.send_goal(goal)
        self.wait(lambda: self.execution.get_state() in [2, 3, 4, 5, 8, 9], 45., 'arm stow')
        if self.execution.get_state() == GoalStatus.SUCCEEDED:
            self.wait(lambda: self.execution.get_result() is not None, 5., 'arm stow result message')
        if self.execution.get_state() != GoalStatus.SUCCEEDED or self.execution.get_result().error_code.val != 1:
            raise RuntimeError('Arm stow failed')
        self.wait(lambda: max(abs(a - b) for a, b in zip(self.arm.get_current_joint_values(), joints)) <= .05,
                  2., 'arm joint feedback')
        self.wait(self.fresh, 60., 'fresh map/sensors/TF')

    @staticmethod
    def grid_data(msg):
        q = msg.info.origin.orientation
        return dict(width=msg.info.width, height=msg.info.height, resolution=msg.info.resolution,
                    origin=[msg.info.origin.position.x, msg.info.origin.position.y,
                            tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]], data=msg.data)

    def explore(self):
        completion = Completion(self.auto['stable_seconds'], self.auto['stable_change_fraction'])
        rospy.set_param('/final_project/automatic/exploration_hold', False)
        parent = self.start_child('mode2_exploration.launch')
        self.mode.publish(String('NAV'))
        start, stale_since = rospy.Time.now().to_sec(), None
        try:
            while True:
                self.check()
                now = rospy.Time.now().to_sec()
                if now - start > self.auto['exploration_timeout']:
                    raise RuntimeError('Exploration simulation-time limit exceeded')
                state = self.exploration or dict(state='WAITING', failures=0)
                self.record['exploration'] = dict(state, elapsed_sim_s=now - start)
                fresh = self.fresh() and now - self.exploration_received <= 3. and state['state'] != 'WAITING'
                if not fresh:
                    self.mode.publish(String('HOLD'))
                    stale_since = now if stale_since is None else stale_since
                    if now - stale_since > 60.:
                        raise RuntimeError('Exploration sensors/status did not recover')
                elif stale_since is not None:
                    stale_since = None
                residuals = None
                if fresh and state['state'] in ['ACTIVE', 'RETRYING', 'BLOCKED', 'PAUSED']:
                    points = state.get('frontier_points', [])
                    if state.get('frontiers', 0) > 0 and len(points) >= state['frontiers']:
                        residuals = residual_regions(self.grid_data(self.grid), points, self.anchor,
                            self.auto['scene_bounds'], self.auto['residual_component_area'], self.auto['residual_total_area'])
                    self.record['exploration']['residuals'] = residuals
                    if state['state'] == 'BLOCKED' and (residuals is None or not residuals['accepted']):
                        raise RuntimeError('Exploration stopped: BLOCKED; unresolved failures=%d; residual check=%s' %
                                           (state['failures'], residuals))
                    if residuals is not None and residuals['accepted']:
                        rospy.logwarn_throttle(10., 'Accepting bounded residual unknown regions: %s', residuals)
                if state['state'] == 'ERROR':
                    raise RuntimeError('Exploration stopped: %s; unresolved failures=%d' % (state['state'], state['failures']))
                terminal = fresh and (state['state'] == 'EXHAUSTED' or (residuals is not None and residuals['accepted']))
                if fresh:
                    # Only residual acceptance needs PAUSED. Keep ordinary
                    # EXHAUSTED observable throughout its stability window.
                    rospy.set_param('/final_project/automatic/exploration_hold',
                                    residuals is not None and residuals['accepted'])
                self.mode.publish(String('HOLD' if not fresh or terminal else 'NAV'))
                cells = None
                if terminal:
                    cells = sample_map(self.grid_data(self.grid), self.anchor, self.auto['scene_bounds'], .025)
                if completion.update(now, state['state'], state['failures'], fresh, cells, residuals=residuals):
                    self.record['exploration_completion'] = 'small_residuals_accepted' if residuals is not None else 'exhausted'
                    return
                time.sleep(1.)
        finally:
            self.mode.publish(String('HOLD'))
            self.nav.cancel_all_goals()
            self.stop_child(parent)
            rospy.set_param('/final_project/automatic/exploration_hold', False)

    def stop_base(self):
        self.mode.publish(String('HOLD'))
        self.nav.cancel_all_goals()
        window = [None]
        def stopped():
            if not self.fresh(False):
                window[0] = None
                return False
            now = rospy.Time.now().to_sec()
            pose = self.odom.pose.pose
            q = pose.orientation
            yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            current = (now, pose.position.x, pose.position.y, yaw, 0., 0.)
            if window[0] is None:
                window[0] = current
            start, x, y, heading, max_distance, max_angle = window[0]
            distance = math.hypot(pose.position.x - x, pose.position.y - y)
            angle = abs(math.atan2(math.sin(yaw - heading), math.cos(yaw - heading)))
            if distance > .003 or angle > .006:
                window[0] = current
                return False
            window[0] = (start, x, y, heading, max(max_distance, distance), max(max_angle, angle))
            if now - start < .6:
                return False
            self.record['stationary_check'] = dict(duration_sim_s=now - start,
                max_position_excursion_m=window[0][4], max_heading_excursion_rad=window[0][5])
            rospy.loginfo('Base stationary window: %s', self.record['stationary_check'])
            return True
        self.wait(stopped, 20., 'stationary base')

    def save_map(self):
        snapshot, initial_pose = copy.deepcopy(self.grid), self.robot_pose()
        self.snapshot.publish(snapshot)
        prefix = os.path.join(self.directory, 'map')
        with open(os.path.join(self.directory, 'map_saver.log'), 'wb') as log:
            process = subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', prefix,
                        'map:=/final_project/mode2/map_snapshot'], stdout=log, stderr=subprocess.STDOUT)
            try:
                self.wait(lambda: process.poll() is not None, 30., 'map snapshot save')
                if process.returncode:
                    raise RuntimeError('map_saver failed; inspect map_saver.log')
            finally:
                if process.poll() is None:
                    process.terminate()
                    deadline = time.time() + 2.
                    while process.poll() is None and time.time() < deadline:
                        time.sleep(.05)
                    if process.poll() is None:
                        process.kill()
                    process.wait()
        with open(prefix + '.yaml') as stream:
            metadata = yaml.safe_load(stream)
        validate_saved_map(prefix + '.yaml', metadata, self.grid_data(snapshot))
        self.record['slam_stop_pose'] = initial_pose
        self.record['map_file'] = prefix + '.yaml'
        rospy.set_param('/final_project/automatic/map_file', prefix + '.yaml')
        return prefix + '.yaml', initial_pose

    @staticmethod
    def pose_message(point):
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = 'map', rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y = point[:2]
        q = tf.transformations.quaternion_from_euler(0., 0., point[2])
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        return msg

    def localize(self, map_file, pose):
        self.localized, self.grid = None, None
        started = rospy.Time.now()
        self.start_child('mode2_localization.launch', ['map_file:=' + map_file,
                         'initial_x:=' + str(pose[0]), 'initial_y:=' + str(pose[1]), 'initial_yaw:=' + str(pose[2])])
        self.wait(lambda: self.initial.get_num_connections() > 0 and self.grid is not None, 60., 'map_server/AMCL startup')
        message = PoseWithCovarianceStamped()
        target = self.pose_message(pose)
        message.header, message.pose.pose = target.header, target.pose
        for index in [0, 7, 35]:
            message.pose.covariance[index] = .01
        self.initial.publish(message)
        self.wait(lambda: self.localized is not None and self.localized.header.stamp >= started, 30., 'first AMCL pose')
        rospy.wait_for_service('/request_nomotion_update', timeout=15.)
        update = rospy.ServiceProxy('/request_nomotion_update', Empty)
        for unused in range(8):
            self.check()
            previous = self.localized.header.stamp
            update()
            self.wait(lambda: self.localized.header.stamp > previous, 8., 'stationary AMCL update')
        def ready():
            msg = self.localized
            return (self.fresh(False) and msg.header.stamp >= started and
                    abs((rospy.Time.now() - msg.header.stamp).to_sec()) <= 1. and
                    all(0 <= msg.pose.covariance[index] <= .04 for index in [0, 7, 35]) and
                    self.tf.getLatestCommonTime('map', self.cfg['base_frame']) >= started)
        self.wait(ready, 30., 'fresh AMCL pose/covariance and TF')
        corrected = self.robot_pose()
        if (math.hypot(corrected[0] - pose[0], corrected[1] - pose[1]) > .5 or
                abs(math.atan2(math.sin(corrected[2] - pose[2]), math.cos(corrected[2] - pose[2]))) > .5):
            raise RuntimeError('AMCL handoff pose changed excessively')
        self.record['localization_pose'] = corrected
        self.wait(lambda: self.nav.wait_for_server(rospy.Duration(.05)), 60., 'saved-map navigation')
        rospy.wait_for_service('/move_base/make_plan', timeout=30.)
        planner = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        for point in [self.auto['stations']['A']['navigation'], self.auto['stations']['B']['navigation'], self.cfg['home']]:
            self.check()
            target = transform_scene(point, self.anchor, pose=True)
            if not planner(self.pose_message(corrected), self.pose_message(target), 0.).plan.poses:
                raise RuntimeError('Final map has no route to a delivery approach/home pose')
        rospy.set_param('/final_project/automatic/localization_ready', True)

    def deliver(self):
        for key in ['navigation_timeout', 'navigation_handoff_distance', 'navigation_retries',
                    'docking_timeout', 'marker_max_age', 'docking_marker_grace',
                    'docking_stable_seconds', 'docking_position_tolerance',
                    'docking_heading_tolerance']:
            rospy.set_param('/final_project/' + key, self.auto[key])
        for name in ['A', 'B']:
            rospy.set_param('/final_project/stations/' + name + '/navigation', self.auto['stations'][name]['navigation'])
        for name in ['linear', 'angular']:
            rospy.set_param('/final_project/docking/' + name, self.auto['docking'][name])
        rospy.set_param('/final_project/docking/position_tolerance', self.auto['docking_position_tolerance'])
        rospy.set_param('/final_project/docking/heading_tolerance', self.auto['docking_heading_tolerance'])
        self.start_child('mode2_delivery.launch')
        self.wait(lambda: self.delivery.wait_for_server(rospy.Duration(.05)), 90., 'automatic delivery action')
        self.check()
        self.delivery.send_goal(DeliveryGoal(self.cfg['object_id'], 'A', 'B'))
        self.wait(lambda: self.delivery.get_state() in [2, 3, 4, 5, 8, 9], 1200., 'A-B-home delivery')
        if self.delivery.get_state() == GoalStatus.SUCCEEDED:
            self.wait(lambda: self.delivery.get_result() is not None, 5., 'delivery result message')
        result = self.delivery.get_result()
        if self.delivery.get_state() != GoalStatus.SUCCEEDED or result is None or not result.success:
            raise RuntimeError('Delivery failed: ' + (result.message if result else self.delivery.get_goal_status_text()))
        self.record['delivery_message'] = result.message

    def run(self):
        try:
            self.stage('PREPARING', 'stowing arm and waiting for live mapping')
            mapping = self.start_child('mode2_mapping.launch')
            self.stow()
            self.anchor = scene_anchor(self.cfg['home'], self.robot_pose())
            rospy.set_param('/final_project/automatic/scene_to_map', self.anchor)
            self.record['scene_to_map'] = self.anchor
            self.stage('MAPPING', 'automatic frontier exploration; no manual goals')
            self.explore()
            self.stop_base()
            self.stage('SAVING_MAP', 'freezing and verifying final map')
            map_file, pose = self.save_map()
            self.check()
            self.stop_child(mapping)
            self.wait(lambda: all(name not in rosnode.get_node_names() for name in ['/slam_gmapping', '/move_base']),
                      20., 'mapping publishers/navigation exit')
            self.stage('LOCALIZING', 'loading saved map and initializing AMCL')
            self.localize(map_file, pose)
            self.stage('DELIVERING', 'A pick, B place, return home')
            self.deliver()
            self.stage('SUCCEEDED', 'automatic mapping and delivery completed')
            self.record['outcome'] = 'succeeded'
        except Cancelled:
            self.record.update(outcome='cancelled', message='Cancelled; already grasped payload remains attached')
        except Exception as exc:
            self.record.update(outcome='failed', message=str(exc))
            rospy.logerr('Mode 2: %s', exc)
        finally:
            self.finished = True
            self.stop()
            self.status.publish(String(self.record['outcome'].upper() + ': ' + self.record.get('message', 'run complete')))
            with open(os.path.join(self.directory, 'run.json'), 'w') as stream:
                json.dump(self.record, stream, ensure_ascii=True, indent=2)
                stream.write('\n')

    def close(self):
        self.stop()
        for parent in reversed(self.children):
            parent.shutdown()
        self.children = []


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('slam_navigation')
    controller = None
    try:
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0:
            time.sleep(.05)
        controller = AutomaticRun()
        controller.run()
        rospy.spin()  # Keep the final map visible; no automatic repeated run.
    finally:
        if controller is not None:
            controller.close()  # Child process shutdown runs on the main thread.
        moveit_commander.roscpp_shutdown()
