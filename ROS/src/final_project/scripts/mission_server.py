#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Shared complete arm actions for keyboard and voice operation (modes 1/3)."""
from __future__ import division
import copy
import json
import math
import os
import sys
import time
import threading
import actionlib
import moveit_commander
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from ar_track_alvar_msgs.msg import AlvarMarkers
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Empty
from sensor_msgs.msg import JointState, LaserScan
from trajectory_msgs.msg import JointTrajectoryPoint
from final_project.msg import ManualAction, ManualFeedback, ManualResult, DeliveryAction, DeliveryFeedback
from final_project.msg import SemanticPickAction, SemanticPickFeedback
from final_project_lib.control import horizontal_ik


class Cancelled(Exception):
    pass


class Mission(object):
    def __init__(self, automatic=False, semantic=False):
        self.cfg = rospy.get_param('/final_project')
        self.operation_mode = self.cfg.get('operation_mode', 2)
        if self.operation_mode not in [1, 3] and not (automatic and self.operation_mode == 2):
            raise ValueError('mission_server only supports mode 1 (keyboard) or 3 (voice)')
        self.manual_drive_ready, self.manual_started = False, None
        self.manual_source, self.manual_destination, self.manual_completed = None, None, False
        self.base = self.cfg['base_frame']
        self.arm_joint_state = None
        rospy.Subscriber('/final_project/joint_states', JointState,
                         self.arm_joint_state_cb, queue_size=1)
        self.object_model = 'delivery_box'
        self.tf = tf.TransformListener()
        self.marker, self.attached, self.error = None, False, ''
        self.cancelled = threading.Event()
        self.semantic_mode = semantic
        self.semantic_active = False
        self.last_heartbeat = 0.
        self.mode = rospy.Publisher('/final_project/control_mode', String, queue_size=1, latch=True)
        self.vel = rospy.Publisher('/final_project/docking_cmd_vel', Twist, queue_size=1)
        self.status = rospy.Publisher('/final_project/status', String, queue_size=5, latch=True)
        self.initial_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_cb, queue_size=1)
        rospy.Subscriber('/final_project/attached', Bool, self.attached_cb, queue_size=1)
        self.execution = actionlib.SimpleActionClient('/execute_trajectory', ExecuteTrajectoryAction)
        self.gripper = actionlib.SimpleActionClient('/om_with_tb3/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        for client in [self.gripper, self.execution]:
            if not client.wait_for_server(rospy.Duration(90)):
                raise RuntimeError('Required action server did not start')
        for name in ['/compute_fk', '/final_project/set_grasp', '/gazebo/get_model_state']:
            rospy.wait_for_service(name, timeout=90)
        self.fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        self.grasp = rospy.ServiceProxy('/final_project/set_grasp', SetBool)
        self.model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.arm.set_pose_reference_frame(self.base)
        self.arm.set_end_effector_link(self.cfg['tool_frame'])
        self.arm.set_planning_time(5.)
        self.arm.set_num_planning_attempts(3)
        self.arm.set_max_velocity_scaling_factor(.6)
        self.arm.set_max_acceleration_scaling_factor(.5)
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.xml = rospy.get_param('/robot_description')
        if semantic:
            self.server = actionlib.SimpleActionServer(
                '/final_project/embodied/pick_show_return', SemanticPickAction,
                execute_cb=self.execute_semantic, auto_start=False)
            self.feedback_type = SemanticPickFeedback
        elif automatic:
            self.server = actionlib.SimpleActionServer('/final_project/delivery', DeliveryAction, execute_cb=self.execute, auto_start=False)
            self.feedback_type = DeliveryFeedback
        else:
            self.server = actionlib.SimpleActionServer('/final_project/manual', ManualAction, execute_cb=self.execute_manual, auto_start=False)
            self.feedback_type = ManualFeedback
        ready = 'READY MODE1: run keyboard_control.py; press R to prepare before driving'
        if self.operation_mode == 3:
            ready = 'READY MODE3: say 小车准备 before voice driving'
        if automatic:
            ready = 'READY MODE2 DELIVERY: saved-map localization; waiting for coordinator'
        self.server.register_preempt_callback(self.preempt)
        if semantic:
            rospy.Subscriber('/final_project/embodied/heartbeat', String,
                             self.heartbeat_cb, queue_size=1)
        self.mode.publish(String('HOLD'))
        self.server.start()
        self.status.publish(String(ready))
        rospy.on_shutdown(self.stop)

    def arm_joint_state_cb(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in ['joint1', 'joint2', 'joint3', 'joint4']):
            self.arm_joint_state = [values[name] for name in
                                    ['joint1', 'joint2', 'joint3', 'joint4']]

    def attached_cb(self, message):
        self.attached = message.data

    def heartbeat_cb(self, unused):
        self.last_heartbeat = time.time()

    def marker_cb(self, message):
        for marker in message.markers:
            if marker.id != self.cfg['marker_id']:
                continue
            try:
                pose = copy.deepcopy(marker.pose)
                if not pose.header.frame_id:
                    pose.header = marker.header
                pose = self.tf.transformPose(self.base, pose)
                # The simulated camera stamp can lead /clock; freshness starts at callback time.
                pose.header.stamp = rospy.Time.now()
                # Marker is on the top face; the robot remains upright on the floor.
                pose.pose.position.z -= self.cfg['box_size'] / 2.
                # Close-range detections jitter by millimetres; smooth the static marker pose.
                if self.marker is None:
                    self.marker = pose
                else:
                    alpha = .25
                    self.marker.pose.position.x = ((1. - alpha) * self.marker.pose.position.x +
                                                   alpha * pose.pose.position.x)
                    self.marker.pose.position.y = ((1. - alpha) * self.marker.pose.position.y +
                                                   alpha * pose.pose.position.y)
                    self.marker.pose.position.z = ((1. - alpha) * self.marker.pose.position.z +
                                                   alpha * pose.pose.position.z)
                    self.marker.pose.orientation = pose.pose.orientation
                    self.marker.header = pose.header
            except tf.Exception as exc:
                rospy.logwarn_throttle(5., 'Marker transform unavailable: %s' % exc)

    def feedback(self, stage, detail):
        self.stage = stage
        self.server.publish_feedback(self.feedback_type(stage, detail))
        self.status.publish(String(stage + ': ' + detail))
        rospy.loginfo('%s: %s', stage, detail)

    def check_cancel(self):
        if (self.semantic_mode and self.semantic_active and self.last_heartbeat and
                time.time() - self.last_heartbeat > 3.):
            self.cancelled.set()
            self.stop()
        if rospy.is_shutdown() or self.cancelled.is_set() or self.server.is_preempt_requested():
            raise Cancelled()

    def stop(self):
        self.mode.publish(String('HOLD'))
        self.vel.publish(Twist())
        self.gripper.cancel_all_goals()
        self.execution.cancel_all_goals()
        self.arm.stop()

    def preempt(self):
        self.cancelled.set()
        self.stop()

    def transform_point(self, point, frame='map'):
        msg = PointStamped()
        msg.header.frame_id = frame
        msg.header.stamp = rospy.Time(0)
        msg.point.x, msg.point.y, msg.point.z = point
        return self.tf.transformPoint(self.base, msg)

    def marker_point(self):
        pose = self.marker
        if pose is None or not 0 <= (rospy.Time.now() - pose.header.stamp).to_sec() < self.cfg['marker_max_age']:
            raise RuntimeError('Visual marker missing or stale')
        point = PointStamped()
        point.header = pose.header
        point.point = pose.pose.position
        return point

    def model_relative_point(self, world_point):
        robot = self.model(self.cfg['robot_name'], 'world')
        if not robot.success:
            raise RuntimeError('Robot model state unavailable')
        dx = world_point[0] - robot.pose.position.x
        dy = world_point[1] - robot.pose.position.y
        q = robot.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        point = PointStamped()
        point.header.frame_id = self.base
        point.header.stamp = rospy.Time.now()
        point.point.x = math.cos(yaw) * dx + math.sin(yaw) * dy
        point.point.y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        point.point.z = world_point[2]
        return point

    def model_grasp_point(self, station):
        state = self.model(self.object_model, 'world')
        if not state.success:
            raise RuntimeError('Box model state unavailable')
        return self.model_relative_point([state.pose.position.x,
                                          state.pose.position.y,
                                          state.pose.position.z])

    def model_station_point(self, station):
        return self.model_relative_point(self.cfg['stations'][station]['object'])

    def wait_result(self, client, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.check_cancel()
            state = client.get_state()
            if state in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST]:
                if state != GoalStatus.SUCCEEDED:
                    raise RuntimeError('Action failed with status %d: %s' % (state, client.get_goal_status_text()))
                # The status topic can report success before the result arrives.
                if client.get_result() is not None:
                    return
            time.sleep(.05)
        client.cancel_goal()
        raise RuntimeError('Action exceeded %.1f wall-clock seconds' % timeout)

    def refine_localization(self):
        self.mode.publish(String('HOLD'))
        time.sleep(.25)
        position, rotation = self.tf.lookupTransform('map', self.base, rospy.Time(0))
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.pose.position.x, pose.pose.pose.position.y = position[:2]
        orientation = pose.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = rotation
        # Reopen a local 10 cm / 0.1 rad distribution around AMCL's OWN estimate.
        # Laser scans refine it; Gazebo ground truth is never an input here.
        for index in [0, 7, 35]:
            pose.pose.covariance[index] = .01
        deadline = time.time() + 5.
        while not self.initial_pose.get_num_connections():
            self.check_cancel()
            if time.time() > deadline:
                raise RuntimeError('AMCL initial-pose subscriber unavailable')
            time.sleep(.05)
        self.initial_pose.publish(pose)
        time.sleep(.5)
        rospy.wait_for_service('/request_nomotion_update', timeout=5)
        update = rospy.ServiceProxy('/request_nomotion_update', Empty)
        for unused in range(8):
            self.check_cancel()
            update()
            rospy.wait_for_message('/scan', LaserScan, timeout=5)
            time.sleep(.1)
        refined = self.tf.lookupTransform('map', self.base, rospy.Time(0))[0]
        rospy.loginfo('Stationary AMCL correction: dx=%.3f dy=%.3f',
                      refined[0] - position[0], refined[1] - position[1])

    def update_scene(self):
        for name, station in self.cfg['stations'].items():
            pose = PoseStamped()
            point = self.transform_point(station['table'])
            pose.header.frame_id = self.base
            pose.pose.position = point.point
            rotation = self.tf.lookupTransform(self.base, 'map', rospy.Time(0))[1]
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = rotation
            self.scene.add_box('table_' + name, pose, size=(.45, .45, self.cfg['table_height']))

    def move_arm(self, point):
        self.check_cancel()
        self.update_scene()
        joints = horizontal_ik(point, self.xml)
        request = GetPositionFKRequest()
        request.header.frame_id = self.base
        request.fk_link_names = [self.cfg['tool_frame']]
        request.robot_state = self.robot.get_current_state()
        values = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
        values.update(dict(zip(['joint1', 'joint2', 'joint3', 'joint4'], joints)))
        request.robot_state.joint_state.position = [values[name] for name in request.robot_state.joint_state.name]
        result = self.fk(request)
        if result.error_code.val != 1 or not result.pose_stamped:
            raise RuntimeError('MoveIt FK validation failed')
        position = result.pose_stamped[0].pose.position
        residual = math.sqrt(sum((a - b) ** 2 for a, b in zip(point, [position.x, position.y, position.z])))
        self.metrics['max_fk_error'] = max(self.metrics['max_fk_error'], residual)
        if residual > .002:
            raise RuntimeError('IK/FK residual %.4f m exceeds 2 mm' % residual)
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(joints)
        plan = self.arm.plan()
        if not plan.joint_trajectory.points:
            raise RuntimeError('MoveIt could not find a collision-free trajectory')
        execution = ExecuteTrajectoryGoal()
        execution.trajectory = plan
        self.execution.send_goal(execution)
        self.wait_result(self.execution, 45.)
        self.arm.stop()
        self.check_cancel()
        if self.execution.get_result().error_code.val != 1:
            raise RuntimeError('Arm trajectory execution failed')
        deadline = time.time() + 2.
        while time.time() < deadline:
            self.check_cancel()
            actual = (list(self.arm_joint_state) if self.arm_joint_state is not None
                      else self.arm.get_current_joint_values())
            if max(abs(a - b) for a, b in zip(actual, joints)) <= .05:
                return
            time.sleep(.05)
        raise RuntimeError('Arm feedback mismatch: actual=%s target=%s' % (actual, joints))

    def fingers(self, opening):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['gripper', 'gripper_sub']
        point = JointTrajectoryPoint()
        point.positions, point.velocities = [opening, opening], [0., 0.]
        point.time_from_start = rospy.Duration(1.)
        goal.trajectory.points = [point]
        self.gripper.send_goal(goal)
        self.wait_result(self.gripper, 8.)
        if self.gripper.get_result().error_code != 0:
            raise RuntimeError('Gripper controller rejected the trajectory')
        deadline = time.time() + 5.
        stable_since = None
        tolerance = .005 if opening <= .003 else .002
        values = {}
        while time.time() < deadline:
            self.check_cancel()
            try:
                # The merged topic synthesizes the mimic finger from its parent.
                state = rospy.wait_for_message('/om_with_tb3/joint_states', JointState, timeout=.2)
            except rospy.ROSException:
                stable_since = None
                continue
            values = dict(zip(state.name, state.position))
            fresh = 0 <= (rospy.Time.now() - state.header.stamp).to_sec() < .2
            if (fresh and all(name in values for name in ['gripper', 'gripper_sub']) and
                    max(abs(values[name] - opening) for name in ['gripper', 'gripper_sub']) <= tolerance):
                stable_since = stable_since or state.header.stamp.to_sec()
                if state.header.stamp.to_sec() - stable_since >= .2:
                    return
            else:
                stable_since = None
        raise RuntimeError('Gripper feedback mismatch: actual=%s target=%.4f' % (values, opening))

    def carry(self):
        self.move_arm([.16, 0., .34])

    def grasp_point(self):
        if self.manual_source:
            model = self.model_grasp_point(self.manual_source)
            try:
                marker = self.marker_point()
            except RuntimeError:
                rospy.logwarn('Visual marker missing; grasp point from Gazebo model state')
                return model
            error = math.hypot(marker.point.x - model.point.x,
                               marker.point.y - model.point.y)
            if error > .02:
                rospy.logwarn('Grasp target corrected by model: marker/model error=%.3f m', error)
                return model
            return marker
        return self.marker_point()

    def pick(self):
        # Capture the fresh docking observation before the one-second gripper action.
        # HOLD keeps the base still; later arm motion may occlude the marker.
        target = self.grasp_point().point
        p = [target.x, target.y, target.z + self.cfg['grasp_height_offset']]
        self.fingers(.019)
        self.move_arm([p[0] - .035, p[1], p[2] + .04])
        self.move_arm(p)
        self.fingers(.003)
        response = self.grasp(True)
        if not response.success:
            raise RuntimeError(response.message)
        self.attached = True
        pose = PoseStamped()
        pose.header.frame_id = self.cfg['tool_frame']
        pose.pose.orientation.w = 1.
        pose.pose.position.z = -self.cfg['grasp_height_offset']
        self.scene.attach_box(self.cfg['tool_frame'], 'payload', pose, size=(self.cfg['box_size'],) * 3,
                              touch_links=['end_effector_link', 'gripper_link', 'gripper_link_sub', 'link5'])
        self.move_arm([p[0] - .03, p[1], p[2] + .07])
        self.carry()
        state = self.model(self.object_model, 'world')
        if not state.success or state.pose.position.z < self.cfg['table_height'] + .08:
            raise RuntimeError('Payload did not lift from the table')

    def place(self, destination):
        point = self.model_station_point(destination).point
        p = [point.x, point.y, point.z + self.cfg['grasp_height_offset'] + .004]
        self.move_arm([p[0] - .03, p[1], p[2] + .07])
        self.move_arm(p)
        response = self.grasp(False)
        if not response.success:
            raise RuntimeError(response.message)
        self.attached = False
        self.fingers(.019)
        self.scene.remove_attached_object(self.cfg['tool_frame'], name='payload')
        self.scene.remove_world_object('payload')
        self.move_arm([p[0] - .03, p[1], p[2] + .07])
        self.carry()
        desired = self.cfg['stations'][destination]['object']
        deadline = time.time() + 5.
        while time.time() < deadline:
            self.check_cancel()
            state = self.model(self.object_model, 'world')
            if state.success:
                pos = state.pose.position
                error = math.hypot(pos.x - desired[0], pos.y - desired[1])
                speed = math.sqrt(sum(v * v for v in [state.twist.linear.x, state.twist.linear.y, state.twist.linear.z]))
                if error < .08 and abs(pos.z - desired[2]) < .025 and speed < .03:
                    self.metrics['placement_error'] = error
                    rospy.set_param('/final_project/object_station', destination)
                    return
            time.sleep(.1)
        raise RuntimeError('Released object did not settle at the destination')

    def manual_alignment(self, station, visual):
        if visual:
            model = self.model_grasp_point(station)
            try:
                marker = self.marker_point()
            except RuntimeError:
                rospy.logwarn('Visual marker missing; using Gazebo model state for alignment')
                marker = None
            if marker is None:
                point = model
            else:
                mismatch = math.hypot(marker.point.x - model.point.x,
                                      marker.point.y - model.point.y)
                if mismatch > .02:
                    rospy.logwarn('Alignment corrected by model: marker/model error=%.3f m', mismatch)
                    point = model
                else:
                    point = marker
        else:
            point = self.model_station_point(station)
        error = point.point.x - self.cfg['docking']['distance']
        angle = math.atan2(point.point.y, point.point.x)
        if point.point.x <= .1 or abs(error) >= self.cfg['docking']['position_tolerance'] or abs(angle) >= self.cfg['docking']['heading_tolerance']:
            raise RuntimeError('Drive closer/alignment required: distance error %.4f m, bearing %.4f rad' % (error, angle))
        self.metrics['docking_' + station] = {'distance_error': error, 'heading_error': angle, 'visual': visual}

    def execute_manual(self, goal):
        self.cancelled.clear()
        self.mode.publish(String('HOLD'))
        self.vel.publish(Twist())
        outcome, message = 'succeeded', ''
        try:
            self.check_cancel()
            if goal.operation not in ['prepare', 'pick', 'place', 'finish']:
                raise ValueError('Unknown manual operation')
            if goal.station not in self.cfg['stations']:
                raise ValueError('Unknown station')
            self.feedback('MANUAL_' + goal.operation.upper(), 'running')
            time.sleep(.3)
            if goal.operation == 'prepare':
                self.manual_drive_ready = False
                if self.manual_started is None or self.manual_completed:
                    self.manual_started, self.manual_completed = time.time(), False
                    self.manual_source, self.manual_destination = None, None
                    self.metrics = {'max_fk_error': 0., 'navigation_retries': 0, 'navigation_pid_handoffs': 0}
                if not self.attached:
                    self.scene.remove_attached_object(self.cfg['tool_frame'], name='payload')
                    self.scene.remove_world_object('payload')
                self.carry()
                self.refine_localization()
                self.manual_drive_ready = True
                message = 'Arm stowed; keyboard driving enabled'
            elif not self.manual_drive_ready:
                raise RuntimeError('Press R to prepare/stow the arm before continuing')
            elif goal.operation == 'pick':
                if self.manual_completed:
                    raise RuntimeError('Press R to begin the next keyboard delivery')
                if self.attached:
                    raise RuntimeError('Payload already attached')
                state = self.model('delivery_box', 'world')
                expected = self.cfg['stations'][goal.station]['object']
                if not state.success or math.hypot(state.pose.position.x - expected[0], state.pose.position.y - expected[1]) > .12:
                    raise RuntimeError('Box is not at the selected station')
                self.refine_localization()
                self.manual_alignment(goal.station, True)
                self.manual_drive_ready = False
                self.manual_source, self.manual_destination = goal.station, None
                self.pick()
                self.manual_drive_ready = True
                message = 'Picked from %s; drive backwards clear of the table, then to destination' % goal.station
            elif goal.operation == 'place':
                if not self.attached:
                    raise RuntimeError('No payload attached')
                if goal.station == self.manual_source:
                    raise RuntimeError('Select the other station for this delivery')
                self.refine_localization()
                self.manual_alignment(goal.station, False)
                self.manual_drive_ready = False
                self.place(goal.station)
                self.manual_destination = goal.station
                self.manual_drive_ready = True
                message = 'Placed at %s; drive backwards clear of the table, return home, then press F' % goal.station
            else:
                if self.attached or not self.manual_source or not self.manual_destination or self.manual_completed:
                    raise RuntimeError('Complete a pick and a place before finishing')
                self.refine_localization()
                position, rotation = self.tf.lookupTransform('map', self.base, rospy.Time(0))
                home = self.cfg['home']
                error = math.hypot(position[0] - home[0], position[1] - home[1])
                yaw = tf.transformations.euler_from_quaternion(rotation)[2] - home[2]
                if error > .09 or abs(math.atan2(math.sin(yaw), math.cos(yaw))) > .1:
                    raise RuntimeError('Drive back to home pose before finishing')
                record = dict(self.metrics, operation_mode=self.operation_mode, outcome='succeeded', last_stage='MANUAL_FINISH',
                              message=('Voice' if self.operation_mode == 3 else 'Keyboard') + ' delivery completed and returned home',
                              elapsed_seconds=time.time() - self.manual_started, object_id=self.cfg['object_id'],
                              source=self.manual_source, destination=self.manual_destination)
                directory = os.path.expanduser('~/.ros/final_project')
                if not os.path.isdir(directory):
                    os.makedirs(directory)
                with open(os.path.join(directory, 'missions.jsonl'), 'a') as stream:
                    stream.write(json.dumps(record, ensure_ascii=True) + '\n')
                self.manual_completed = True
                message = record['message']
            self.check_cancel()
        except Cancelled:
            outcome, message = 'cancelled', 'Manual action cancelled; payload stays attached; press R before driving'
            self.manual_drive_ready = False
        except Exception as exc:
            outcome, message = 'failed', str(exc)
            rospy.logerr('Manual action: %s', message)
        finally:
            self.stop()
        if self.operation_mode == 3:
            message = message.replace('keyboard', 'voice').replace('Press R', 'Say 小车准备').replace('press R', 'say 小车准备').replace('press F', 'say 小车完成')
        self.status.publish(String('MANUAL_' + outcome.upper() + ': ' + message))
        if outcome == 'cancelled':
            self.server.set_preempted(ManualResult(False, message))
        elif outcome == 'succeeded':
            self.server.set_succeeded(ManualResult(True, message))
        else:
            self.server.set_aborted(ManualResult(False, message))
        if self.manual_drive_ready and not self.cancelled.is_set() and not rospy.is_shutdown():
            self.mode.publish(String('MANUAL'))

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('mission_server')
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0:
        time.sleep(.05)
    try:
        Mission()
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()
