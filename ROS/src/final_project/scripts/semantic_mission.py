#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Mode 2.2 semantic pick/show/return action.

The model selects a YOLO candidate; this node alone performs navigation,
marker verification and the existing arm actions.
"""
from __future__ import division
import copy
import json
import math
import threading
import time
from collections import deque

import moveit_commander
import rospy
import tf
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from final_project.msg import SemanticPickResult

from automatic_delivery import AutomaticMission, Cancelled


class SemanticMission(AutomaticMission):
    def __init__(self):
        self.observation_lock = threading.Lock()
        self.observation_history = deque(maxlen=256)
        rospy.Subscriber('/final_project/embodied/observation', String,
                         self.observation_cb, queue_size=1)
        self.semantic_active = False
        self.last_heartbeat = 0.
        auto = rospy.get_param('/final_project/automatic')
        cfg = rospy.get_param('/final_project')
        for key in ['navigation_timeout', 'navigation_handoff_distance', 'navigation_retries', 'docking_timeout']:
            cfg[key] = auto[key]
        docking = dict(cfg.get('docking', {}))
        docking.update(auto['docking'])
        cfg['docking'] = docking
        rospy.set_param('/final_project', cfg)
        locator = tf.TransformListener()
        deadline = time.time() + 90.
        localized = False
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                pose = rospy.wait_for_message('/amcl_pose',
                    __import__('geometry_msgs.msg', fromlist=['PoseWithCovarianceStamped']).PoseWithCovarianceStamped,
                    timeout=1.)
                if pose.header.stamp and abs((rospy.Time.now() - pose.header.stamp).to_sec()) < 1.:
                    locator.waitForTransform('map', cfg['base_frame'], rospy.Time(0), rospy.Duration(1.))
                    localized = True
                    break
            except (rospy.ROSException, tf.Exception):
                pass
        if not localized:
            raise RuntimeError('Fresh AMCL pose and map-to-base TF were not available')
        rospy.set_param('/final_project/automatic/localization_ready', True)
        rospy.set_param('/final_project/automatic/scene_to_map', [0., 0., 0.])
        AutomaticMission.__init__(self, semantic=True)
        self.cancel_service = rospy.Service('/final_project/embodied/cancel',
                                            Trigger, self.cancel_cb)
        self.status.publish(String('READY MODE2.2: waiting for a bound visual-language target'))

    def observation_cb(self, message):
        try:
            data = json.loads(message.data)
            if isinstance(data, dict) and data.get('observation_id'):
                observation = {
                    'observation_id': data['observation_id'],
                    'candidates': data.get('candidates', []),
                    'received_at': time.time()
                }
                with self.observation_lock:
                    self.observation_history.append(observation)
        except (TypeError, ValueError):
            rospy.logwarn_throttle(5., 'Invalid embodied observation JSON')

    def cancel_cb(self, unused):
        self.cancelled.set()
        self.stop()
        return TriggerResponse(True, 'Semantic task cancellation requested')

    def feedback_semantic(self, stage, detail):
        self.stage = stage
        self.server.publish_feedback(self.feedback_type(stage, detail))
        self.status.publish(String(stage + ': ' + detail))

    def bind_candidate(self, goal):
        if goal.color not in self.cfg['colored_boxes']:
            raise ValueError('Unsupported target color: %s' % goal.color)
        with self.observation_lock:
            observation = next((copy.deepcopy(item) for item in
                                reversed(self.observation_history)
                                if item['observation_id'] == goal.observation_id), None)
        if observation is None or time.time() - observation['received_at'] > 300.:
            raise ValueError('Observation expired after 300 seconds; analyze the current camera frame again')
        candidate = next((item for item in observation.get('candidates', [])
                          if item.get('candidate_id') == goal.candidate_id), None)
        if candidate is None or candidate.get('color') != goal.color:
            raise ValueError('Candidate is absent or its color does not match the goal')
        highest = max(observation.get('candidates', []),
                      key=lambda item: float(item.get('confidence', 0.)))
        if candidate.get('candidate_id') != highest.get('candidate_id'):
            raise ValueError('Candidate is not the highest-confidence YOLO detection')
        spec = self.cfg['colored_boxes'][goal.color]
        state = self.model(spec['model'], 'world')
        expected = spec['position']
        if not state.success or math.hypot(state.pose.position.x - expected[0],
                                           state.pose.position.y - expected[1]) > .12:
            raise RuntimeError('Selected colored box is not at the A station')
        self.object_model = spec['model']
        self.cfg['marker_id'] = spec['marker_id']
        self.cfg['stations']['A']['object'] = list(expected)
        self.marker = None
        self.metrics['selected_box'] = dict(color=goal.color, model=self.object_model,
                                            marker_id=spec['marker_id'],
                                            candidate=candidate,
                                            observation_id=goal.observation_id)
        from final_project.srv import SetObjectGrasp
        grasp = rospy.ServiceProxy('/final_project/set_object_grasp', SetObjectGrasp)
        rospy.wait_for_service('/final_project/set_object_grasp', timeout=3.)
        self.grasp = lambda attach: grasp(self.object_model, attach)

    def execute_semantic(self, goal):
        started = time.time()
        self.cancelled.clear()
        self.metrics = {'max_fk_error': 0., 'navigation_retries': 0,
                        'navigation_pid_handoffs': 0}
        outcome, message = 'succeeded', 'Semantic pick/show/return completed'
        self.semantic_active = True
        try:
            if self.attached:
                raise RuntimeError('Robot is already holding a payload')
            if not self.last_heartbeat or time.time() - self.last_heartbeat > 3.:
                raise RuntimeError('Windows operation desk heartbeat is stale')
            self.feedback_semantic('VALIDATE_TARGET', 'checking observation binding and box identity')
            self.check_cancel()
            self.bind_candidate(goal)
            self.feedback_semantic('PREPARE', 'retracting arm and holding the base')
            self.carry()
            self.feedback_semantic('DOCK_PICK_PID', 'visual PID alignment at A')
            self.dock('A', True)
            self.feedback_semantic('PICK', 'executing the existing constrained grasp')
            self.pick()
            self.feedback_semantic('SHOW', 'holding the selected box for 2 seconds')
            show_until = time.time() + 2.
            while time.time() < show_until:
                self.check_cancel()
                time.sleep(.05)
            self.feedback_semantic('PLACE', 'returning the box to its original A position')
            self.place('A')
            self.feedback_semantic('RETREAT', 'retreating and stopping')
            self.retreat()
        except Cancelled:
            outcome, message = 'cancelled', 'Cancelled; an attached payload remains attached'
        except Exception as exc:
            outcome, message = 'failed', '%s: %s' % (getattr(self, 'stage', 'UNKNOWN'), exc)
            rospy.logerr(message)
        finally:
            self.stop()
            self.semantic_active = False
        result = SemanticPickResult(outcome == 'succeeded', message,
                                    time.time() - started)
        self.status.publish(String(outcome.upper() + ': ' + message))
        if outcome == 'succeeded':
            self.server.set_succeeded(result)
        elif outcome == 'cancelled':
            self.server.set_preempted(result)
        else:
            self.server.set_aborted(result)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    rospy.init_node('semantic_mission')
    try:
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0:
            time.sleep(.05)
        SemanticMission()
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()
