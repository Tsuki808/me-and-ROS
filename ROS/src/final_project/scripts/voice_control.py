#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Mode 3: final speech text drives the existing manual velocity/action chain."""
from __future__ import unicode_literals
import copy
import json
import math
import os
import subprocess
import threading
import time
import unicodedata
import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool, String
from final_project.msg import ManualAction, ManualGoal


def parse_voice_command(text):
    if not isinstance(text, type(u'')):
        text = text.decode('utf-8')
    if len(text.encode('utf-8')) > 2048:
        raise ValueError('Utterance exceeds 2048 UTF-8 bytes')
    text = unicodedata.normalize('NFKC', text).strip().rstrip('。！？，.!?,')
    text = ''.join(text.split())
    if not text.startswith('小车'):
        raise ValueError('Say one complete command beginning with 小车')
    command = text[2:]
    directions = {'前进': (1., 0.), '后退': (-1., 0.), '左转': (0., 1.), '右转': (0., -1.)}
    if command in directions:
        return 'move', directions[command]
    if command.startswith('微调') and command[2:] in directions:
        return 'step', directions[command[2:]]
    commands = {'准备': ('action', 'prepare'), '抓取': ('action', 'pick'),
                '放置': ('action', 'place'), '完成': ('action', 'finish'),
                '停止': ('stop', None), '取消': ('stop', None),
                '细调': ('fine', True), '普通': ('fine', False),
                '选择A台': ('station', 'A'), '选择B台': ('station', 'B'),
                '状态': ('status', None), '退出': ('quit', None)}
    if command.upper() not in commands:
        raise ValueError('Unknown, negative or compound command; nothing executed')
    return commands[command.upper()]


class Voice(object):
    def __init__(self):
        if rospy.get_param('/final_project/operation_mode', 2) != 3:
            raise RuntimeError('Start simulation.launch mode:=3 before using voice_control.py')
        self.cfg = rospy.get_param('/final_project')
        self.lock = threading.RLock()
        self.station, self.fine, self.attached = 'A', False, False
        self.marker, self.message = None, ''
        self.move, self.deadline = (0., 0.), 0.
        self.drive_ready, self.finished, self.fault = False, False, False
        self.process = None
        self.tf = tf.TransformListener()
        self.pub = rospy.Publisher('/final_project/manual_cmd_vel', Twist, queue_size=1)
        self.transcripts = rospy.Publisher('/final_project/voice_command', String, queue_size=1)
        self.feedback = rospy.Publisher('/final_project/voice_status', String, queue_size=5, latch=True)
        self.client = actionlib.SimpleActionClient('/final_project/manual', ManualAction)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_cb, queue_size=1)
        rospy.Subscriber('/final_project/attached', Bool, self.attached_cb, queue_size=1)
        rospy.Subscriber('/final_project/status', String, self.status_cb, queue_size=1)
        rospy.Subscriber('/final_project/control_mode', String, self.mode_cb, queue_size=1)
        rospy.Subscriber('/final_project/voice_command', String, self.command, queue_size=1)
        rospy.on_shutdown(self.close)
        # A wall-clock bound also works while Gazebo has not started its clock yet.
        deadline = time.time() + 120.
        while not rospy.is_shutdown() and not self.client.wait_for_server(rospy.Duration(.2)):
            if time.time() > deadline:
                raise RuntimeError('Manual action server is unavailable')

    def attached_cb(self, message):
        self.attached = message.data

    def status_cb(self, message):
        self.message = message.data.decode('utf-8') if isinstance(message.data, bytes) else message.data

    def mode_cb(self, message):
        with self.lock:
            self.drive_ready = message.data == 'MANUAL'
            if not self.drive_ready:
                self.clear_motion()

    def marker_cb(self, message):
        for marker in message.markers:
            if marker.id == self.cfg['marker_id']:
                try:
                    pose = copy.deepcopy(marker.pose)
                    if not pose.header.frame_id:
                        pose.header = marker.header
                    self.marker = self.tf.transformPose(self.cfg['base_frame'], pose)
                except tf.Exception:
                    pass

    def busy(self):
        return self.client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING]

    def clear_motion(self):
        self.move, self.deadline = (0., 0.), 0.
        self.pub.publish(Twist())

    def stop(self):
        self.clear_motion()
        if self.busy():
            self.client.cancel_goal()

    def report(self, text):
        self.feedback.publish(String(text))
        rospy.loginfo(text.encode('utf-8'))

    def command(self, message):
        try:
            text = message.data.decode('utf-8') if isinstance(message.data, bytes) else message.data
            kind, value = parse_voice_command(text)
        except (ValueError, UnicodeError) as exc:
            self.report(exc.args[0])
            return
        with self.lock:
            if self.finished or self.fault:
                return
            if kind in ['stop', 'quit']:
                self.stop()
                self.finished = kind == 'quit'
            elif kind == 'status':
                self.report(self.hint() + ' | ' + self.message)
            elif kind in ['station', 'fine']:
                self.clear_motion()
                if kind == 'station':
                    self.station = value
                else:
                    self.fine = value
            elif kind == 'action':
                self.clear_motion()
                if self.busy():
                    self.report('Arm action busy; command discarded')
                    return
                self.client.send_goal(ManualGoal(value, self.station))
            else:
                if self.busy() or not self.drive_ready:
                    self.clear_motion()
                    self.report('Driving locked; finish the arm action or say 小车准备')
                    return
                speed, turn = (.02, .08) if self.fine or kind == 'step' else (.1, .3)
                self.move = (value[0] * speed, value[1] * turn)
                self.deadline = time.time() + (.25 if kind == 'step' else 10.)
            self.report('Accepted: ' + text)

    def hint(self):
        prefix = '%s %s box=%d' % (self.station, 'FINE' if self.fine else 'FAST', self.attached)
        try:
            position, rotation = self.tf.lookupTransform('map', self.cfg['base_frame'], rospy.Time(0))
            home = self.cfg['home']
            distance = math.hypot(position[0] - home[0], position[1] - home[1])
            yaw = tf.transformations.euler_from_quaternion(rotation)[2] - home[2]
            heading = math.atan2(math.sin(yaw), math.cos(yaw))
            if self.attached:
                target = PointStamped()
                target.header.frame_id = 'map'
                target.point.x, target.point.y, target.point.z = self.cfg['stations'][self.station]['object']
                point = self.tf.transformPoint(self.cfg['base_frame'], target).point
                source = 'TF'
            else:
                pose = self.marker
                if pose is None or not 0 <= (rospy.Time.now() - pose.header.stamp).to_sec() < self.cfg['marker_max_age']:
                    return prefix + ' | no fresh marker | home=%.3fm yaw=%+.3f' % (distance, heading)
                point, source = pose.pose.position, 'V'
            return prefix + ' | %s dx=%+.3fm a=%+.3f | home=%.3fm yaw=%+.3f' % (
                source, point.x - self.cfg['docking']['distance'], math.atan2(point.y, point.x), distance, heading)
        except tf.Exception:
            return prefix + ' | waiting for TF'

    def speech_output(self):
        try:
            for line in iter(self.process.stdout.readline, b''):
                event = json.loads(line.decode('utf-8'))
                if 'final_text' in event:
                    self.transcripts.publish(String(event['final_text']))
                elif event.get('event') == 'listening':
                    self.report('VOICE_LISTENING: say 小车准备, then one command at a time')
                elif event.get('event') == 'error':
                    raise RuntimeError('Speech input failed; check audio/account/network')
        except (ValueError, UnicodeError, RuntimeError):
            with self.lock:
                self.fault = True
                self.stop()

    def run(self):
        if rospy.get_param('~audio', True):
            script = rospy.get_param('~speech_script')
            interpreter = rospy.get_param('~speech_python')
            self.process = subprocess.Popen([interpreter, '-u', script, '--continuous', '--print-only',
                                             '--device', rospy.get_param('~device', 'pulse'),
                                             '--credentials', os.path.expanduser('~/.config/final_project/xfyun.env')],
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            reader = threading.Thread(target=self.speech_output)
            reader.daemon = True
            reader.start()
            # SDK diagnostics can contain connection details; never copy them to ROS logs.
            drain = threading.Thread(target=lambda: self.process.stderr.read())
            drain.daemon = True
            drain.start()
        else:
            self.report('TEXT_VALIDATION_ONLY: microphone and online recognition disabled')
        displayed_at = 0.
        try:
            while not rospy.is_shutdown() and not self.finished:
                cycle = time.time()
                with self.lock:
                    if self.fault or (self.process and self.process.poll() is not None):
                        raise RuntimeError('Speech process failed; base stopped and action cancelled')
                    command = Twist()
                    if self.busy() or not self.drive_ready or cycle >= self.deadline:
                        self.move, self.deadline = (0., 0.), 0.
                    else:
                        command.linear.x, command.angular.z = self.move
                    self.pub.publish(command)
                if cycle - displayed_at >= 2.:
                    self.feedback.publish(String(self.hint() + ' | ' + self.message))
                    displayed_at = cycle
                time.sleep(max(0., .05 - (time.time() - cycle)))
        finally:
            self.close()

    def close(self):
        with self.lock:
            self.finished = True
            self.stop()
        if self.process and self.process.poll() is None:
            self.process.terminate()
            deadline = time.time() + 3.
            while self.process.poll() is None and time.time() < deadline:
                time.sleep(.05)
            if self.process.poll() is None:
                self.process.kill()
            self.process.wait()


if __name__ == '__main__':
    rospy.init_node(str('voice_control'))
    try:
        Voice().run()
    except (RuntimeError, OSError, rospy.ROSException) as exc:
        rospy.logerr('%s', exc)
        raise SystemExit(1)
