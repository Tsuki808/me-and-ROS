#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Mode 1 terminal: dead-man driving and explicit, cancellable grasp/place keys."""
from __future__ import print_function
import copy
import math
import os
import select
import sys
import termios
import time
import tty
import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool, String
from final_project.msg import ManualAction, ManualGoal


class Keyboard(object):
    def __init__(self):
        if rospy.get_param('/final_project/operation_mode', 2) != 1:
            raise RuntimeError('Start simulation.launch mode:=1 before using the keyboard')
        self.cfg = rospy.get_param('/final_project')
        self.tf = tf.TransformListener()
        self.station, self.fine, self.attached = 'A', False, False
        self.marker, self.message = None, ''
        self.pub = rospy.Publisher('/final_project/manual_cmd_vel', Twist, queue_size=1)
        self.client = actionlib.SimpleActionClient('/final_project/manual', ManualAction)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_cb, queue_size=1)
        rospy.Subscriber('/final_project/attached', Bool, self.attached_cb, queue_size=1)
        rospy.Subscriber('/final_project/status', String, self.status_cb, queue_size=1)
        if not self.client.wait_for_server(rospy.Duration(15)):
            raise RuntimeError('Manual action server is unavailable')

    def attached_cb(self, message):
        self.attached = message.data

    def status_cb(self, message):
        self.message = message.data

    def marker_cb(self, message):
        for marker in message.markers:
            if marker.id == self.cfg['marker_id']:
                try:
                    pose = copy.deepcopy(marker.pose)
                    if not pose.header.frame_id:
                        pose.header = marker.header
                    smoothed = self.tf.transformPose(self.cfg['base_frame'], pose)
                    # The simulated camera stamp can lead /clock; freshness starts at callback time.
                    smoothed.header.stamp = rospy.Time.now()
                    # Close-range detections jitter by millimetres; smooth the static marker pose.
                    if self.marker is not None:
                        alpha = .25
                        smoothed.pose.position.x = ((1. - alpha) * self.marker.pose.position.x +
                                                    alpha * smoothed.pose.position.x)
                        smoothed.pose.position.y = ((1. - alpha) * self.marker.pose.position.y +
                                                    alpha * smoothed.pose.position.y)
                        smoothed.pose.position.z = ((1. - alpha) * self.marker.pose.position.z +
                                                    alpha * smoothed.pose.position.z)
                    self.marker = smoothed
                except tf.Exception:
                    pass

    def busy(self):
        return self.client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING]

    def stop(self):
        self.pub.publish(Twist())
        if self.busy():
            self.client.cancel_goal()

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

    def run(self):
        print('MODE 1: manually drive A -> B -> home. Keep this terminal focused.')
        print('W/S forward/back, A/D turn, T toggle fine speed, SPACE stop/cancel.')
        print('1 select station A, 2 select station B; R prepare/stow before driving.')
        print('P complete pick, O complete place, F verify delivery and home; Q/Ctrl-C quit.')
        print('Align dx and a within 0.008 m / 0.025 rad before P/O; then back away manually.')
        print('A missing marker prevents picking. Arm actions lock base motion.')
        fd = sys.stdin.fileno()
        previous = termios.tcgetattr(fd)
        move, moved_at, displayed_at, last_message = (0., 0.), 0., 0., ''
        try:
            tty.setraw(fd)
            while not rospy.is_shutdown():
                cycle = time.time()
                ready = select.select([sys.stdin], [], [], .05)[0]
                key = os.read(fd, 1).decode('ascii', 'ignore').lower() if ready else ''
                if ready and not key:
                    break
                if key in ['q', '\x03', '\x04']:
                    break
                if key == ' ':
                    move, moved_at = (0., 0.), 0.
                    self.stop()
                elif key in ['1', '2']:
                    self.station = 'A' if key == '1' else 'B'
                    move, moved_at = (0., 0.), 0.
                elif key == 't':
                    self.fine = not self.fine
                    move, moved_at = (0., 0.), 0.
                elif key in ['r', 'p', 'o', 'f']:
                    move, moved_at = (0., 0.), 0.
                    self.pub.publish(Twist())
                    if not self.busy():
                        operation = {'r': 'prepare', 'p': 'pick', 'o': 'place', 'f': 'finish'}[key]
                        self.client.send_goal(ManualGoal(operation, self.station))
                elif key in ['w', 's', 'a', 'd'] and not self.busy():
                    speed, turn = (.02, .08) if self.fine else (.1, .3)
                    move = {'w': (speed, 0.), 's': (-speed, 0.), 'a': (0., turn), 'd': (0., -turn)}[key]
                    moved_at = time.time()
                command = Twist()
                if not self.busy() and time.time() - moved_at < .25:
                    command.linear.x, command.angular.z = move
                self.pub.publish(command)
                if self.message != last_message:
                    sys.stdout.write('\r\033[2K' + self.message + '\r\n')
                    last_message = self.message
                if time.time() - displayed_at > .25:
                    sys.stdout.write('\r\033[2K' + self.hint()[:79])
                    sys.stdout.flush()
                    displayed_at = time.time()
                time.sleep(max(0., .05 - (time.time() - cycle)))
        finally:
            self.stop()
            termios.tcsetattr(fd, termios.TCSADRAIN, previous)
            print('\r\nKeyboard stopped.')


if __name__ == '__main__':
    if not sys.stdin.isatty():
        sys.exit('Use an interactive VM terminal, or ssh -t, for keyboard_control.py')
    rospy.init_node('keyboard_control')
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0:
        time.sleep(.05)
    try:
        Keyboard().run()
    except (RuntimeError, rospy.ROSException) as exc:
        sys.exit(str(exc))
