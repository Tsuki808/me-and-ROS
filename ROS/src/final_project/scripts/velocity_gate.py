#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Single owner of base velocity, with startup-selected manual/automatic sources."""
import math
import time
import threading
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Gate(object):
    def __init__(self):
        operation_mode = rospy.get_param('/final_project/operation_mode', 2)
        if operation_mode not in [1, 2, 3]:
            raise ValueError('operation_mode must be 1 (keyboard), 2 (automatic mapping/delivery) or 3 (voice)')
        self.allowed_modes = ['MANUAL'] if operation_mode in [1, 3] else ['NAV', 'DOCK']
        self.mode, self.commands, self.scan = 'HOLD', {}, None
        self.lock = threading.Lock()
        self.pub = rospy.Publisher('/om_with_tb3/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/final_project/control_mode', String, self.set_mode, queue_size=1)
        rospy.Subscriber('/final_project/navigation_cmd_vel', Twist, self.command, callback_args='NAV', queue_size=1)
        rospy.Subscriber('/final_project/docking_cmd_vel', Twist, self.command, callback_args='DOCK', queue_size=1)
        rospy.Subscriber('/final_project/manual_cmd_vel', Twist, self.command, callback_args='MANUAL', queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.laser, queue_size=1)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))

    def set_mode(self, message):
        with self.lock:
            self.mode = message.data if message.data in self.allowed_modes else 'HOLD'
            self.commands.clear()
            self.pub.publish(Twist())

    def command(self, message, mode):
        with self.lock:
            self.commands[mode] = (message, time.time())

    def laser(self, message):
        self.scan = message

    def run(self):
        while not rospy.is_shutdown():
            command = Twist()
            with self.lock:
                cached = self.commands.get(self.mode)
                timeout = .3 if self.mode == 'MANUAL' else 1.0
                if cached and time.time() - cached[1] < timeout:
                    command.linear.x = max(-.08, min(.18, cached[0].linear.x))
                    command.angular.z = max(-.7, min(.7, cached[0].angular.z))
            scan = self.scan
            if scan is None or (rospy.Time.now() - scan.header.stamp).to_sec() > 1.0:
                command = Twist()
            elif command.linear.x:
                centre = 0. if command.linear.x > 0 else math.pi
                clearance = .14 if command.linear.x > 0 else .29
                for i, distance in enumerate(scan.ranges):
                    angle = scan.angle_min + i * scan.angle_increment - centre
                    if abs(math.atan2(math.sin(angle), math.cos(angle))) < .35 and scan.range_min < distance < clearance:
                        command = Twist()
                        break
            self.pub.publish(command)
            time.sleep(.05)


if __name__ == '__main__':
    rospy.init_node('velocity_gate')
    Gate().run()
