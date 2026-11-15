# -*- coding: utf-8 -*-
"""Small control/kinematics functions independent of ROS execution."""
from __future__ import division, unicode_literals
import math
import xml.etree.ElementTree as ET


def clip(value, limit):
    return max(-limit, min(limit, value))


class PID(object):
    def __init__(self, cfg):
        self.cfg, self.integral, self.previous, self.derivative = cfg, 0., None, 0.

    def update(self, error, dt):
        if dt <= 0 or dt > .5:
            self.previous = error
            return 0.
        derivative = 0. if self.previous is None else (error - self.previous) / dt
        self.derivative = .7 * self.derivative + .3 * derivative
        candidate = clip(self.integral + error * dt, self.cfg['integral_limit'])
        raw = self.cfg['kp'] * error + self.cfg['ki'] * candidate + self.cfg['kd'] * self.derivative
        result = clip(raw, self.cfg['limit'])
        if raw == result or error * raw < 0:
            self.integral = candidate
        self.previous = error
        return result



def horizontal_ik(point, robot_xml):
    """Exact 4-DOF IK for the inspected URDF, with horizontal fingers.

    q2/q3 are solved in the arm plane; q4 = -(q2+q3). Joint offsets
    and limits are read from the robot model rather than duplicated constants.
    """
    root = ET.fromstring(robot_xml)
    joints = dict((j.get('name'), j) for j in root.findall('joint'))
    def xyz(name):
        return [float(v) for v in joints[name].find('origin').get('xyz', '0 0 0').split()]
    origin = [sum(xyz(j)[i] for j in ['base_joint', 'base_fixed', 'joint1', 'joint2']) for i in range(3)]
    vx, _, vz = xyz('joint3')
    first = math.hypot(vx, vz)
    second, wrist = xyz('joint4')[0], xyz('end_effector_joint')[0]
    beta = math.atan2(vz, vx)
    dx, dy = point[0] - origin[0], point[1] - origin[1]
    yaw = math.atan2(dy, dx)
    r, z = math.hypot(dx, dy) - wrist, point[2] - origin[2]
    cosine = (r * r + z * z - first * first - second * second) / (2 * first * second)
    if abs(cosine) > 1.000001:
        raise ValueError('Target outside the horizontal-grasp workspace')
    solutions = []
    for delta in [math.acos(clip(cosine, 1.)), -math.acos(clip(cosine, 1.))]:
        q2 = beta - math.atan2(z, r) + math.atan2(second * math.sin(delta), first + second * math.cos(delta))
        q3 = -beta - delta
        values = [yaw, q2, q3, -q2 - q3]
        limits = [joints['joint%d' % i].find('limit') for i in range(1, 5)]
        if all(float(l.get('lower')) <= q <= float(l.get('upper')) for q, l in zip(values, limits)):
            solutions.append(values)
    if not solutions:
        raise ValueError('No horizontal-grasp IK satisfies the URDF joint limits')
    return min(solutions, key=lambda values: sum(q * q for q in values))
