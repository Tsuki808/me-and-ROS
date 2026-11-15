#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import Twist


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def finite(value):
    return not (math.isnan(value) or math.isinf(value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class PID(object):
    """PID with filtered error derivative and conditional integration."""

    def __init__(self, settings, angular=False):
        for name in ("kp", "ki", "kd", "integral_limit", "integral_zone",
                     "derivative_tau"):
            value = float(settings[name])
            if not finite(value) or value < 0.0:
                raise ValueError("PID %s must be finite and nonnegative" % name)
            setattr(self, name, value)
        self.angular = angular
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.derivative = 0.0

    def update(self, error, dt, lower, upper):
        if self.previous_error is None:
            raw_derivative = 0.0
        else:
            difference = error - self.previous_error
            if self.angular:
                difference = wrap_angle(difference)
            raw_derivative = difference / dt
        alpha = dt / (self.derivative_tau + dt)
        self.derivative += alpha * (raw_derivative - self.derivative)
        self.previous_error = error

        # Integral separation avoids accumulation during large tracking errors.
        if self.ki == 0.0 or abs(error) > self.integral_zone:
            self.integral = 0.0
            candidate = 0.0
        else:
            candidate = clamp(self.integral + error * dt,
                              -self.integral_limit, self.integral_limit)
        proportional_derivative = self.kp * error + self.kd * self.derivative
        requested = proportional_derivative + self.ki * candidate
        # Reject increments that would drive the effective actuator further
        # into saturation. The linear bounds already include heading gating.
        if not ((requested > upper and error > 0.0) or
                (requested < lower and error < 0.0)):
            self.integral = candidate
        return clamp(proportional_derivative + self.ki * self.integral,
                     lower, upper)


def positive_parameter(name):
    value = float(rospy.get_param("~" + name))
    if not finite(value) or value <= 0.0:
        raise ValueError("%s must be finite and positive" % name)
    return value


def main():
    rospy.init_node("turtle_follower")
    control_rate = positive_parameter("control_rate")
    max_linear = positive_parameter("max_linear_speed")
    max_angular = positive_parameter("max_angular_speed")
    stop_distance = positive_parameter("stop_distance")
    resume_distance = positive_parameter("resume_distance")
    tf_timeout = positive_parameter("tf_timeout")
    max_dt = positive_parameter("max_dt")
    if resume_distance <= stop_distance or max_dt <= 1.0 / control_rate:
        raise ValueError("Require resume_distance > stop_distance and "
                         "max_dt > the nominal control period")
    distance_pid = PID(rospy.get_param("~linear_pid"))
    heading_pid = PID(rospy.get_param("~angular_pid"), angular=True)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    publisher = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=1)

    def stop():
        distance_pid.reset()
        heading_pid.reset()
        publisher.publish(Twist())

    rospy.on_shutdown(stop)
    rate = rospy.Rate(control_rate, reset=True)
    previous_time = None
    arrived = False
    rospy.loginfo("TF2 PID follower: %.1f Hz, stop=%.3f, resume=%.3f",
                  control_rate, stop_distance, resume_distance)

    while not rospy.is_shutdown():
        try:
            transform = buffer.lookup_transform("turtle2", "turtle1", rospy.Time(0))
            now = rospy.Time.now()
            age = (now - transform.header.stamp).to_sec()
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            if not all(finite(value) for value in (x, y, age)):
                raise ValueError("Non-finite transform")
            if age < 0.0 or age > tf_timeout:
                raise ValueError("Transform age %.3f s is outside [0, %.3f]" %
                                 (age, tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError) as error:
            rospy.logwarn_throttle(5.0, "Waiting for turtle transforms: %s", error)
            previous_time = None
            arrived = False
            stop()
            rate.sleep()
            continue

        # Prime the clock after startup/TF loss; never differentiate over a gap.
        if previous_time is None:
            previous_time = now
            stop()
            rate.sleep()
            continue
        dt = (now - previous_time).to_sec()
        previous_time = now
        if dt <= 0.0 or dt > max_dt:
            stop()
            rate.sleep()
            continue

        distance = math.hypot(x, y)
        if distance <= stop_distance:
            arrived = True
        elif distance >= resume_distance:
            arrived = False
        if arrived:
            stop()
            rate.sleep()
            continue

        heading = math.atan2(y, x)
        command = Twist()
        command.angular.z = heading_pid.update(heading, dt, -max_angular, max_angular)
        alignment = max(0.0, math.cos(heading))
        if alignment <= 0.0:
            # The goal is behind turtle2: turn in place without accumulating I.
            distance_pid.reset()
        else:
            command.linear.x = distance_pid.update(
                distance, dt, 0.0, max_linear * alignment,
            )
        if rospy.is_shutdown():
            break
        publisher.publish(command)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("Invalid PID follower configuration: %s", error)
        sys.exit(1)
