#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy


if __name__ == "__main__":
    rospy.init_node("hello_ros")
    rospy.loginfo("Hello ROS!")
