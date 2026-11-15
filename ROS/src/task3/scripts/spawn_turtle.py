#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

import rospy
from turtlesim.srv import Spawn


def main():
    rospy.init_node("spawn_turtle")
    rospy.loginfo("Waiting for /spawn ...")
    rospy.wait_for_service("/spawn")
    try:
        spawn = rospy.ServiceProxy("/spawn", Spawn)
        response = spawn(x=1.0, y=1.0, theta=3.14, name="turtle2")
    except rospy.ServiceException as error:
        rospy.logerr("Failed to spawn turtle2: %s", error)
        return 1
    rospy.loginfo("Spawned %s", response.name)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
