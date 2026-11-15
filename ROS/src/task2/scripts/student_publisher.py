#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from task2.msg import Student


def main():
    rospy.init_node("student_publisher")
    student = Student(**rospy.get_param("student"))
    publisher = rospy.Publisher("student_info", Student, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        publisher.publish(student)
        rospy.loginfo("Published student_id=%s", student.student_id)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
