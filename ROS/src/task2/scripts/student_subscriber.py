#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from task2.msg import Student


def receive_student(student):
    rospy.loginfo(
        "Student: name=%s, gender=%s, age=%d, student_id=%s",
        student.name, student.gender, student.age, student.student_id,
    )


if __name__ == "__main__":
    rospy.init_node("student_subscriber")
    subscriber = rospy.Subscriber(
        "student_info", Student, receive_student, queue_size=10,
    )
    rospy.spin()
