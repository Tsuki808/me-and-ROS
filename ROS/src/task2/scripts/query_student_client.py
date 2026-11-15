#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

import rospy
from task2.srv import QueryStudent


def main():
    arguments = rospy.myargv()[1:]
    if len(arguments) != 1:
        sys.stderr.write("Usage: rosrun task2 query_student_client.py STUDENT_ID\n")
        return 2

    # 直接保留命令行字符串，学号不转换为数字。
    student_id = arguments[0]
    rospy.init_node("query_student_client", anonymous=True)
    service_name = "/task2/query_student"
    rospy.loginfo("Waiting for %s ...", service_name)
    rospy.wait_for_service(service_name)
    try:
        query = rospy.ServiceProxy(service_name, QueryStudent)
        response = query(student_id)
    except rospy.ServiceException as error:
        rospy.logerr("Student query failed: %s", error)
        return 1

    if response.found:
        student = response.student
        rospy.loginfo(
            "found=true: name=%s, gender=%s, age=%d, student_id=%s",
            student.name, student.gender, student.age, student.student_id,
        )
    else:
        rospy.loginfo("found=false: student_id=%s", student_id)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
