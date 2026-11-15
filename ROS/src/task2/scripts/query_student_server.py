#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from task2.msg import Student
from task2.srv import QueryStudent, QueryStudentResponse


def main():
    rospy.init_node("query_student_server")
    student = Student(**rospy.get_param("student"))

    def query_student(request):
        found = request.student_id == student.student_id
        rospy.loginfo("Query student_id=%s: found=%s", request.student_id, found)
        return QueryStudentResponse(
            found=found, student=student if found else Student(),
        )

    service = rospy.Service("query_student", QueryStudent, query_student)
    rospy.loginfo("Student query service ready.")
    rospy.spin()


if __name__ == "__main__":
    main()
