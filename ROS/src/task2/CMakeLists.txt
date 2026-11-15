cmake_minimum_required(VERSION 3.0.2)
project(task2)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

add_message_files(FILES Student.msg)
add_service_files(FILES QueryStudent.srv)
generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime rospy)

catkin_install_python(PROGRAMS
  scripts/student_publisher.py
  scripts/student_subscriber.py
  scripts/query_student_server.py
  scripts/query_student_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
