cmake_minimum_required(VERSION 3.0.2)
project(task3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  tf2_ros
  tf_conversions
  turtlesim
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs rospy tf2_ros tf_conversions turtlesim
)

catkin_install_python(PROGRAMS
  scripts/spawn_turtle.py
  scripts/turtle_tf_broadcaster.py
  scripts/turtle_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md VALIDATION.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
