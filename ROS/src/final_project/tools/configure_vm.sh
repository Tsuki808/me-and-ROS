#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "$project_dir/../.." && pwd)"
if [ ! -f /opt/ros/melodic/setup.bash ]; then
  echo 'ROS Melodic must already be installed.' >&2
  exit 1
fi
sudo -n apt-get install -y ros-melodic-moveit ros-melodic-navigation \
  ros-melodic-turtlebot3-description ros-melodic-open-manipulator-description \
  ros-melodic-gazebo-ros-control ros-melodic-joint-trajectory-controller \
  ros-melodic-joint-state-controller ros-melodic-ar-track-alvar \
  ros-melodic-gmapping ros-melodic-joint-state-publisher \
  ros-melodic-robot-state-publisher ros-melodic-controller-manager \
  ros-melodic-gazebo-ros ros-melodic-gazebo-plugins ros-melodic-xacro ros-melodic-rviz \
  ros-melodic-cv-bridge python-yaml xvfb
chmod +x "$project_dir"/scripts/*.py "$project_dir"/tools/render_robot.py
set +u
source /opt/ros/melodic/setup.bash
set -u
cd "$workspace_dir"
catkin_make -j2 -l2
echo 'Simulation dependencies configured; voice remains disabled.'
