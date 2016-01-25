#!/bin/bash
source ~/catkin_workspace/devel/setup.sh
ROS_PACKAGE_PATH="/home/alessio/quadrotore2_moveit":$ROS_PACKAGE_PATH
roslaunch quadrotore2_moveit plan_and_execute.launch &
sleep 5
rosrun dynamic_reconfigure dynparam set /move_group/trajectory_execution execution_duration_monitoring false
