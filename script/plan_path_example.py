#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

def plan_path():
  global moveit_commander
  rate = rospy.Rate(1) # 1hz
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Quadrotore")
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=1)
  print "============ Reference frame: %s" % group.get_planning_frame()
  print "============ Robot Groups:"
  print robot.get_group_names()
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============ Generating plan 1"

  group.set_planner_id("RRTConnectkConfigDefault");

  group.allow_replanning(True)

  ybounds = 5
  height = 6
  #returnheight = 4
  group.set_start_state_to_current_state()
  group.set_planning_time(1.0)
  #	group.set_workspace ([-1, -ybounds, 0, 16, ybounds, height])
  group.set_workspace([-10, -10, -10, 20, 20, 20])
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values
  # Joint values:  [-2.9999998042218223, 1.5518367769893308e-06, 0.18246423280801333, -1.1849068583765888e-08, -3.36711925306939e-07, 4.95787472175594e-06, 0.999999999987653]
  # [ x, y, z, 
  group_variable_values[0] = -5 # X
  group_variable_values[1] = 1 # Y
  group_variable_values[2] = 1 # Z
  group.set_joint_value_target(group_variable_values)
  plan1 = group.plan()
  result = group.go(group_variable_values,wait=True)
  print "============ Printing robot state"
  print robot.get_current_state()
  
  group_variable_values[0] = -5 # X
  group_variable_values[1] = -2 # Y
  group_variable_values[2] = 1 # Z
  group.set_joint_value_target(group_variable_values)
  plan1 = group.plan()
  result = group.go(group_variable_values,wait=True)
  print "============ Printing robot state"
  print robot.get_current_state()
  
  group_variable_values[0] = -2 # X
  group_variable_values[1] = -2 # Y
  group.set_joint_value_target(group_variable_values)
  plan1 = group.plan()
  result = group.go(group_variable_values,wait=True)
  print robot.get_current_state()
  print "============ Done"
if __name__ == '__main__':
	global move_publisher, moveit_commander
	print "============ Starting tutorial setup"
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('plan_path', anonymous=True)
	move_publisher = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=1)
	plan_path()
		
	
