#!/usr/bin/env python

# Software License Agreement (BSD License)

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def take_off():
    # publish to cmd_vel
    p = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0.3; twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
    # announce move, and publish the message
    rospy.loginfo("Taking Off !")
    for i in range(20):
        p.publish(twist)
        rospy.sleep(0.1) # 30*0.1 = 3.0
      # create a new message
    twist = Twist()
    # note: everything defaults to 0 in twist, if we don't fill it in, we stop!
    rospy.loginfo("Done")
    p.publish(twist)

def move_group_python_interface_tutorial():
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  #rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("Quadrotore")


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  #rospy.sleep(10)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Name of the end-effector : %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
  print "============ set_planner_id "
  group.set_planner_id('RRTConnectkConfigDefault')
  group.set_planning_time(5)
  group.set_workspace([0, 0, 0, 20, 20, 10])
  goal_Quad = PoseStamped()
  goal_Quad.header.frame_id = "base_link"
  #x, y, and z position
  goal_Quad.pose.position.x = 2.0
  goal_Quad.pose.position.y = 2.0
  goal_Quad.pose.position.z = 3.0
  goal_Quad.pose.orientation.w = 1.0
  #Set the goal state
  group.set_pose_target(goal_Quad, "base_link")
  #Set the start state
  group.set_start_state_to_current_state()
  #Plan a path
  plan_quad = group.plan()
  #Execute plan
  group.execute(plan_quad)
  #robot.set_start_state(RobotState())

  print "============ STOPPING"


if __name__=='__main__':
  try:
    rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
    take_off()
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

