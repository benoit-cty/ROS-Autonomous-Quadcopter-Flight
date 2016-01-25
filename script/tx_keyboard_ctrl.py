#! /usr/bin/env python
# Derived by Trancept (https://github.com/trancept) from http://www.romin.upm.es/wiki/index.php?title=ROS%3ATutorials/Control_Your_Robot
# to move a quadcopter with keyboard in a mode 2 airplane TX style
# 
# Released under BSD License
#
# Warning : keys is from AZERTY keyboard !
#
import rospy, math
import numpy as np
import sys, termios, tty, select, os
from geometry_msgs.msg import Twist


# q : Turn left [0,1,0,0]
# d : Turn right [0,-1,0,0]
# z : Up [0,0,1,0]
# s : Down [0,0,-1,0]

# i : Forward [1,0,0,0]
# k : Backward [-1,0,0,0]
# j : Slide left [0,0,0,1]
# l : Slide right [0,0,0,-1]

class KeyTeleop(object):
    #[LinearX , AngularZ , LinearZ, LinearY]
  cmd_bindings = {
				  'z':np.array([0,0,1,0]),
                  's':np.array([0,0,-1,0]),
                  'q':np.array([0,1,0,0]),
				  'd':np.array([0,-1,0,0]),
                  'i':np.array([1,0,0,0]),
                  'k':np.array([-1,0,0,0]),
                  'j':np.array([0,0,0,1]),
                  'l':np.array([0,0,0,-1])
                  }
  def init(self):
    # Save terminal settings
    self.settings = termios.tcgetattr(sys.stdin)
    # Initial values
    self.inc_ratio = 0.1
    self.speed = np.array([0.1, 0.1, 0.1, 0.1])
    self.command = np.array([0, 0, 0, 0])
    self.commandToDo = np.array([0, 0, 0, 0])
    self.update_rate = 10   # Hz
    self.alive = True
    # Setup publisher
    self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
 
  def fini(self):
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
 
  def run(self):
    try:
      self.init()
      self.print_usage()
      r = rospy.Rate(self.update_rate) # Hz
      while not rospy.is_shutdown():
        ch = self.get_key()
        self.process_key(ch)
        self.update()
        r.sleep()
    except rospy.exceptions.ROSInterruptException:
      pass
    finally:
      self.fini()
 
  def print_usage(self):
    msg = """
    Quadcopter keyboard that Publish to /cmd_vel (geometry_msgs/Twist)
    Released under BSD License
    --------------------------------------------------
    h:       Print this menu
    Moving around:
        z               i
    q   s   d       j   k   l
    
    z : Up
    s : Down
    q : Turn left
    d : Turn right

    i : Forward
    k : Backward
    j : Slide left
    l : Slide right

    z : up
    anything else : stop moving
 
    x :   Exit
    --------------------------------------------------
    """
    self.loginfo(msg)
    self.show_status()
 
  # Used to print items to screen, while terminal is in funky mode
  def loginfo(self, str):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    print(str)
    tty.setraw(sys.stdin.fileno())
 
  # Used to print teleop status
  def show_status(self):
    msg = 'Command:\tLinearX %.2f\tAngleZ %.2f\tLinearZ %.2f\tLinearY %.2f' % (self.command[0],self.command[1],self.command[2],self.command[3])
    self.loginfo(msg)
    
  # For everything that can't be a binding, use if/elif instead
  def process_key(self, ch):
    if ch == 'h':
      self.print_usage()
    elif ch in self.cmd_bindings.keys():
      self.commandToDo = self.cmd_bindings[ch]
    elif ch == 'x':
      self.loginfo('Exiting')
      # Stop the robot
      twist = Twist()
      self.pub_twist.publish(twist)
      rospy.signal_shutdown('Shutdown')
    else:
      self.loginfo('Stop the robot')
      self.command = np.array([0, 0, 0, 0])
      self.commandToDo = np.array([0, 0, 0, 0])

 
  def update(self):
    if rospy.is_shutdown():
      return
    twist = Twist()
    self.command  = self.command+self.speed*self.commandToDo
    twist.linear.x = self.command[0]
    twist.angular.z = self.command[1]
    twist.linear.z = self.command[2]
    twist.linear.y = self.command[3]
    self.pub_twist.publish(twist)
    self.show_status()
 
  # Get input from the terminal
  def get_key(self):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    return key.lower()
 
if __name__ == '__main__':
        rospy.init_node('keyboard_teleop')
        teleop = KeyTeleop()
        teleop.run()
