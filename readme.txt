Original project from AlessioT (https://bitbucket.org/AlessioT/autonomous-flight-ros.git)
Modified for ROS Indigo support by XXX

My (XXX) contribution is quite limited to testing and updating this readme.
Tested on Ubuntu 14.04 under ROS Indigo



You can find additional info on this work in XXX thesis, however it's in italian:
https://mega.co.nz/#!bdYEWKDZ!WmsdbkD-DifIXAGAL4cHTBwb_hYw36mMpB_XMmLX5VA

In order to run the simulation you will need: 
	ROS, Hector_quadrotor + hector_quadrotor_world, gazebo, Moveit!
The external programs can be found here:
ROS               --->  http://wiki.ros.org/
Hector_quadrotor  --->  http://wiki.ros.org/hector_quadrotor
gazebo            --->  http://wiki.ros.org/gazebo
Moveit!           --->  http://moveit.ros.org/


If you are using ubuntu you could follow these instruction :
Installing ROS Indigo



Make a workspace folder
mkdir my_workspace
cd my_workspace
git clone https://bitbucket.org/Trancept/autonomous-quadcopter-flight-ros.git
mv autonomous-quadcopter-flight-ros src
cd src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
roslaunch quadrotore2_moveit start.launch

Open another terminal and run to manual piloting it
XXXX

For autonomous navigation you can run XXXX


Rigth-clic in RVIZ Displays panel border to show Motion Panel.
Choose RRTConnectkConfigDefault
Grow up the workspace
Plan
Execute




Contents of the folders:

->action_controller:
	 action controller to translate geometrical trajectory in cmd_vel to control the quadrotor, it automatically translate the geometrical 		traiectory produced by moveit in comand to make the quadrotor move.

->moveit_simple_controller_manager: 
	modified version of moveit_simple_controller_manager to handle multiDofFollowJointTrajectoryAction and to send the trajectory to the custom 		action_controller above defined

->hector_quadrotor:
	modified urdf of the quadrotor to publish the odometry of the joints

->pr2_teleop:
	simple node to teleop the simulated quadrotor using the keyboard XXXX supprimer, non ?

->quadrotore2_moveit:
	configuration file for moveit!

->sh files:
	bash file to launch the different part of the simulation
