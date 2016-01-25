This project aim to create an autonomous flying quadcopter using a stereo camera like Kinect/Guidance/ZED.
For now you have to move it manually to discover the environment. After that it could find is way by itself.

Video demonstration : https://www.youtube.com/watch?v=VlBQLbmc03g
You can find additional info on this work in AlessioT thesis, however it's in italian:
https://mega.co.nz/#!bdYEWKDZ!WmsdbkD-DifIXAGAL4cHTBwb_hYw36mMpB_XMmLX5VA

Original project from AlessioT (https://bitbucket.org/AlessioT/autonomous-flight-ros.git)
Modified for ROS Indigo support by WLemkens (https://bitbucket.org/wlemkens/)
My (Trancept https://bitbucket.org/Trancept/) contribution is quite limited to :
- Testing on Ubuntu 14.04 under ROS Indigo and updating this readme.
- Enlarge the sensor range to 20 meter in hector_quadrotor/hector_quadrotor_urdf/urdf/quadrotore.urdf
- Removing the drift to obtain a more stable quad in hector_quadrotor/hector_quadrotor_urdf/urdf/quadrotore.urdf
- Writing a script to pilot the quadcopter in all axis with keyboard

In order to run the simulation you will need: 
	ROS, Hector_quadrotor + hector_quadrotor_world (http://wiki.ros.org/hector_quadrotor), gazebo (http://wiki.ros.org/gazebo), Moveit! (http://moveit.ros.org/)

If you are using ubuntu you could follow these instruction :
- Install ROS Indigo ros-indigo-desktop-full like explained in http://wiki.ros.org/indigo/Installation/Ubuntu
- Install complementary package :
$ sudo apt-get install ros-indigo-hector-quadrotor-demo ros-indigo-pr2-gazebo-plugins ros-indigo-moveit-full
- Make a workspace folder
$ mkdir my_workspace
- Get the sourcecode
$ cd my_workspace
$ git clone https://bitbucket.org/Trancept/autonomous-quadcopter-flight-ros.git
$ mv autonomous-quadcopter-flight-ros src
- Init workspace
$ cd src
$ catkin_init_workspace
- Compile the source
$ cd ..
$ catkin_make

- Initiate the environment variable (to do everytime)
$ source devel/setup.bash
- Run the programm
$ roslaunch quadrotore2_moveit start.launch
- Wait for RVIZ to come up
- To do manually pilot, open another terminal and run
$ python ./src/script/tx_keyboard_ctrl.py
- Then in RVIZ you could assign a Goal to the quadcopter :
  - In "Motion Planning" panel, choose "RRTConnectkConfigDefault" as OMPL solver
  - In "Motion Planning" Panel, panel "Planning" grow up the workspace to 20, 20, 10
  - Move the goal to where you whant
  - Click on Plan
  - Click on Execute
- If you whant to launch only part of simulation, you can run sh script (re-run source devel/setup.bash in every terminal) :
$ sh src/script/1-simulator.sh
$ sh src/script/2-controller.sh
$ sh src/script/3-planner.sh


Contents of the folders:

->action_controller:
	 action controller to translate geometrical trajectory in cmd_vel to control the quadrotor, it automatically translate the geometrical 		trajectory produced by moveit in comand to make the quadrotor move.

->moveit_simple_controller_manager: 
	modified version of moveit_simple_controller_manager to handle multiDofFollowJointTrajectoryAction and to send the trajectory to the custom 		action_controller above defined

->hector_quadrotor:
	modified urdf of the quadrotor to publish the odometry of the joints

->pr2_teleop:
	simple node to teleop the simulated quadrotor using the keyboard => Prefer tx_keyboard_ctrl.py as it allow vertical move.

->quadrotore2_moveit:
	configuration file for moveit!

->script:
	file to launch the different part of the simulation
