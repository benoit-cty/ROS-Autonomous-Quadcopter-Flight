#!/bin/bash
#!/bin/bash
roslaunch hector_gazebo_worlds small_indoor_scenario.launch &
sleep 10
roslaunch hector_quadrotor_gazebo mySpawn_quadrotor_with_kinect.launch 



