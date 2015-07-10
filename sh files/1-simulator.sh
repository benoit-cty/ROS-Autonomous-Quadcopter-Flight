#!/bin/bash
roslaunch hector_gazebo_worlds wg_collada_realtime.launch &
sleep 10
roslaunch hector_quadrotor_gazebo mySpawn_quadrotor_with_kinect.launch 



