#!/usr/bin/env bash

rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py
gnome-terminal --tab -e '/bin/bash -c ". baxter.sh; roslaunch realsense_camera r200_nodelet_default.launch"'
gnome-terminal --tab -e '/bin/bash -c ". baxter.sh; roslaunch ar_track_alvar pr2_indiv_no_kinect.launch"'
gnome-terminal --tab -e '/bin/bash -c ". baxter.sh; roslaunch realsense_camera r200_nodelet_default.launch"'
gnome-terminal --tab -e '/bin/bash -c ". baxter.sh; ./ProductFiles/broadcast_transform.py"'
gnome-terminal --tab -e '/bin/bash -c ". baxter.sh; roslaunch baxter_moveit_config baxter_grippers.launch"'
gnome-terminal --tab -e '/bin/bash -c ". baxter.sh; ./ProductFiles/master.py"'
