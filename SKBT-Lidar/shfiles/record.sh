#! /bin/bash

roslaunch livox_ros_driver2 msg_MID360.launch & sleep 3
roslaunch faster_lio mapping_avia.launch & sleep 3; 

