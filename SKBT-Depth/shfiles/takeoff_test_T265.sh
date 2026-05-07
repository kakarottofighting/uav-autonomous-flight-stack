#! /bin/bash
sudo chmod 777 /dev/ttyACM0 & sleep 1;   
# sudo chmod 777 /dev/ttyUSB0 & sleep 2;  #给激光测距模块串口权限
roslaunch mavros px4.launch & sleep 3; #启动mavros
roslaunch realsense2_camera demo_t265.launch & sleep 3;   #开启T265
#roslaunch usb_cam usb_cam-test.launch & sleep 3;
#roslaunch apriltag_ros continuous_detection.launch
wait;
