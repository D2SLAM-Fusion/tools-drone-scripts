#!/bin/bash
source /opt/ros/noetic/setup.bash
echo dji |sudo -S chmod +777 /dev/ttyTHS0
roslaunch mavros px4.launch fcu_url:=serial:///dev/ttyTHS0:921600 gcs_url:=udp-b://@