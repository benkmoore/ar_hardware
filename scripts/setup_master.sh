#!/usr/bin/env bash

masterIP="$1"
if [ -z "$1" ]; then echo Must specify a master IP address; exit 1; fi

localIP = $(hostname -I | awk '{print $1}')
 
export ROS_MASTER_URI=http://$masterIP:11311
export ROS_IP=localIP
source ~/catkin_ws/devel/setup.bash

echo ROS master setup:
echo -----------------
echo ROS master IP URI: "$ROS_MASTER_URI"
echo Local IP address:  "$localIP"
