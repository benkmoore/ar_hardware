#!/usr/bin/env bash

localRobotID = "$1"
masterIP="$2"

if [ -z "$1" ]; then echo Must specify a local robot ID. Note: check robot ID matches namespace in launch file.; exit 1; fi
if [ -z "$2" ]; then echo Must specify a master IP address.; exit 1; fi

localIP = $(hostname -I | awk '{print $1}')
 
export ROS_MASTER_URI=http://$masterIP:11311
export ROS_IP=localIP
export ROS_NAMESPACE=localRobotID
source ~/catkin_ws/devel/setup.bash

echo ROS master setup:
echo -----------------
echo ROS master IP URI: "$ROS_MASTER_URI"
echo Local IP address:  "$localIP"
echo Local robot name:  "$localRobotID"
