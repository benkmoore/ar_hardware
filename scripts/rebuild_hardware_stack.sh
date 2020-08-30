#!/bin/bash

( cd ~/catkin_ws; catkin_make )

rm -rf ~/Arduino/libraries/ros_lib;
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries ~/catkin_ws/src/ar_commander/msg
