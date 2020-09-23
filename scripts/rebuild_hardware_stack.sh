#!/bin/bash

# rebuild workspace
( cd ~/catkin_ws; catkin_make )

# rosserial: remove old ros msgs and rebuild new msgs
rm -rf ~/Arduino/libraries/ros_lib;
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries ~/catkin_ws/src/ar_commander/msg;

# reload teensy with new msgs
./load_arduino_file.sh teensy41 hardware_interface.ino;
