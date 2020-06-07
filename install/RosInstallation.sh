#!/bin/bash
# Follows Instructions at: http://wiki.ros.org/melodic/Installation/Ubuntu

if [ -z "$1" ]; then echo Must specify a target. Options are ./test.sh [computer,robot]; exit 1; fi
if [ $1 != "computer" ] && [ $1 != "robot" ]; then echo Options are ./test.sh [computer,robot]; exit 1; fi

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

if [ $1 == "computer" ]; then sudo apt install ros-melodic-desktop-full; fi
if [ $1 == "robot" ]; then sudo apt install ros-melodic-desktop; fi

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
