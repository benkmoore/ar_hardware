#!/bin/bash

if [ -z "$1" ]; then echo Must specify a target. Options are ./test.sh [computer,robot]; exit 1; fi
if [ $1 != "computer" ] && [ $1 != "robot" ]; then echo Options are ./test.sh [computer,robot]; exit 1; fi

# setup catkin
. ~/.bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# setup git config
git config --global credential.helper store

# clone repository
cd ~/catkin_ws/src/
git clone https://github.com/zprihoda/ar_commander.git
if [ $1 == "computer" ]; then git clone https://github.com/zprihoda/ar_sim.git; fi

# remake ws
cd ~/catkin_ws/
catkin_make

# add workspace setup to bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# TODO: Python dependencies
