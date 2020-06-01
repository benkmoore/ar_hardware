#!/bin/bash

# setup catkin
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# setup git config
git config --global credential.helper store

# clone repository
cd ~/catkin_ws/src/
git clone https://github.com/zprihoda/ar_commander.git
# git clone https://github.com/zprihoda/ar_sim.git   # for desktops only

# remake ws
cd ~/catkin_ws/
catkin_make

# add workspace setup to bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# TODO: Python dependencies
