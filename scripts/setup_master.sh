#!/usr/bin/env bash

# Setups slave robot to attach to master.
# Follows steps at: http://www.iri.upc.edu/files/scidoc/1607-Multi-master-ROS-systems.pdf
#
# example usage:
# sudo ./setup_master.sh robot1 192.168.1.19 bens_machine


localName="$1"
masterIP="$2"
masterName="$3"

if [ -z "$1" ]; then echo Must specify a local robot name. Note: check robot name must match namespace in launch file.; exit 1; fi
if [ -z "$2" ]; then echo Must specify a master IP address.; exit 1; fi
if [ -z "$3" ]; then echo Must specify a master name.; exit 1; fi

localIP=$(hostname -I | awk '{print $1}')

# export ROS config if not already set
sudo grep -qF -- "export ROS_MASTER_URI=http://$masterIP:11311" ~/.bashrc || echo "export ROS_MASTER_URI=http://$masterIP:11311" >> ~/.bashrc
sudo grep -qF -- "export ROS_IP=$localIP" ~/.bashrc || echo "export ROS_IP=$localIP" >> ~/.bashrc
sudo grep -qF -- "export ROS_NAMESPACE=$localName" ~/.bashrc || echo "export ROS_NAMESPACE=$localName" >> ~/.bashrc
source ~/.bashrc
source ~/catkin_ws/devel/setup.bash

echo ROS master setup:
echo -----------------
echo ROS master IP URI: "$ROS_MASTER_URI"
echo Local IP address:  "$localIP"
echo Local namespace:  "$localName"

# setup hosts on network if not already set
host_file="/etc/hosts"
sudo grep -qF -- "$localIP $localName" "$host_file"  || echo "$localIP $localName" >> "$host_file"
sudo grep -qF -- "$masterIP $masterName" "$host_file" || echo "$masterIP $masterName" >> "$host_file"

# enable multicast networking if not already set
multicast="net.ipv4.icmp_echo_ignore_broadcasts=0"
sudo grep -qF -- "$multicast" /etc/sysctl.conf || echo "$multicast" >> /etc/sysctl.conf
sudo service procps restart
