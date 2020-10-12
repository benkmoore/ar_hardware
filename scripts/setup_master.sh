#!/usr/bin/env bash

# Setups slave robot to attach to master.
# Follows steps at: http://www.iri.upc.edu/files/scidoc/1607-Multi-master-ROS-systems.pdf

localName ="$1"
masterIP="$2"
masterName="$3"

if [ -z "$1" ]; then echo Must specify a local robot ID. Note: check robot ID matches namespace in launch file.; exit 1; fi
if [ -z "$2" ]; then echo Must specify a master IP address.; exit 1; fi
if [ -z "$3" ]; then echo Must specify a master name.; exit 1; fi

localIP=$(hostname -I | awk '{print $1}')

# export ROS config if not already set
grep -qF -- "ROS_MASTER_URI=http://$masterIP:11311" "~/.bashrc" || export ROS_MASTER_URI=http://$masterIP:11311
grep -qF -- "ROS_IP=$localIP" "~/.bashrc" || export ROS_IP=localIP
grep -qF -- "ROS_NAMESPACE=$localName" "~/.bashrc" || export ROS_NAMESPACE=localname
source ~/.bashrc
source ~/catkin_ws/devel/setup.bash

echo ROS master setup:
echo -----------------
echo ROS master IP URI: "$ROS_MASTER_URI"
echo Local IP address:  "$localIP"
echo Local namespace:  "$localName"

# setup hosts on network if not already set
host_file='/etc/hosts'
grep -qF -- "$localIP  $localName" "$host_file" || echo "$localIP  $localName" >> "$host_file"
grep -qF -- "$masterIP $masterName" "$host_file" || echo "$masterIP $masterName" >> "$host_file"

# enable multicast networking if not already set
multicast = "net.ipv4.icmp_echo_ignore_broadcasts=0"
grep -qF -- "$multicast" "/etc/sysctl.conf" || echo "$multicast" >> "/etc/sysctl.conf"
sudo service procps restart
