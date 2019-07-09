#!/usr/bin/env bash

# export ROS_HOSTNAME=$(hostname).local
# export ROS_IP=192.168.10.148
export ROS_MASTER_URI=http://ardec-VirtualBox.local:11311

source '/home/ubuntu/RBX1/moveo_ws/devel/setup.bash'
exec "$@"