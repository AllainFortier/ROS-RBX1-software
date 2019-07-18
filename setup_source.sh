#!/usr/bin/env bash

source '/home/ubuntu/RBX1/moveo_ws/devel/setup.bash'

# export ROS_HOSTNAME=$(hostname).local
export ROS_HOSTNAME="piarm.local"

export ROS_MASTER_URI=http://ardec-VirtualBox.local:11311

exec "$@"