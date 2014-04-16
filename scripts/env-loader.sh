#!/bin/bash

. /opt/ros/fuerte/setup.sh
# ROS path stuff
source ~/ros/starmac-ros-pkg/scripts/set-env-vars.sh
viper_pelican $(cat ~/ros/starmac-ros-pkg/scripts/ros_master_host_name)
exec "$@"
