export STARMAC_ROS_PKG_ROOT=~/ros/starmac-ros-pkg

export ROS_PACKAGE_PATH=${STARMAC_ROS_PKG_ROOT}:${ROS_PACKAGE_PATH}
export ROS_PACKAGE_PATH=~/ros/asctec_mav_framework:${ROS_PACKAGE_PATH}
export ROS_PACKAGE_PATH=~/ros/ethz-asl/ros-drivers/vicon_bridge:${ROS_PACKAGE_PATH}

#export ROS_PACKAGE_PATH=~/ros/fuerte_src/rosh_core:${ROS_PACKAGE_PATH}
#export ROS_PACKAGE_PATH=~/ros/gps_umd:${ROS_PACKAGE_PATH}


export ROS_WORKSPACE=~/ros
export BUILDER_32_BIT_HOSTNAME=lynx
export VICON_BRIDGE_HOSTNAME=wildcat

source ~/ros/starmac-ros-pkg/scripts/viper_pelican.bsh
