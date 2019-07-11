#!/bin/bash

export ROS_IPV6=on
export ROS_MASTER_URI=http://master:11311

. /opt/ros/kinetic/setup.sh
. ~/ros_workspace/devel/setup.sh
roslaunch tutorial_pkg tutorial_robomaker_1_ec2.launch