#!/usr/bin/env bash

source /home/gloin/workspaces/dan_study_ws/devel/setup.bash  # note this is ABSOLUTE PATH
export ROS_MASTER_URI=http://localhost:11311
#export ROS_PACKAGE_PATH=$HOME/study_ws_catkin:$ROS_PACKAGE_PATH
export ROS_HOSTNAME=gloin
export DISPLAY=:0
exec "$@" 
