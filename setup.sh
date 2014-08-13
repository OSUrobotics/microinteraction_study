#!/usr/bin/env bash

source /home/lazewatd/study_ws_catkin/devel/setup.bash
export ROS_MASTER_URI=http://lsc.local:11311
export ROS_HOSTNAME=dvalinn.local
export DISPLAY=:0
exec "$@" 
