#!/bin/bash
# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash

# Source your workspace (ROS 2 uses install/ instead of devel/)
source /home/dtc/ws/install/setup.bash

# if [ "$DIFF_GPS" = true ]; then
if [ true ]; then
    echo "[DIFF-GPS] Launching diff_GPS"
    ros2 launch diff_GPS diff_GPS.launch.py
else
    echo "[DIFF-GPS] RUN set to false, not launching diff_GPS"
    exec "$@"  
fi


