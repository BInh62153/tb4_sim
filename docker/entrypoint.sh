#!/bin/bash
set -e
# Source ROS2
source /opt/ros/humble/setup.bash
# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi
# Export env vars
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export TURTLEBOT4_MODEL=${TURTLEBOT4_MODEL:-standard}
export IGN_GAZEBO_RESOURCE_PATH=/opt/ros/humble/share:${IGN_GAZEBO_RESOURCE_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Start virtual display nếu chưa có DISPLAY thật
# Cần cho OGRE render camera sensors (OAK-D) khi chạy headless
if [ "${START_XVFB:-true}" = "true" ]; then
    # Xóa file lock cũ nếu có
    rm -f /tmp/.X${DISPLAY#:}*
    
    Xvfb "${DISPLAY:-:99}" -screen 0 1280x1024x24 -ac +extension GLX +render -noreset &
    export DISPLAY="${DISPLAY:-:99}"
    sleep 2
fi
# Print status
echo "╔══════════════════════════════════════════════╗"
echo "║      TurtleBot4 Simulation Container         ║"
echo "╠══════════════════════════════════════════════╣"
echo "║  ROS_DISTRO   : $ROS_DISTRO"
echo "║  DOMAIN_ID    : $ROS_DOMAIN_ID"
echo "║  TB4_MODEL    : $TURTLEBOT4_MODEL"
echo "║  RMW          : $RMW_IMPLEMENTATION"
echo "╚══════════════════════════════════════════════╝"
exec "$@"