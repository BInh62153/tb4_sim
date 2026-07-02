#!/bin/bash
set -e

#region agent log
_dbg() {
    local msg="$1" data="$2" hid="${3:-H0}"
    mkdir -p /ros2_ws/.cursor 2>/dev/null || true
    printf '{"sessionId":"a3be86","timestamp":%s,"location":"entrypoint.sh","message":"%s","data":%s,"hypothesisId":"%s"}\n' \
        "$(date +%s%3N)" "$msg" "$data" "$hid" >> /ros2_ws/.cursor/debug-a3be86.log 2>/dev/null || true
}
#endregion

# Source ROS2
source /opt/ros/humble/setup.bash
# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi
# Export env vars
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export TURTLEBOT4_MODEL=${TURTLEBOT4_MODEL:-standard}
export IGN_GAZEBO_RESOURCE_PATH=/ros2_ws/worlds:/opt/ros/humble/share:${IGN_GAZEBO_RESOURCE_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# VirtualGL: EGL backend works in Docker with --gpus all (no host X required).
# Override with VGL_DISPLAY=:0 if host runs a headless NVIDIA X server.
export USE_VIRTUALGL="${USE_VIRTUALGL:-false}"
export VGL_DISPLAY="${VGL_DISPLAY:-egl0}"

# Start virtual display nếu chưa có DISPLAY thật
# Cần cho OGRE render camera sensors (OAK-D) khi chạy headless
if [ "${START_XVFB:-true}" = "true" ]; then
    XVFB_DISPLAY="${DISPLAY:-:99}"
    XVFB_RES="${XVFB_RESOLUTION:-1280x720x24}"

    # Xóa file lock cũ nếu có
    rm -f /tmp/.X${XVFB_DISPLAY#:}*

    # +extension GLX bắt buộc cho VirtualGL; giữ độ phân giải ≤1280x720 tiết kiệm VRAM
    Xvfb "${XVFB_DISPLAY}" -screen 0 "${XVFB_RES}" -ac +extension GLX +render -noreset &
    export DISPLAY="${XVFB_DISPLAY}"
    sleep 2

    #region agent log
    _dbg "xvfb started" "{\"display\":\"${DISPLAY}\",\"resolution\":\"${XVFB_RES}\",\"use_virtualgl\":\"${USE_VIRTUALGL}\",\"vgl_display\":\"${VGL_DISPLAY}\"}" "H4"
    #endregion
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