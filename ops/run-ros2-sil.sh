#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="$ROOT/ros2_ws/install/setup.bash"
DOMAIN="${SIL_ROS_DOMAIN_ID:-42}"

if [[ ! "$DOMAIN" =~ ^[0-9]+$ ]] || (( DOMAIN < 1 || DOMAIN > 232 )); then
    echo "SIL_ROS_DOMAIN_ID must be an integer from 1 to 232" >&2
    exit 2
fi
if [[ ! -f "$ROS_SETUP" || ! -f "$WORKSPACE_SETUP" ]]; then
    echo "Build the ROS 2 workspace first: source $ROS_SETUP && cd $ROOT/ros2_ws && colcon build --symlink-install" >&2
    exit 2
fi

source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
export ROS_DOMAIN_ID="$DOMAIN"
export WHEELTEC_FOLLOW_RESOURCE_ROOT="$ROOT"
exec ros2 launch follow_sim sil.launch.py
