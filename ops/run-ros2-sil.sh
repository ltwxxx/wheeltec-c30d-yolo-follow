#!/usr/bin/env bash
set -eo pipefail

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
if [[ "${ROS_LOCALHOST_ONLY:-1}" != "0" ]]; then
    export ROS_LOCALHOST_ONLY=1
fi
export WHEELTEC_FOLLOW_RESOURCE_ROOT="$ROOT"
python3 -c 'import os, signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp("ros2", ["ros2", "launch", "follow_sim", "sil.launch.py"])' &
launch_pid=$!

shutdown() {
    trap - INT TERM
    kill -INT "$launch_pid" 2>/dev/null || true
    status=0
    wait "$launch_pid" || status=$?
    exit "$status"
}
trap shutdown INT TERM
wait "$launch_pid"
