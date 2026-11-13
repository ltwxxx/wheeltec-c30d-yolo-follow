#!/bin/bash
set -u

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 1
svc="$HOME/.local/share/wheeltec-follow"
if [[ -e "$svc/use-robot-ui" ]]; then
    exec bash ./ops/watch-robot-ui.sh
fi
socket="$svc/ssh-control"
sensor_socket="$svc/ssh-sensor"
identity="$svc/robot-tunnel-ed25519"
unit=wheeltec-follow-viewer
page=http://127.0.0.1:8769
robot_ip=192.168.1.242
announced=false
enable_detection=false
control_failures=0
sensor_failures=0

[[ -r "$identity" && -x "$svc/venv/bin/python" && -r wheeltec-yolo11n.pt ]] || exit 1
exec 9>"$svc/watch.lock"
flock -n 9 || exit 1

connect() {
    local address=$1 path=$2 port=$3
    ssh -n -fNT -M -S "$path" -o BatchMode=yes -o ConnectTimeout=3 \
        -o ExitOnForwardFailure=yes -o StrictHostKeyChecking=yes \
        -o "HostKeyAlias=$robot_ip" -o UserKnownHostsFile=/mnt/c/Users/lenovo/.ssh/known_hosts \
        -o IdentitiesOnly=yes -i "$identity" -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
        -C -L "127.0.0.1:$port:127.0.0.1:$port" "wheeltec@$address" 9>&- >/dev/null 2>&1
}

master_running() {
    ssh -S "$1" -O check "wheeltec@$robot_ip" >/dev/null 2>&1
}

ensure_tunnel() {
    local path=$1 port=$2 resolved
    if master_running "$path"; then return 0; fi
    if [[ -S "$path" ]]; then rm -f -- "$path"; fi
    if connect "$robot_ip" "$path" "$port"; then return 0; fi
    # Windows can resolve the robot's mDNS name even in WSL NAT mode.
    resolved=$(timeout 5 /mnt/c/Windows/System32/PING.EXE -4 -n 1 -w 400 wheeltec.local 2>/dev/null |
        sed -nE 's/.*\[([0-9]+(\.[0-9]+){3})\].*/\1/p' | head -1)
    [[ -n "$resolved" && "$resolved" != "$robot_ip" ]] && connect "$resolved" "$path" "$port"
}

check_forward() {
    local path=$1 url=$2
    local -n failures=$3
    # A ROS readiness error still proves that HTTP reached the car.
    if curl --noproxy '*' -sS --max-time 2 -o /dev/null "$url" 2>/dev/null; then
        failures=0
    else
        ((failures += 1))
        if ((failures == 3)); then
            printf 'Reconnecting unresponsive robot forward: %s\n' "$url" >&2
            ssh -S "$path" -O exit "wheeltec@$robot_ip" >/dev/null 2>&1 || true
        fi
    fi
}

start_viewer() {
    systemd-run --user --unit="$unit" --collect --property=Type=exec --working-directory="$PWD" \
        --setenv="WHEELTEC_YOLO_WEIGHTS=$PWD/wheeltec-yolo11n.pt" \
        --setenv="YOLO_CONFIG_DIR=$svc/ultralytics" --setenv="MPLCONFIGDIR=$svc/matplotlib" \
        --setenv=MPLBACKEND=Agg --setenv=YOLO_AUTOINSTALL=false --setenv=OMP_NUM_THREADS=2 \
        --setenv=OPENBLAS_NUM_THREADS=1 --setenv=MKL_NUM_THREADS=1 \
        "$svc/venv/bin/python" -B -u "$PWD/camera_viewer.py" >&2
}

while true; do
    if ! master_running "$socket"; then
        if ((control_failures < 3)); then announced=false; fi
        control_failures=0
    fi
    if ! master_running "$sensor_socket"; then sensor_failures=0; fi
    if ! ensure_tunnel "$socket" 18768 || ! ensure_tunnel "$sensor_socket" 18767; then
        sleep 4
        continue
    fi
    check_forward "$socket" http://127.0.0.1:18768/health control_failures
    check_forward "$sensor_socket" http://127.0.0.1:18767/scan sensor_failures

    if ! systemctl --user is-active --quiet "$unit"; then
        if start_viewer; then enable_detection=true; fi
    fi
    if $enable_detection && curl --noproxy '*' -fsS --max-time 2 -H 'Content-Type: application/json' \
        --data '{"enabled":true}' "$page/detection" >/dev/null 2>&1; then
        enable_detection=false
    fi
    if ! $announced && curl --noproxy '*' -fsS --max-time 2 "$page/status" 2>/dev/null |
        python3 -c 'import json,sys; d=json.load(sys.stdin); sys.exit(not (d["video"] and d["sensors"]["aligned"] and d["control_ready"]))' 2>/dev/null; then
        printf 'WHEELTEC_READY\n'
        announced=true
    fi
    sleep 4
done
