#!/bin/bash
set -u

svc="$HOME/.local/share/wheeltec-follow"
socket="$svc/ssh-robot-ui"
identity="$svc/robot-tunnel-ed25519"
last_address="$svc/last-robot-ip"
robot_ip=192.168.1.242
page=http://127.0.0.1:8769
announced_boot=
failures=0
launch_file=
launch_token=${2-}

[[ -r "$identity" ]] || exit 1
if [[ -n "$launch_token" ]]; then
    launch_file=$(wslpath -u "$1") || exit 1
fi
exec 9>"$svc/watch.lock"
flock -w 30 9 || exit 1

connect() {
    ssh -n -fNT -M -S "$socket" -o BatchMode=yes -o ConnectTimeout=4 \
        -o ExitOnForwardFailure=yes -o StrictHostKeyChecking=yes \
        -o "HostKeyAlias=$robot_ip" -o UserKnownHostsFile=/mnt/c/Users/lenovo/.ssh/known_hosts \
        -o IdentitiesOnly=yes -i "$identity" -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
        -o IPQoS=none \
        -L 127.0.0.1:8769:127.0.0.1:18769 "wheeltec@$1" 9>&- >/dev/null 2>&1
}

candidate_addresses() {
    {
        if [[ -r "$last_address" ]]; then cat -- "$last_address"; fi
        # Resolve through Windows so both Wi-Fi and Ethernet addresses are available in WSL NAT mode.
        timeout 6 /mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe -NoProfile -NonInteractive -Command \
            "[System.Net.Dns]::GetHostAddresses('wheeltec.local') | Select-Object -ExpandProperty IPAddressToString" 2>/dev/null
        printf '%s\n' "$robot_ip"
    } | tr -d '\r' | awk '/^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$/ && !seen[$0]++ { print; fflush() }'
}

connect_robot() {
    local address
    if [[ -S "$socket" ]]; then rm -f -- "$socket"; fi
    while IFS= read -r address; do
        if connect "$address"; then
            printf '%s\n' "$address" > "$last_address"
            return 0
        fi
    done < <(candidate_addresses)
    return 1
}

ready_boot() {
    # An unavailable status means the tunnel may be stale. A valid but unready status means wait.
    curl --noproxy '*' -fsS --max-time 2 "$page/status" 2>/dev/null |
        python3 -c '
import json, sys

try:
    status = json.load(sys.stdin)
    sensors = status["sensors"]
    boot = status["start_status"][0]
    if status["backend"] != "robot" or status["view_only"] or not boot:
        sys.exit(2)
    ready = status["video"] and status["enabled"] and status["control_ready"]
    if not (ready and sensors["aligned"] and sensors["lidar"]):
        sys.exit(1)
except (ValueError, KeyError, IndexError, TypeError):
    sys.exit(2)
print(boot)
' 2>/dev/null
}

record_failure() {
    ((failures += 1))
    if ((failures >= 3)); then
        announced_boot=
        ssh -S "$socket" -O exit "wheeltec@$robot_ip" >/dev/null 2>&1 || true
    fi
}

while true; do
    # Task Scheduler can stop PowerShell without stopping its WSL process.
    # A new launcher changes the token before waiting for the old watcher to exit.
    if [[ -n "$launch_file" && $(tr -d '\r\n' < "$launch_file") != "$launch_token" ]]; then
        exit 0
    fi
    if ! ssh -S "$socket" -O check "wheeltec@$robot_ip" >/dev/null 2>&1 && ! connect_robot; then
        record_failure
    elif boot=$(ready_boot); then
        failures=0
        if [[ "$boot" != "$announced_boot" ]]; then
            printf 'WHEELTEC_READY\n'
            announced_boot=$boot
        fi
    elif [[ $? == 1 ]]; then
        failures=0
    else
        record_failure
    fi
    sleep 4
done
