#!/bin/bash
set -u

svc="$HOME/.local/share/wheeltec-follow"
socket="$svc/ssh-robot-ui"
identity="$svc/robot-tunnel-ed25519"
robot_ip=192.168.1.242
page=http://127.0.0.1:8769
announced=false
failures=0

[[ -r "$identity" ]] || exit 1
exec 9>"$svc/watch.lock"
flock -n 9 || exit 1

connect() {
    ssh -n -fNT -M -S "$socket" -o BatchMode=yes -o ConnectTimeout=4 \
        -o ExitOnForwardFailure=yes -o StrictHostKeyChecking=yes \
        -o "HostKeyAlias=$robot_ip" -o UserKnownHostsFile=/mnt/c/Users/lenovo/.ssh/known_hosts \
        -o IdentitiesOnly=yes -i "$identity" -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
        -o IPQoS=none \
        -L 127.0.0.1:8769:127.0.0.1:18769 "wheeltec@$1" 9>&- >/dev/null 2>&1
}

while true; do
    if ! ssh -S "$socket" -O check "wheeltec@$robot_ip" >/dev/null 2>&1; then
        if [[ -S "$socket" ]]; then rm -f -- "$socket"; fi
        failures=0
        if ! connect "$robot_ip"; then
            resolved=$(timeout 5 /mnt/c/Windows/System32/PING.EXE -4 -n 1 -w 400 wheeltec.local 2>/dev/null |
                sed -nE 's/.*\[([0-9]+(\.[0-9]+){3})\].*/\1/p' | head -1)
            if [[ -z "$resolved" || "$resolved" == "$robot_ip" ]] || ! connect "$resolved"; then
                sleep 4
                continue
            fi
        fi
    fi

    if curl --noproxy '*' -fsS --max-time 2 "$page/healthz" 2>/dev/null |
            python3 -c 'import json,sys; d=json.load(sys.stdin); sys.exit(not (d["backend"]=="robot" and not d["view_only"]))' 2>/dev/null; then
        failures=0
        if ! $announced; then
            printf 'WHEELTEC_READY\n'
            announced=true
        fi
    else
        ((failures += 1))
        if ((failures == 3)); then
            printf 'Reconnecting robot console forward\n' >&2
            ssh -S "$socket" -O exit "wheeltec@$robot_ip" >/dev/null 2>&1 || true
        fi
    fi
    sleep 4
done
