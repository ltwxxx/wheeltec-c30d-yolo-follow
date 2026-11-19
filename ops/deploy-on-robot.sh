#!/bin/bash
# Run on the robot as wheeltec with an explicit local release archive path.
# Never starts a follow session. Previous runtime/assets/units are saved for rollback.
set -euo pipefail
archive=$(realpath -- "${1:?release archive required}")
runtime="$HOME/.local/share/wheeltec-follow-edge"
assets=/usr/local/share/wheeltec-follow
container=codex-camera-live
release=$(date +%Y%m%d-%H%M%S)
backup="$HOME/.local/share/wheeltec-follow-backups/$release"
stage=$(mktemp -d /tmp/wheeltec-release.XXXXXX)
mkdir -p "$backup"
tar -xf "$archive" -C "$stage"
cd "$stage"
sha256sum -c SHA256SUMS
python3 -m compileall -q .
python3 -c 'from follow_settings import SETTINGS; print(SETTINGS)'
expected_version=$(python3 -c 'from follow_settings import VERSION; print(VERSION)')
bash -n ops/robot-follow-boot.sh
curl -fsS http://127.0.0.1:18769/status | python3 -c '
import json,sys
d=json.load(sys.stdin)["follow"]
assert not d["automatic"] and d["target"] is None and not d["unlocked"], "Follow is active; stop it before deploying"
assert d["forward_mps"]==0 and d["turn_rad_s"]==0
'
curl -fsS -H 'Content-Type: application/json' --data '{}' http://127.0.0.1:18768/stop >/dev/null
tar -C "$(dirname "$runtime")" -cf "$backup/runtime.tar" "$(basename "$runtime")"
sudo tar -C /usr/local/share -cf "$backup/assets.tar" wheeltec-follow
cp "$HOME/.config/systemd/user/wheeltec-follow-edge.service" "$backup/edge.service"
sudo cp /usr/local/libexec/wheeltec-follow-boot "$backup/boot.sh"
printf '%s\n' "$backup" > "$runtime/last-backup.txt"
bridges=(follow_sensors.py follow_actuator.py follow_settings.py follow_settings.json runtime_paths.py battery_state.py lidar_geometry.py)

stop_ros_container() {
    # Restart producers and consumers together so ROS can discover them afresh.
    # The boot service starts the same container after files have been copied.
    if [[ $(docker inspect -f '{{.State.Running}}' "$container") == true ]]; then
        docker stop --time 8 "$container" >/dev/null
    fi
}

rollback() {
    trap - ERR
    echo "Deployment failed; restoring $backup" >&2
    systemctl --user stop wheeltec-follow-edge.service || true
    sudo systemctl stop wheeltec-follow-boot.service || true
    stop_ros_container
    tar -xf "$backup/runtime.tar" -C "$(dirname "$runtime")"
    sudo tar -xf "$backup/assets.tar" -C /usr/local/share
    cp "$backup/edge.service" "$HOME/.config/systemd/user/wheeltec-follow-edge.service"
    sudo install -m 0755 "$backup/boot.sh" /usr/local/libexec/wheeltec-follow-boot
    for file in "${bridges[@]}"; do
        if [[ -f "$assets/$file" ]]; then docker cp "$assets/$file" "$container:/tmp/$file"; fi
    done
    systemctl --user daemon-reload
    sudo systemctl start wheeltec-follow-boot.service
    systemctl --user start wheeltec-follow-edge.service
    exit 1
}
trap rollback ERR
systemctl --user stop wheeltec-follow-edge.service
sudo systemctl stop wheeltec-follow-boot.service
stop_ros_container
for file in "${bridges[@]}"; do
    sudo install -m 0644 "$file" "$assets/$file"
    docker cp "$file" "$container:/tmp/$file"
done
sudo install -m 0644 ops/boot_health.py "$assets/boot_health.py"
sudo install -m 0755 ops/robot-follow-boot.sh /usr/local/libexec/wheeltec-follow-boot
cp ./*.py ./*.json "$runtime/"
mkdir -p "$runtime/templates" "$runtime/models" "$runtime/ops"
cp templates/follow.html "$runtime/templates/"
cp models/osnet_x0_25.onnx models/osnet_x0_25.json models/OSNET-LICENSE "$runtime/models/"
cp ops/record_validation.py "$runtime/ops/"
cp SHA256SUMS "$runtime/release-$release.sha256"
cp ops/wheeltec-follow-edge.service "$HOME/.config/systemd/user/wheeltec-follow-edge.service"
systemctl --user daemon-reload
sudo systemctl start wheeltec-follow-boot.service
ready=false
for attempt in {1..45}; do
    if curl -fsS --max-time 1 http://127.0.0.1:18768/health >/dev/null; then ready=true; break; fi
    sleep 1
done
$ready
systemctl --user start wheeltec-follow-edge.service
ready=false
for attempt in {1..30}; do
    if curl -fsS --max-time 1 http://127.0.0.1:18769/status | python3 -c '
import json,sys
d=json.load(sys.stdin)
assert d.get("version")==sys.argv[1] and d["follow"]["state"]=="idle"
assert d["control_ready"] and d["video"] and d["sensors"]["lidar"]
' "$expected_version" 2>/dev/null; then ready=true; break; fi
    sleep 1
done
$ready
trap - ERR
printf 'DEPLOYED %s BACKUP %s\n' "$release" "$backup"
