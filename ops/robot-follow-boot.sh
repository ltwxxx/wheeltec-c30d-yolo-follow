#!/bin/bash
set -u

container=codex-camera-live
assets=/usr/local/share/wheeltec-follow

start_missing() {
    local pattern=$1 log=$2
    shift 2
    if ! docker exec "$container" pgrep -f -- "$pattern" >/dev/null; then
        docker exec -d "$container" bash -c '
            log=$1
            shift
            exec >>"$log" 2>&1
            source /opt/ros/humble/setup.bash &&
            source /home/wheeltec/wheeltec_ros2/install/setup.bash &&
            exec "$@"
        ' bash "$log" "$@"
    fi
}

while true; do
    if [[ $(docker inspect -f '{{.State.Running}}' "$container") != true ]]; then
        if ! docker start "$container"; then
            sleep 8
            continue
        fi
    fi

    for script in follow_sensors.py follow_actuator.py follow_settings.py follow_settings.json runtime_paths.py battery_state.py lidar_geometry.py; do
        if ! docker exec "$container" test -s "/tmp/$script"; then
            docker cp "$assets/$script" "$container:/tmp/$script"
        fi
    done

    if docker exec "$container" test -c /dev/wheeltec_controller; then
        start_missing '^(.*python3 )?.*/ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py$' \
            /tmp/wheeltec-boot-base.log /opt/ros/humble/bin/ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
    fi
    if docker exec "$container" test -c /dev/wheeltec_lidar; then
        start_missing '^(.*python3 )?.*/ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py$' \
            /tmp/wheeltec-boot-lidar.log /opt/ros/humble/bin/ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py
    fi
    start_missing '^(.*python3 )?.*/ros2 launch astra_camera astra.launch.xml ' \
        /tmp/wheeltec-boot-camera.log /opt/ros/humble/bin/ros2 launch astra_camera astra.launch.xml \
        depth_registration:=true color_depth_synchronization:=true enable_d2c_viewer:=false \
        enable_point_cloud:=false enable_ir:=false \
        color_width:=320 color_height:=240 depth_width:=320 depth_height:=240
    start_missing '^python3 -u /tmp/follow_sensors.py$' \
        /tmp/wheeltec-boot-sensors.log python3 -u /tmp/follow_sensors.py
    start_missing '^python3 -u /tmp/follow_actuator.py$' \
        /tmp/wheeltec-boot-actuator.log python3 -u /tmp/follow_actuator.py
    timeout 25 python3 "$assets/boot_health.py" || true
    sleep 8
done
