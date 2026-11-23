# ROS 2 SIL Closed-Loop Design

## Goal

Provide a repeatable Software-in-the-Loop (SIL) workflow that exercises the repository's existing actuator bridge against its kinematic simulator. A developer should be able to start both processes in an isolated ROS 2 domain, send a bounded command through the same loopback HTTP interface used by the application, observe simulated motion, and see the watchdog stop the robot.

## Current Components

- `ros2_ws/src/follow_sim/follow_sim/kinematics_node.py` subscribes to `/cmd_vel` and publishes `/odom`, `/scan`, `/PowerVoltage`, and `/tf`.
- `ros2_ws/src/follow_bridges/follow_bridges/actuator_bridge.py` loads the existing `src/follow_actuator.py` without changing its control behavior. The actuator publishes `/cmd_vel`, consumes `/odom` and `/PowerVoltage`, and exposes HTTP on `127.0.0.1:18768`.
- The actuator rejects commands unless its simulated battery feedback is fresh and a `/cmd_vel` subscriber is present. It stops active commands after 0.28 seconds without refresh; the simulator independently zeros stale commands after 0.5 seconds.

## Proposed Changes

1. Add `follow_sim/launch/sil.launch.py` to launch the existing simulator and actuator bridge together. Install the launch file through `follow_sim/setup.py` and declare the ROS launch runtime dependencies in `package.xml`.
2. Add `ops/run-ros2-sil.sh` as the supported entry point. It sources ROS and the built workspace, sets `WHEELTEC_FOLLOW_SRC` to this checkout's `src/`, and selects a dedicated nonzero `ROS_DOMAIN_ID` (default 42, configurable through `SIL_ROS_DOMAIN_ID`). It must reject domain 0 and invalid domain values.
3. Add `docs/ros2-sil.md` with build/start/stop commands, topic and HTTP health checks, a low-speed command walkthrough, watchdog observation steps, VS Code debugging pointers, and a troubleshooting table.

No simulator physics, perception, actuator limits, or application control logic changes are in scope. This SIL slice does not launch camera, YOLO, or physical base drivers. The existing `/PowerVoltage` publisher check remains a secondary startup interlock; domain separation is the primary boundary and the SIL domain must not be bridged to a vehicle domain.

## Data Flow

```text
HTTP POST /command (127.0.0.1:18768)
  -> existing actuator bridge
  -> ROS /cmd_vel
  -> kinematic simulator
  -> ROS /odom and /PowerVoltage
  -> actuator health state (HTTP GET /health)
```

The operator can inspect `/cmd_vel`, `/odom`, `/scan`, and `/PowerVoltage` from a shell that uses the same `ROS_DOMAIN_ID` as the launcher. A single short command is expected to time out automatically; sustained motion requires refreshing commands and is not part of this initial manual walkthrough.

## Acceptance Criteria

- A clean ROS 2 Humble workspace build installs the launch entry and the launcher starts exactly one simulator and one actuator bridge in the selected domain.
- After simulated voltage and odometry arrive, `GET /health` reports ready; arming and a bounded low-speed command produce a nonzero `/cmd_vel` and changing `/odom`.
- When the command is not refreshed, the actuator publishes zero velocity after its 0.28-second timeout, and the simulator also has its independent 0.5-second stale-command stop.
- Startup, Ctrl-C shutdown, and troubleshooting steps are documented. No real vehicle, camera, or networked hardware is needed for the SIL walkthrough.

## Operational Constraints

- Target runtime is Linux with ROS 2 Humble; the current macOS Python environment does not provide `rclpy`.
- Domain isolation prevents ordinary DDS discovery across domains but is not a security boundary if an explicit bridge is configured. Keep the SIL domain separate from the vehicle domain and do not run physical base drivers in this launch.
- The simulator is a kinematic contract double, not a dynamics-accurate or safety-certified model. Results validate interface wiring and selected stop behavior only.
