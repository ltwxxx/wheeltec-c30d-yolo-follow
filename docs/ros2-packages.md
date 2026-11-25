# ROS 2 包化运行

当前版本把传感器读取、`cmd_vel` 执行、指令看门狗和电压保护放进 `follow_bridges` ROS 2 包。Flask 页面、人物检测/跟踪/Re-ID、会话和运动规划仍保留现有 HTTP 接口，避免一次改变全部控制行为。

## 包结构

```text
ros2_ws/src/
├── follow_bridges/         传感器与执行器节点、安全逻辑、运行配置
├── follow_bringup/         组合 launch 与 ROS 参数
└── follow_sim/             差速运动学 SIL 仿真
```

`src/follow_actuator.py`、`src/follow_sensors.py` 仅保留兼容入口。现有发布包通过 `src/follow_bridges/` 携带同一份节点实现。

## 构建

在 Linux ROS 2 Humble 环境执行：

```console
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 实车启动

在车辆 ROS 域内执行：

```console
ros2 launch follow_bringup follow_bringup.launch.py
```

默认启动只读传感器桥（`127.0.0.1:18767`）和带独立安全层的执行器桥（`127.0.0.1:18768`），话题和端口集中在 `follow_bringup/config/follow_bridge_params.yaml`。

```console
# 只启动传感器，不启动执行器
ros2 launch follow_bringup follow_bringup.launch.py start_actuator:=false

# 更换参数文件或命名空间
ros2 launch follow_bringup follow_bringup.launch.py \
  params_file:=/absolute/path/to/params.yaml namespace:=/robot1
```

继续按原流程启动 `camera_viewer.py`。人工停止、0.28 秒指令看门狗、停止后的零速度突发以及低电压锁存行为保持不变。

## SIL 启动

SIL 使用独立 ROS 域，默认 `ROS_DOMAIN_ID=42`，不能与真车域桥接：

```console
ops/run-ros2-sil.sh
```

脚本加载已构建 workspace，并启动一个 `follow_sim` 与一个 `actuator_bridge`，不启动相机或物理底盘驱动。

另开终端，加载相同环境并设置相同域：

```console
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 node list
ros2 topic hz /odom
curl http://127.0.0.1:18768/health
```

等待 `/health` 返回 `ok: true` 后，可以 arm 并发送低速命令：

```console
curl -X POST http://127.0.0.1:18768/arm
curl -X POST http://127.0.0.1:18768/command \
  -H 'Content-Type: application/json' \
  -d '{"active":true,"forward_mps":0.05,"turn_rad_s":0.0}'
```

命令不刷新时，执行器在 0.28 秒后发布零速度；仿真自身也在 0.5 秒无新命令后停止积分。人工停止：

```console
curl -X POST http://127.0.0.1:18768/stop
```

## 参数与资源

- 话题名、HTTP 端口：ROS 参数，见 `follow_bringup/config/follow_bridge_params.yaml`。
- 跟随阈值、电池阈值和车辆几何：`follow_bridges/config/follow_settings.json`，安装到包的 `resources/config/`；仍通过 `WHEELTEC_FOLLOW_CONFIG` 覆盖，修改后需重启进程。
- 当前包化阶段不把跟随 JSON 全部暴露为可动态修改的 ROS 参数，避免绕过现有范围校验。
- 包资源可用 `WHEELTEC_FOLLOW_RESOURCE_ROOT` 覆盖；普通源码开发不需要设置。

## 验证边界

硬件无关测试覆盖执行器看门狗、同步停止、低电压锁存、陈旧数据拒绝、发布归档与独立运行资源。ROS launch、DDS 发现和实车行为必须在 ROS 2 Humble Linux 环境验证。运动学仿真只验证接口和指定停车语义，不替代动力学、制动距离或现场安全验收。
