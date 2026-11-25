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

脚本加载已构建 workspace，并启动一个 `follow_sim` 与一个 `actuator_bridge`，不启动相机或物理底盘驱动。默认设置 `ROS_LOCALHOST_ONLY=1`，仅允许本机 DDS 通信；确需跨主机 SIL 时可显式设置 `ROS_LOCALHOST_ONLY=0`，但仍不得桥接真车域。

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

硬件无关测试覆盖执行器看门狗、同步停止、低电压锁存、陈旧数据拒绝、发布归档、独立运行资源，以及 TF 直接/反向/多跳变换。运动学仿真只验证接口和指定停车语义，不替代动力学、制动距离或现场安全验收。

## 2026-11-26 安装与 SIL 验收

验收环境为 Docker Desktop 中的 Linux ARM64 ROS 2 Humble，镜像 `ros2-learn:follow-acceptance`（基于本机 `ros2-learn:dev`），Python 3.10、OpenCV 4.5.4、NumPy 1.21.5。仓库挂载到 `/work`，主验收使用 `ROS_DOMAIN_ID=42`，独立健康状态迁移探针使用 `ROS_DOMAIN_ID=43`，均设置 `ROS_LOCALHOST_ONLY=1`。本次没有连接真车，也没有修改实车 systemd 单元或人工停止路径。

在容器内完成以下检查：

```console
source /opt/ros/humble/setup.bash
cd /work/ros2_ws
colcon build --symlink-install
```

| 检查项 | 结果 |
| --- | --- |
| ROS 2 导入与工作区构建 | Humble、`rclpy`、`tf2_msgs`、OpenCV 可用；`follow_bridges`、`follow_bringup`、`follow_sim` 三包构建成功 |
| 安装资源与 launch | 两个桥节点入口、launch 参数及配置资源可通过安装空间加载 |
| 节点与话题发现 | 发现 `/follow_sim`、`/codex_follow_actuator`，以及 `/cmd_vel`、`/odom`、`/scan`、`/PowerVoltage`、`/tf` |
| 执行器健康迁移 | 仿真启动前 `/health` 返回 `503`；里程计和电压到达后返回 `200`，实测电池电压 `12.4 V` |
| 人工 arm/stop HTTP 契约 | 无请求体的 `POST /arm`、`POST /stop` 均返回 `200 {"ok": true}`；同一 keep-alive 连接上的非空 JSON 请求体也被正确消费 |
| 低速运动 | 命令 `0.05 m/s` 后前进位移约 `0.030 m`，最大前进速度 `0.05 m/s`，末次命令速度为 `0.0` |
| 执行器看门狗 | 停止刷新命令后进入 `指令看门狗超时` 并发布零速度；配置阈值保持 `0.28 s` |
| 仿真看门狗 | 无新命令约 `0.499 s` 后停止积分；配置阈值保持 `0.5 s` |
| 传感器 HTTP | `/frame` 返回彩色 JPEG，实测帧龄约 `0.002 s`；`/scan` 最近距离 `0.12 m`、龄期约 `0.053 s` |
| TF 与雷达投影 | 合成静态 TF 下得到 179 个投影点，最近距离约 `0.376 m`，前方净距约 `0.3756 m` |
| 信号关停 | 容器 `SIGTERM` 转为 launch 的 `SIGINT`；`ExitCode=0`、`OOMKilled=false`，`sim_node` 与 `actuator_bridge` 均正常退出 |
| 主机回归 | `50 passed, 2 skipped`；两项跳过均因未安装可选 MuJoCo 依赖 |
| 发布包 | 构建、解压与全部 SHA-256 校验通过；归档大小 `809641` 字节，SHA-256 为 `53b6c161ec2c36a31091c81b4c062947764ca48d26249836513fc6811c1177f7` |

补充说明：

- DDS 发现需要时间，短时发布脉冲不能代表稳定话题；验收使用持续发布和条件轮询。执行器 `0.28 s` 阈值由单元测试验证，运行时观察值还包含订阅者发现延迟，不能把观察端延迟当作配置值。
- 当前 TF 实现直接订阅 `tf2_msgs/TFMessage`，支持正向、反向、动态/静态和多跳变换，并使用最新保存的变换；它不按图像时间戳插值。`10000` 次随机旋转与三次精确 180° 旋转压力检查通过。
- 实车仍需检查：真实话题和 TF 帧名、相机/雷达标定、底盘订阅与里程计、实际电压曲线、低速及转向、人工停止延迟、低电压锁存、遮挡/多人场景和制动距离。SIL 不构成车辆安全认证。
