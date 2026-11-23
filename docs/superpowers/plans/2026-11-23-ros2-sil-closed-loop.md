# ROS 2 SIL 闭环实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为现有差速运动学仿真和真实执行器桥增加可重复、与车辆 ROS 域隔离的 SIL 启动与手动验收流程。

**Architecture:** 在 `follow_sim` ament_python 包中添加 launch 文件，同时启动 `sim_node` 与 `follow_bridges/actuator_bridge`。由仓库脚本加载 ROS Humble 和已构建的工作空间，设置非零 ROS_DOMAIN_ID 及 `WHEELTEC_FOLLOW_SRC`；HTTP 控制仍走既有 127.0.0.1:18768 接口。

**Tech Stack:** ROS 2 Humble、Python 3.10、ament_python、`launch`/`launch_ros`、Bash、标准库 HTTP。

**Spec:** `docs/superpowers/specs/2026-11-23-ros2-sil-closed-loop-design.md`

## Global Constraints

- 默认使用 ROS_DOMAIN_ID 42；仅接受 1–232，拒绝 0。
- `WHEELTEC_FOLLOW_SRC` 必须指向当前检出的 `src/` 目录。
- 复用现有仿真和执行器行为；不修改运动学、感知、限速、电池策略或业务控制逻辑。
- SIL 启动不得包含物理底盘驱动，也不得把 SIL ROS 域桥接到车辆域。
- 面向 Linux ROS 2 Humble；仓库当前 macOS Python 环境没有 `rclpy`。
- 手动验收命令写入运行手册；本实现计划不新增自动化测试。

---

## 文件结构

- `ros2_ws/src/follow_sim/launch/sil.launch.py`：组合启动仿真节点和执行器桥。
- `ros2_ws/src/follow_sim/setup.py`：把 launch 文件安装到 ament share 目录。
- `ros2_ws/src/follow_sim/package.xml`：声明 `launch`、`launch_ros` 与 `follow_bridges` 运行依赖。
- `ops/run-ros2-sil.sh`：加载环境、验证 SIL 域并启动组合 launch。
- `docs/ros2-sil.md`：构建、启动、闭环手动验收、VS Code 调试和排障说明。

### Task 1: 添加组合 Launch 与隔离启动器

**Files:**
- Create: `ros2_ws/src/follow_sim/launch/sil.launch.py`
- Modify: `ros2_ws/src/follow_sim/setup.py`
- Modify: `ros2_ws/src/follow_sim/package.xml`
- Create: `ops/run-ros2-sil.sh`

**Interfaces:**
- Launch 启动 `follow_sim/sim_node` 与 `follow_bridges/actuator_bridge`，两者输出到当前终端。
- 启动脚本从 `/opt/ros/humble/setup.bash` 加载 ROS；读取可选的 `SIL_ROS_DOMAIN_ID`，默认 SIL 域为 `42`。
- 启动脚本将 `WHEELTEC_FOLLOW_SRC` 设为仓库根目录下的 `src/`，并执行 `ros2 launch follow_sim sil.launch.py`。

- [ ] 创建 `launch/sil.launch.py`，构造 `LaunchDescription`，依次包含 `Node(package='follow_sim', executable='sim_node', output='screen')` 和 `Node(package='follow_bridges', executable='actuator_bridge', output='screen')`。
- [ ] 在 `setup.py` 用 `glob('launch/*.launch.py')` 将 launch 文件安装到 `share/follow_sim/launch`；保留现有 console script。
- [ ] 在 `package.xml` 增加 `launch`、`launch_ros`、`follow_bridges` 的 `exec_depend`，保留当前消息依赖。
- [ ] 创建可执行 Bash 启动脚本；定位仓库根目录，检查 `/opt/ros/humble/setup.bash` 与 `ros2_ws/install/setup.bash` 存在，加载两者，然后检查 `SIL_ROS_DOMAIN_ID` 是十进制整数且范围为 1–232。
- [ ] 设置 `ROS_DOMAIN_ID` 与 `WHEELTEC_FOLLOW_SRC`，输出当前域与启动说明，最后 `exec ros2 launch follow_sim sil.launch.py`，让 Ctrl-C 交由 ROS launch 正常清理子进程。
- [ ] 在手动操作说明中给出 Humble 工作空间构建命令；实现过程中不添加或运行自动化测试。

**Operator check:** 在 Linux ROS 2 Humble 环境构建 `follow_sim` 及其依赖，并确认 `ros2 launch follow_sim sil.launch.py` 能解析已安装的 launch 文件。该检查由用户在 ROS 环境中执行，不在当前 macOS 环境模拟 `rclpy`。

### Task 2: 编写 SIL 闭环操作与排障手册

**Files:**
- Create: `docs/ros2-sil.md`

**Interfaces:**
- 说明统一从仓库根目录运行 `ops/run-ros2-sil.sh`，默认域为 42；示例中所有观测命令显式使用同一 ROS_DOMAIN_ID。
- HTTP 命令只访问 `127.0.0.1:18768`，并沿用现有 `/health`、`/arm`、`/command`、`/stop` 请求格式。

- [ ] 写出 Humble 环境和工作空间构建命令：`source /opt/ros/humble/setup.bash`、`cd ros2_ws`、`colcon build --symlink-install --packages-up-to follow_sim`。
- [ ] 写出启动流程：在一个终端导出 `SIL_ROS_DOMAIN_ID=42` 并运行启动脚本；第二个终端加载 ROS 与 `install/setup.bash`，导出相同的 `ROS_DOMAIN_ID`。
- [ ] 写出启动检查命令：`ros2 node list`、`ros2 topic list -t`、`ros2 topic hz /odom`、`ros2 topic hz /scan` 和 `curl http://127.0.0.1:18768/health`；说明等待仿真电压和里程计就绪后再 arm。
- [ ] 写出一条低速 HTTP 命令的 JSON 请求，并说明执行器在 0.28 秒没有刷新时停止发布非零速度；观察 `/cmd_vel`、`/odom` 与 `/health` 的命令年龄和停止原因。
- [ ] 单独写出仿真器看门狗步骤：在同一 ROS 域单独运行 `ros2 run follow_sim sim_node`，用 `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'` 发出一次低速命令，再观察 `/odom` 的线速度在 0.5 秒无新命令后归零；说明该步骤隔离验证仿真器自身的超时。
- [ ] 写出人工 `/stop` 和 Ctrl-C 退出步骤；VS Code 指引要求在 ROS Humble Linux/WSL 环境打开仓库，并给出 `follow_actuator.py` 的 `accept`/`tick` 与 `kinematics_node.py` 的 `_on_cmd`/`_integrate` 断点位置。
- [ ] 加入排障表，覆盖缺少 `rclpy`、launch 包未构建、执行器找不到 `src/`、域不一致、HTTP health 503、电池数据尚未到达及命令立即超时等现象。
- [ ] 明确 SIL 域不能与车辆域桥接，且运动学仿真只验证接口连线与指定停车行为，不代表动力学精度。

**Operator check:** 按文档完成一次启动、健康检查、低速命令、超时停车和人工停止。当前环境没有 ROS 2 Humble 运行时，因此这些运行步骤由用户在目标 Linux/WSL 环境执行。

## Review Checklist

- `setup.py` 安装后的 launch 路径与 ROS package 名一致。
- 启动器使用绝对仓库路径设置 `WHEELTEC_FOLLOW_SRC`，且不会默认使用 ROS_DOMAIN_ID 0。
- 文档中的命令 JSON 字段、HTTP 路径和超时数字与 `src/follow_actuator.py` 一致。
- 当前工作树已有的用户改动保持原状；提交或暂存时只选择本计划涉及的文件。
