# 小车分层架构与具身岗位切入点

**调研日期：2026-11-23**。这份说明把仓库现有实现、ROS 2 官方文档、具身策略论文和少量当前招聘页面放在同一张地图上，回答三个问题：小车有哪些层、层与层如何交接、入行时先深耕哪一层。

## 摘要

这辆 Wheeltec C30D 的主链不是“一个 YOLO 模型直接控制电机”，而是一个带独立安全边界的闭环：传感器 ROS 2 节点收集图像/深度/雷达，普通 Python/Flask 业务进程完成人物感知、目标状态、跟随会话和速度规划，再由 ROS 2 执行桥发布 `/cmd_vel`，底盘里程计和电压反馈回来。仓库当前适合练习机器人应用与系统集成，也覆盖一部分感知和运动控制；它没有集成通用地图导航、机械臂操作或 VLA 策略训练。

对当前项目最匹配的入行路线是：先以**机器人应用/系统集成工程师**为入口，再把**感知到动作的闭环**做成技术专长。近期招聘页面把感知、导航控制、策略学习和系统集成拆成不同岗位；因此，不必把“具身”误解成只训练大模型。若后续选择 VLA，仍需补机器人接口、低层控制、仿真、数据和真实机验证。

## 1. 调研范围与问题

- **RQ1：** 本仓库的感知、状态、规划、执行、ROS 2 和硬件边界分别在哪里？
- **RQ2：** ROS 2 与更下层控制框架各负责什么？本车当前实现属于哪一种？
- **RQ3：** 对已有这套代码基础的初学者，哪条岗位路线最有连续性？

代码事实以仓库文件为准，重点检查 `src/camera_viewer.py`、`src/follow_control.py`、`src/follow_session.py`、`src/follow_motion.py`、`src/follow_sensors.py`、`src/follow_actuator.py`、`ros2_ws/src/` 和 `reference/ros2_ws/README.md`。外部资料选用 ROS 2、ros2_control、Nav2 官方文档；VLA 论文选 RT-2、OpenVLA、Octo；岗位样本选 Muguang Robotics、Bosch 和 A*STAR 的招聘页面。招聘样本用于辨认职责类别，不代表整个市场的岗位比例。

## 2. 从宏观到微观的分层

| 层 | 它回答的问题 | 本项目的对应内容 |
| --- | --- | --- |
| 任务与交互 | 用户要机器人做什么？ | Flask 控制页、启动/停止与人物选择 |
| 行为与会话 | 现在处于确认、跟随、等待还是故障？ | `FollowSession` 状态与目标管理 |
| 感知与状态 | 看到了什么，目标在哪，数据可信且新鲜吗？ | YOLO、轨迹关联、OSNet Re-ID、深度/雷达几何 |
| 运动决策 | 目标误差和障碍条件对应什么速度？ | `FollowMotion` 距离、转角、速度上限与加速度约束 |
| 中间件与接口 | 独立模块怎样交换数据和命令？ | ROS 2 topics 加本机 HTTP 接口 |
| 硬件与本体 | 指令如何变成轮子运动，反馈如何返回？ | 现有底盘 ROS 接口 `/cmd_vel`、`/odom`、`/PowerVoltage` |

安全、时间同步、坐标系、日志、仿真和部署是横切能力，不能只归到某个单独层。一个判断“能否用于机器人控制”，至少要问它是否有时间戳、坐标系、有效性/置信度和超时处理。

## 3. 本车实际运行链

### 3.1 传感器进入业务进程

`src/follow_sensors.py` 中的 ROS 2 `Sensors` 节点订阅 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info` 和 `/scan`，监听 TF，把雷达点投影到相机画面并检查时间差。它在本机 `127.0.0.1:18767` 提供帧、雷达和健康数据；`src/follow_control.py` 通过 HTTP 读取这些内容。

### 3.2 从像素变成目标状态

`src/camera_viewer.py` 的采集线程取得新帧，推理线程再调用检测和跟踪。车端 ONNX 路径由 `src/edge_detector.py` 检测人物，`src/edge_tracking.py` 做高/低分关联和 Kalman 预测；`src/edge_reid.py` 异步抽取 OSNet 特征，帮助锁定目标恢复身份。预测框能辅助关联，但当前运动命令要求可信的新观测。

`src/follow_control.py` 把人物框与深度、相机内参、雷达点和 TF 结合，形成观察值：目标方向、距离、雷达关联质量和传感器新鲜度。它是从视觉算法通向控制的关键接口。

### 3.3 从状态到速度，再回到底盘

`src/follow_session.py` 管理人物确认、跟随、丢失后等待、故障暂停和人工停止。`src/follow_motion.py` 根据目标距离、角度、障碍、数据时效、电量限制和制动参数规划线速度/角速度；会话层再限制加速度和转向变化。

业务控制循环按 50 ms 周期更新并通过 HTTP `127.0.0.1:18768` 发送命令。`src/follow_actuator.py` 是 ROS 2 `Actuator` 节点：它向 `/cmd_vel` 发布 `Twist`，订阅 `/odom` 和 `/PowerVoltage`，以 50 ms 定时器执行，并在命令超过 0.28 秒未刷新时停车。周期数字是代码配置，不等于系统已经获得硬实时保证。

### 3.4 两个容易混淆的目录

当前 `ros2_ws/src/` 含 `follow_bridges` 和开发用 `follow_sim`。后者是差速小车运动学模拟，按相同 ROS 话题契约发布里程计、雷达、电压和 TF，不是高保真物理仿真。`reference/ros2_ws/README.md` 明确将另一套 `follow_msgs`、`follow_bringup`、`follow_motion`、`follow_session` 代码标成不可运行的架构参考。当前源码/活动工作区未集成 Nav2、MoveIt 或 ros2_control；这不证明厂商底盘内部没有自己的控制实现。

## 4. ROS 2 放在架构中的位置

ROS 2 官方把模块间主要接口分为三类：Topic 用于持续数据流；Service 用于短请求/应答；Action 用于有反馈、可取消的长任务。[1] 对应到本车，图像、雷达、里程计适合 Topic；“开始跟随一个人”作为长任务在参考架构里用 Action 表达；当前运行应用则用 HTTP 接口连接 Python 业务进程和 ROS 2 桥接节点。

`ros2_control` 是可选的硬件控制框架。它在 ROS 2 中管理 controllers、hardware interfaces 和控制循环；硬件接口抽象电机/传感器状态读取与命令写入。[2] 因而“会 ROS 2”不等于“会 ros2_control”，也不等于“已经会规划控制”。在当前仓库，`Actuator` 直接发布 `/cmd_vel`，并没有看到 ros2_control controller manager 或 hardware interface 包。

Nav2 的职责范围比人物跟随更广：其架构包括状态估计、环境表示/costmap、planner、controller、behavior tree 和故障恢复。[3] 本项目根据眼前人物做局部跟随，没有证据表明它建立地图或规划去任意目标点的全局路线。若目标岗位是移动导航，Nav2、SLAM/定位、TF 和传感器融合是下一批补课点；若目标是操作，则学习机械臂运动学、MoveIt 2、轨迹和力控。

## 5. 具身技术与岗位分支

VLA 是模型策略的一条路线，不是整机栈的同义词。RT-2 把互联网视觉/语言预训练与机器人轨迹共同微调，并把动作纳入 token 输出；OpenVLA 提供开源的视觉语言动作模型和微调/部署工具；Octo 则聚焦从大规模机器人轨迹训练通用策略，并适配新的观察与动作空间。[4][5][6] 三者共同指向数据、策略训练、跨任务评测和推理部署等独立工作；与本仓库显式拆开的感知、会话和速度控制相比，端到端策略减少部分手工模块边界，但实际执行仍须连到机器人动作接口并做实体验证。

招聘页面也把职责拆得很具体。Muguang Robotics 分别列出底盘导航、机械臂轨迹/力控、感知算法、任务规划、策略学习、系统集成与系统测试；Bosch 的具身岗位同时提到 ROS 2/SDK 集成、感知/任务规划微调、现场部署；A*STAR 的岗位把 ROS 2、仿真、机器人学习、运动规划与多模态感知放在同一应用研发工作里。[7][8][9] 这说明岗位有专业分支，也有跨层交付角色；三个页面只是样本，不能据此推断哪类职位数量最多。

| 岗位方向 | 核心工作 | 与当前仓库的连续性 |
| --- | --- | --- |
| 机器人应用 / 系统集成 | ROS 2 节点、接口、时序、调试、仿真、部署、整机验收 | **最高**：项目已经串起相机、算法、控制和底盘 |
| 感知 | 2D/3D 感知、标定、融合、实时推理 | **高**：检测、跟踪、Re-ID、深度/雷达关联已有实践 |
| 运动控制 / 导航 | 运动学、控制器、定位、路径规划、动态避障 | **中**：已有跟随控制；缺通用定位和全局导航 |
| 机器人学习 / VLA | 示教数据、模仿/强化学习、策略微调、泛化评测 | **低到中**：现有检测器不是机器人动作策略 |
| 嵌入式 / 本体 | 电机控制、总线、固件、硬件接口和时序 | **较低**：底盘驱动在项目外部接口之后 |

## 6. 给你的切入建议

**第一目标：机器人应用/系统集成工程师；技术标签：感知到动作闭环。** 这条路线接着当前项目走，能把“能识别一个人”推进到“机器人在真实传感器与底盘上持续、安全、可复现地完成任务”。它同时要求 ROS 2、Python/C++、机器人几何、控制基础、日志与仿真，适合先建立全栈地图。随后再按兴趣向下走控制/导航，或向上走任务规划/VLA。

建议按 12 周做一份能面试展示的证据链：

1. **第 1–3 周：ROS 2。** 熟悉 node/topic/service/action、QoS、TF、时间戳、参数和 launch；对照本车列出节点、接口、消息频率与超时。
2. **第 4–6 周：闭环阅读。** 从 `/camera/...` 或 `/scan` 追到 Observation、`FollowSession`、`FollowMotion` 和 `/cmd_vel`，能解释每层输入、输出、失败行为。
3. **第 7–9 周：验证与故障。** 用仿真/回放构造丢帧、遮挡、雷达过期、障碍、电压异常和人工停止；记录延迟、停车、目标恢复等指标。
4. **第 10–12 周：做一个聚焦扩展。** 选其一：把长期跟随任务做成 ROS 2 Action；补 Nav2 定位与导航；或搭一个示教数据到策略评测的小型 VLA/模仿学习项目。每个项目展示架构图、指标、失败案例和复现实验步骤。

如果你已经有较强的深度学习/研究背景，并且更喜欢模型训练，可以把第 10–12 周提前：先做数据和策略评测，再用现有 ROS 2/安全知识把策略接到仿真机器人上。对于产品落地导向，先练系统集成和控制闭环更连贯。

## 7. 范围与限制

本结论针对仓库中可见代码和截至 **2026-11-23** 可查的招聘页面样本；没有检查当前运行中的机器人 ROS graph，也没有据此断言厂商底盘内部实现。招聘页面会变化，样本不等于劳动力市场统计。RT-2、OpenVLA 和 Octo 是代表性论文，不构成所有 VLA 方法的完整综述。

## 参考资料

1. ROS 2 Documentation, [Interfaces: Topics, Services, and Actions](https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/Interfaces-Topics-Services-Actions.rst).
2. ros2_control, [Controller Manager (Humble)](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html); [Hardware Interface Types (Humble)](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html).
3. Nav2, [Navigation Concepts](https://docs.nav2.org/rolling/getting_started/navigation_concepts/) and [Navigation Servers](https://docs.nav2.org/rolling/getting_started/navigation_concepts/navigation_servers/).
4. Brohan et al., [RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control](https://arxiv.org/abs/2307.15818), 2023.
5. Kim et al., [OpenVLA: An Open-Source Vision-Language-Action Model](https://arxiv.org/abs/2406.09246), 2024.
6. Ghosh et al., [Octo: An Open-Source Generalist Robot Policy](https://arxiv.org/abs/2405.12213), 2024.
7. Muguang Robotics, [Careers](https://www.muguangrobotics.com/en/careers/).
8. Bosch Group, [Embodied AI Engineer (Humanoid Robotics)](https://jobs.smartrecruiters.com/BoschGroup/744000154663199-embodied-ai-engineer-humanoid-robotics-rbcn).
9. A*STAR, [Robotics Software Engineer (ROS-I Embodied AI)](https://careers.a-star.edu.sg/default/job/Robotics-Software-Engineer-%28ROS-I-Embodied-AI%29/700-en_GB).
