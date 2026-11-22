# ROS2 业界形态骨架预览（**不可运行，不参与构建**）

> ⚠️ 读我之前先看这三句
> 1. 本目录**不在 `ros2_ws/src/` 下**，所以 `colcon build` 永远碰不到它。它不可能被误启动，也不可能接管任何车辆行为。
> 2. 代码按 ROS 2 Humble 的写法组织，但**只做过语法检查，没有编译、没有链接、没有实车运行**。把它当"形态参考"，不要当"能直接抄的成品"。
> 3. 里面每个文件都标了「对应现在仓库的哪个文件」，所以它的真正用途是**对照阅读**，不是替换。

---

## 这份骨架和真实仓库的逐项对照

| 业界形态里的东西 | 干什么 | 你仓库现在对应什么 | 差距性质 |
| --- | --- | --- | --- |
| `follow_msgs/msg/Track.msg` | 描述"一个人 + 他的框 + 他的距离" | `follow_control.py` 里的 Python dict，靠 HTTP JSON 传 | 有形态差异，**没有功能差异** |
| `follow_msgs/action/FollowPerson.action` | 可派、可问进度、可取消的跟随任务 | `POST /follow` + `seq` 请求序号 + 会话 ID | 作者手搓了一遍 Action 免费送的东西 |
| `follow_bringup/launch/*.launch.py` | 一条命令点起全栈 | `ops/robot-follow-boot.sh` 的 `while true` + `pgrep` 判重 | 启动语义差距（顺序、参数、失败处理） |
| `follow_bringup/config/follow_params.yaml` | 阈值集中，在线可改 | `config/follow_settings.json` + `follow_settings.py` | **只差"改了要不要重启"** |
| `motion_node`（订 DDS 话题） | 感知结果直接进 DDS | `follow_motion.py` + `follow_sensors.py` 的 HTTP `:18767` | 少一次序列化往返；**但会失去无硬件单测** |
| `ros2 bag` / `rviz2` | 录下来回放、现场看图 | `assets/samples/*.jpg`、`validation/*.json`、`diagnose_tracking.py` | **真实缺口，且收益最直接** |
| systemd 开机与退避重启 | 开机就有、崩了自己爬起来 | `ops/wheeltec-follow-*.service` | ✅ **已经做对了，不要动** |
| 感知/会话脱离 ROS 依赖 | 单测不需要 ROS 环境 | 现状的 `tests/` 用假 `rclpy` | ✅ **这是优点，业界也这么干** |

**读这张表最重要的结论**：右边两行 ✅ 说明"改成业界形态"不等于"全部 DDS 化"。你仓库里**比一般工业项目还好的部分**是安全层独立进程 + 可离线单测；**真正的缺口**只有包制度、参数、launch、以及 rosbag/rviz 调试。

---

## 目录结构（这就是"业界形态"长什么样）

```text
reference/ros2_ws/src/
├── follow_msgs/          ← 自定义接口：消息 + 动作（ament_cmake，要被"编译"）
│   ├── msg/Track.msg  msg/TrackArray.msg
│   ├── action/FollowPerson.action
│   └── CMakeLists.txt  package.xml
├── follow_bringup/       ← 只放启动与配置，没有业务逻辑（ament_python）
│   ├── launch/follow_bringup.launch.py
│   └── config/follow_params.yaml
├── follow_motion/        ← 决策节点：参数 + 订阅 DDS + 发布速度
│   └── follow_motion/motion_node.py
└── follow_session/       ← 对外接口：Action 服务器（可派/可问/可取消）
    └── follow_session/session_node.py
```

没写出来的包（**形状和上面两个完全一样**，照抄即可，故意省掉是为了让你只读两个范例而不是读八个）：

- `follow_perception/` —— 包现在的 `edge_detector.py` / `edge_tracking.py` / `edge_reid.py`，订图像、发 `TrackArray` + rviz 用的 `MarkerArray`
- `follow_safety/` —— 包现在的 `follow_actuator.py` 里那半份纯 ROS 的东西（看门狗、电压锁），外加 `twist_mux`
- `follow_bridges/` —— **这个已经真实存在并可运行**，在 `ros2_ws/src/`，就是阶段 A 的成果

---

## 想真正跑起来，还缺什么（**已在容器里实测**，不是猜）

1. ~~缺 ament_cmake / rosidl / cmake~~ —— **这句是错的，我实测纠正过**：
   `ament_cmake`、`rosidl_default_generators`、`rosidl_adapter`、`cmake` 在 `ros2-learn:humble` 里**都有**。
   真正缺的是**编译器与构建器**：`cc / gcc / g++ / make / ninja` 全部 MISSING（`ros-core` 是极简镜像，不装 build-essential）。
   没有它们，`ament_cmake` 类包（`follow_msgs`、`follow_bringup`）一步都编不出来 → `Track.msg` / `FollowPerson.action` 生成不出 Python 类 → 任何 `from follow_msgs...` 的 import 立刻失败。
2. 本骨架里我**故意没写** `follow_perception/` 和 `follow_safety/` 两个包，但 launch 引用了它们 → 直接 `ros2 launch` 会报 package not found。这是设计，不是漏：让你只读两个范例。
3. `motion_node` 编译出来后能启动，但没有相机/雷达数据时它会**一直停在 STOP**（感知数据超时）——这恰好是最该先看到的正确行为：**没数据就不动**。

补齐 1 只要一条 `apt-get install -y build-essential make`（约 300MB，改镜像不动仓库）。
补齐 2 就是回到渐进式路线：真做的时候一次只写一个包，写完就让它跑，不攒大爆炸。

