# 技术评估：wheeltec-c30d-yolo-follow 与工业级嵌入式机器人软件的差距

调研范围：全文读完 `src/`（14 模块，约 2200 行）、`config/`、`docs/architecture.md`、`docs/IMPLEMENTATION.md`、`docs/tuning-guide.md`、`docs/deployment.md`、`tests/`、`ops/`（systemd 单元、开机监督、发布器）、`requirements/`、`validation/`、`web/templates/follow.html`。另核对了当前分支 `feat/ros2-package` 上的迁移进展：`ros2_ws/src/follow_bridges`（正式 `ament_python` 包）、`follow_bringup`（组合 launch）和 `follow_sim`（S1 运动学仿真）。2026-11-26 在 Docker Linux ARM64 ROS 2 Humble 中完成三包构建与 SIL 验收，主机回归为 `50 passed, 2 skipped`；未连接真车，详见 `docs/ros2-packages.md`。除特别标注外，后文 `src/follow_*.py` 行号对应评估时版本；当前 ROS 桥实现位于 `src/follow_bridges/` 与 `ros2_ws/src/follow_bridges/`。

**总体判断**：这不是"玩具脚本"级别。它有明显高于业余项目的工程自觉——控制环与感知环解耦、执行器侧独立看门狗、配置范围校验、可复现发布包、真实验收数据，而且已经把"HTTP 桥 + 非 ROS 节点"识别为过渡形态并在用 strangler-fig 逐阶段替换（正式桥包化 + bringup + S1 仿真已验证）。但它的**实时性与故障隔离上限仍受当前进程模型约束**，且**安全体系停留在"软件约定"层面，缺少硬件回路与独立保护通道**。按工业标准衡量，大致处于"实验室可演示 / 产品化前夜"的位置。

---

## 1. 并发与进程模型

### 现状

三个裸线程在 Flask 主进程里启动，全部 `daemon=True`，无 supervisor、无重启、无优先级：

- `src/camera_viewer.py:571-574` — `capture` / `worker` / `control_loop` 三线程 + `app.run(host='127.0.0.1', port=VIEWER_PORT, threaded=True)`。每个 HTTP 请求再开一条新线程（Werkzeug threading），所以运行时线程数 = 3 + N(浏览器连接)。
- 第四个线程来自 Re-ID 后台 worker：`src/edge_reid.py:28-29`。
- 第五、六个在两个 ROS 桥进程里：`src/follow_sensors.py:272`、`src/follow_actuator.py:213`（`rclpy.spin` 线程）+ `ThreadingHTTPServer`（`src/follow_sensors.py:271`、`src/follow_actuator.py:214`）。

锁拓扑是两级嵌套，方向正确但粒度粗：

- `lock` + `observations_changed`（同一把锁的 Condition）：`src/camera_viewer.py:43-45`
- `motion_lock` 单独串行化"计算 + 下发"：`src/camera_viewer.py:142`，写入侧 `src/camera_viewer.py:364`、`393`、`452`
- 控制环内 `with motion_lock: with lock:`（`src/camera_viewer.py:142-143`）→ 会话更新 + HTTP 下发都在 `motion_lock` 临界区内完成。**这意味着一次最坏 0.22 s 超时的底盘 HTTP 调用（`CONTROL_TIMEOUT = 0.22`，`src/camera_viewer.py:65`；调用点 `src/camera_viewer.py:155`）会阻塞 `/detection`、`/follow`、`/shutdown` 拿锁**（它们都要 `motion_lock`，见 `364`/`393`/`452`）。人工"停止"按钮因此可能被拖到数百毫秒级——这是安全相关的延迟路径。

GIL / CPU 推理阻塞控制环路：**问题确实存在，但被架构部分掩盖了。**

- 车端 ONNX Runtime 检测在主进程 `worker` 线程内同步执行：`src/edge_detector.py:12`（`intra_op_num_threads = 2`）、`src/edge_detector.py:27`（`self.session.run(...)`）。ONNX Runtime 的原生算子在 `session.run` 期间会释放 GIL，所以纯推理时控制环还能跑；但 `worker` 里的 **PIL 预处理**（`src/edge_detector.py:24-26`，`Image.new` / `paste` / `resize` / `asarray`）和 **JPEG 编码**（`src/camera_viewer.py:337`，`quality=48`）都是纯 Python/C 混合、持 GIL 的，会与 `control_loop` 抢 CPU。
- `control_loop` 的周期补偿是"事后追帧"而非绝对时间调度：`src/camera_viewer.py:167` `time.sleep(max(.005, CONTROL_PERIOD_S - (time.monotonic() - tick)))`。一旦单轮耗时 > 50 ms（HTTP 超时、GIL 争用、GC），下一轮立刻补睡 5 ms 继续跑，周期抖动直接进控制律。同理 `src/camera_viewer.py:237`。
- 非车端分支更糟：`src/camera_viewer.py:295-296` 走 ultralytics `model.track(..., imgsz=480, device=DEVICE)`，`DEVICE` 由 `torch.backends.mps.is_available()` 决定（`src/camera_viewer.py:36-39`）。这条路径下推理与控制同进程同 GIL，实测 P95 延迟 230.6 ms 就是这种结构下的产物（`validation/idle-30min.summary.json:max_reported_latency_p95_ms`）。

链路延迟（有自测数据支撑）：

代码把延迟拆成四段埋点，这点做得好：`src/camera_viewer.py:223-224`（`capture_fetch_ms` / `capture_age_ms`）、`254`（`worker_start_age_ms`）、`316`（`worker_end_age_ms`）、`165`（`control_performance`，注释明确 basis 为 "camera receive to control bridge acknowledgement"，`src/camera_viewer.py:543`）。综合各级超时（相机 `FRAME_TIMEOUT = 0.5`、雷达 `SCAN_TIMEOUT = 0.4`，`src/follow_control.py:21-22`；传感器缓存窗口 0.25 s，`src/follow_sensors.py:35,40`），**从光子到 `/cmd_vel` 的实际链路 ≈ 150–250 ms（静止实测 P95 230.6 ms），最坏情况可达 ~500 ms**。控制律自己也承认这一点并把延迟喂进制动余量：`src/follow_motion.py:71` `latency = min(.6, max(.1, now - frame_at + .1))`。也就是说作者知道延迟大，是用"降低速度上限"来补偿延迟，而不是压缩延迟。

实时性保障：**几乎没有。** 全仓搜索 `chrt|taskset|os.nice|SCHED_FIFO` 无任何命中。反向操作还有一处——`ops/wheeltec-follow-edge.service:22` 设 `Nice=5`，即整个感知+控制进程被**降权**；配合 `OPENBLAS_NUM_THREADS=1`、`OMP_NUM_THREADS=2`（同文件 15-16）说明作者意识到核竞争，但选择了"限线程数"而非"绑核 + 提优先级"。ROS 桥在容器内以默认策略运行（`ops/robot-follow-boot.sh` 的 `docker exec -d`），无 SCHED_FIFO、无 CPU pinning、无内存锁定（`mlockall`）。

### 工业做法 / 差距

1. **进程隔离**：工业上感知、规划、控制、I/O 各为独立进程（ROS2 composition 或独立 node + `ros2 run`），一个 OOM/段错误不牵连其他。当前 `worker` 若因模型异常崩溃（`src/camera_viewer.py:305-308` 捕获后置 `inference_error`，但没有线程重启），控制环虽然能进入 `fault_paused`（`src/camera_viewer.py:148-150`），但整个 Web 面板也随进程一起失去意义；只有外部 `boot_health.py` 的退避重启兜底（`ops/boot_health.py:73-98`），其判据依赖 HTTP 可读性，属于"慢速外部看门狗"。
2. **绝对周期 + 优先级**：控制环应使用 `clock_nanosleep(TIMER_ABSTIME)` 或 ROS2 WallTimer + `SCHED_FIFO` + 绑核，避免累积漂移与调度抢占。当前 `time.sleep` 相对睡眠 + `Nice=5` 在 Pi5 4GB 上与 Docker/astra/lslidar/wheeltec_robot_node 争抢，尾延迟不可控。
3. **推理与同进程控制分离**：业界标准做法是把 NN 推理放独立进程（或 RKNN/NPU/GPU 加速卡 + DMA），控制进程只消费结构化结果。当前 PIL 预处理/JPEG 编码留在主进程是关键瓶颈点。
4. **安全路径不应与 UI 争锁**：`/follow` 停止命令走 `motion_lock`（`src/camera_viewer.py:393`），而 `motion_lock` 内含网络 I/O。工业上急停/停止必须有独立的、不经业务锁的下发通道。

---

## 2. 数据一致性与时间戳

### 现状

这一块是本仓库**最用心、也最接近工业水准**的部分，但仍不是工业实现。

**用了消息头时间戳，也用了墙钟，两者混用**：

- 相机/深度/雷达去重与配对全部基于 `message.header.stamp`：`src/follow_sensors.py:30-31`（`stamp_ns`）、`97`、`103`、`117`（重复包丢弃）。
- 新鲜度判定一律用 `time.monotonic()`：`src/follow_sensors.py:99`（`self.colors.append((time.monotonic(), message))`）、`105`、`119`、`156-171`。也就是"到达时刻"而非"采集时刻"作为主时钟。
- **跨进程时间对齐靠"age 差值"重建**：传感器桥返回 `age` 与 `request_age`（`src/follow_sensors.py:208-209`，`requested_at` 在 handler 入口取 `src/follow_sensors.py:230`），应用层用 `captured_at = started - age`（`src/follow_control.py:124`）、`LidarFrame(started - scan_age, ...)`（`src/follow_control.py:163`）反推采集时刻。注释写明意图：`src/follow_control.py:120` "The request start is a conservative clock reference even across hosts."。**这套办法在同机 loopback 下可用，但它把"HTTP 往返 + JSON/base64 序列化耗时"全部算进了 age**，系统性高估延迟；且没有任何单调时钟一致性保证（两进程各自 `monotonic()`，靠代数抵消，无法验证）。

**没有 message_filters，手写近似同步**：

- 全仓搜索 `message_filters|approximate_time|strict_sync` → **零命中**。
- 彩色-深度配对是手写的"最近邻 + 硬阈值"：`src/follow_sensors.py:34-58`（`aligned_pair`），要求 `frame_id` 相同、分辨率相同（`46-47`）、`gap_ns <= 60_000_000`（`src/follow_sensors.py:54`）；否则抛错。color-only 模式另有一套并行实现：`src/follow_sensors.py:157-169`。**两份逻辑高度重复**（`min(compatible, key=...)` 出现在 `52` 和 `168`），是典型的一致性隐患。
- 雷达与相机的同步只有一个窗口常数：`SCAN_FRAME_WINDOW_S = 0.20`（`src/follow_sensors.py:26`，用于 `130`），超过就丢弃投影。这是"就近取一帧"，不是插值/外推，也不是带协方差的时间不确定度传播。

**TF 用了，但只在传感器桥内部，且几何仍有硬编码**：

- 当前包化实现直接订阅 `tf2_msgs/TFMessage` 的 `/tf` 与 `/tf_static`，在 `src/follow_bridges/follow_sensors.py` 的 `transform_between()` 中维护变换图，支持正向、反向和多跳组合；不再依赖 `tf2_ros.Buffer` / `TransformListener`。查询仍取最新保存的变换，不按彩色帧时间戳插值，车体转动时雷达投影仍会带旋转误差。
- 四元数归一化、旋转矩阵、逆变换和组合均在桥内用 NumPy 实现，`project_scan()` 通过旋转矩阵平移雷达点。`tests/test_sensor_tf.py` 覆盖动态/静态、直接/反向/多跳、无效边回退和精确 180° 旋转；另完成 `10000` 次随机旋转与三次精确半周压力检查。
- 相机外参走 TF，但**车头距离偏移是配置文件里的常数**：`SETTINGS.camera_to_front_m`（`src/follow_settings.py:16`，实际值 `config/follow_settings.json:2` = 0.0244），减 offset 的操作散落在 `src/lidar_geometry.py:13,21`、`src/follow_control.py:148,161-162`、`src/follow_motion.py:47,56`、`src/camera_viewer.py:521`。**同一个物理量在 6 处分别减去**，容易漏改。
- `config/calibration.json` 记录了来源与置信度（`geometry_basis: "factory model, not a physical tape measurement"`，第 9 行；`stop_tests_completed: 0`、`brake_tests_completed: 0`，第 10-11 行），诚实度高。但**这个文件在代码里从未被读取**——grep 全仓只有 `ops/build_release.py:22`（打包）和 `validation/deployment-audit.json:104`（清单）引用它。真正生效的是 `follow_settings.json` 里的 `camera_to_front_m`/`geometry_verified`。标定溯源与实际参数之间**没有代码级绑定**。
- 走廊半宽等几何量硬编码：`src/lidar_geometry.py:12,19` 用 `body_half_width_m + corridor_margin_m`（配置里有，OK）；但 `src/lidar_geometry.py:15` 的 `.16`（聚类分裂距离）、`22,35` 的 `.03`（后方剔除）、`27,38` 的归一化视口边界是常数。

### 工业做法 / 差距

1. **统一时钟**：工业系统强制所有节点使用同一时间基准（PTP/chrony + ROS2 sim/system time 一致策略），并检查 `header.stamp` 与接收时刻之差是否合理（`message_filters::TimeSynchronizer` 或自定义带最大偏差的策略）。当前方案对 loopback 有效，一旦跨主机部署（README 提到局域网入口待配置）就会失效。
2. **按时刻查 TF**：应在彩色帧时间戳上查询或插值，并区分“暂未可用可重试”与“永久失败”。当前直接消费 `TFMessage` 的实现只保存最新变换，也没有带时间戳的历史缓存；`projected_lidar()` 在缺变换时返回 `None`，最终让 `src/follow_motion.py:46` 报“雷达数据过期”停车。行为安全，但日志没有 TF 失败原因，根因不易定位。
3. **单一几何真源**：车头净距应由 URDF/TF 树派生（`base_link → camera_link → lidar_link → front`），一处定义。当前 6 处手工减 offset + 未被读取的 `calibration.json`，是产品化时必须消除的隐患。
4. **同步逻辑收敛**：`aligned_pair` 与 color-only 分支应合并为一个带策略参数的函数，避免两套阈值漂移。

---

## 3. 控制算法

### 现状

**结论：纯 P 控制器 + 多级限幅 + 滞回 + 加速度斜坡，无前馈、无轨迹跟踪、无避障、无路径规划。**

主实现在 `src/follow_motion.py`（154 行，无 HTTP/ROS 依赖，可测试性好）：

- 前进 P 律：`src/follow_motion.py:150` `forward = min(forward, cfg.forward_gain_per_s * max(0., distance - cfg.stop_m))`，增益 0.65（`config/follow_settings.json:8`）。
- 转向 P 律：`src/follow_motion.py:142-143` `-cfg.turn_gain_per_s * obs.angle_rad`（增益 0.90，第 9 行），带死区 5°（`turn_deadband_deg`）。
- 角度→前进的线性降权（不是真正的耦合项，只是比例衰减）：`src/follow_motion.py:144-145`，边界 9°/12°；`146-147` 目标贴近画面边缘时 steering 直接归零。
- 制动距离上限（这是亮点，比多数业余项目强）：`src/follow_motion.py:136-139` 用 `v = a(√(L²+2d/a) − L)` 解析式限制速度，保证考虑感知延迟 `L` 后仍能刹住；`72-73` 用 `margin = speed*latency + speed²/(2a)` 动态抬高停车阈值。
- 滞回：`src/follow_motion.py:88-91`（`stop_m=0.30` 停、`resume_m=0.40` 起）。
- 障碍迟滞确认：`src/follow_motion.py:98-105`（需连续清空 `obstacle_clear_s=0.5`）。
- 单束异常回波需二次采样确认：`src/follow_motion.py:77-85`（`.15` s 持续 + `lidar.at > isolated_sample_at`）。
- 距离连续性门限（防止背景/前景跳变）：`src/follow_motion.py:50-64`，需 `range_hits >= 2` 才信任。
- 加速度斜坡在会话层：`src/follow_session.py:134-136`，线加速度 0.30 m/s²、角加速度 0.60 rad/s²，`dt` 由控制环实测间隔给出（`src/follow_session.py:68`，钳到 [0, 0.2]）。

**缺失的能力**：

- **无速度闭环**。里程计只用来算"当前速度以推制动余量"（`src/follow_session.py:123-125`）和执行器健康判断（`src/follow_actuator.py:55-59`、`146`），从不反馈修正 `forward/turn`。mini_mec 是差速底盘，打滑、地毯、低电压导致的速度偏差完全不被补偿。
- **无偏航角/IMU 融合**，转弯时目标方位角被相机运动污染，只能靠跟踪器的 `gmc_method: sparseOptFlow` 在 WSL 分支补偿（`config/botsort_reid.yaml:10`），车端 `EdgeTracker` 无对应机制。
- **无避障/绕行**。检测到前方障碍只会停车等待（`src/follow_motion.py:106-108`），不会横向规避；跟随对象绕开家具时车只能原地等。
- **无路径规划、无地图、无 SLAM 集成**——纯反应式（reactive）跟随。
- **无预测**：`src/edge_tracking.py:1` 的模块 docstring 明确 "Predictions are for association only. Only current detections can command motion."，设计上是刻意的保守选择（安全性优先），代价是人快速移动时响应滞后。

**关键问题：存在两套并行的控制器，其中一套是死代码。**

- 现役：`FollowMotion.plan()`（`src/follow_motion.py:35`），经 `src/follow_session.py:126` 调用。
- 遗留：`plan_follow()`（`src/follow_control.py:320-356`）和 `plan_visual_follow()`（`src/follow_control.py:292-317`）——**grep 全仓除定义处外零调用者**。这两套里有各自独立的魔法数字（`src/follow_control.py:308` `-0.9*angle` clip ±0.16；`334` `gain = 0.25`；`311-317` 另一套 speed_limit 逻辑），与 `follow_motion.py` 的语义已经分叉。同样死亡的还有 `fetch_lidar()`（`src/follow_control.py:172`）和 `trial_depth_block()`（`src/follow_control.py:280`）。**留着两套控制器是产品代码里最危险的一类债**：改参数时容易改错那套。

### 工业做法 / 差距

工业人物跟随通常分层：`(1) 目标状态估计（带 CV/EKF 的人运动模型 + 可达集预测）→ (2) local planner（DWA / TEB / MPCC）在速度空间内同时满足"跟上目标"与"避开障碍" → (3) 底层 velocity controller（PID 或串级，闭环跟踪 odom 反馈）→ (4) 底盘电机 FOC`。差距集中在：缺 (1) 的目标预测、缺 (2) 的局部规划（只有硬门限）、缺 (3) 的速度闭环。另外工业实现会把 gain/limit 之类做成在线可调参数（ROS2 parameter + dynamic reconfigure），当前必须重启进程（见第 6 节）。

不过要公平指出：`src/follow_motion.py:136-139` 的解析制动上限 + `72-73` 的动态停车余量，本身就是 DWA 中"velocity obstacle / braking curve"思想的简化版，方向是对的，只是没有扩展到横向。

---

## 4. 安全机制

### 现状

**已实现的（比我预期的多）**：

| 机制 | 证据 | 评价 |
| --- | --- | --- |
| 指令看门狗（执行器侧独立） | `src/follow_actuator.py:24` `COMMAND_TIMEOUT_S = 0.28`；`116-126` 超时即置零并记 `stop_reason="指令看门狗超时"`；`46` `create_timer(0.05)` 每 50 ms 检查 | 好。不依赖上层存活 |
| 零速突发广播 | `src/follow_actuator.py:25` `STOP_BURST_S = 1.5`；`38/73/126` 设 `zero_until`；`127-131` 到期前持续发零 | 好，符合"安全信号宁可重复广播"的原则（作者自己在学习笔记里写了这条：`docs/ros2-learning-notes-01.md:102-108`） |
| 停止与发布共锁（防陈旧非零） | `src/follow_actuator.py:61-62` 注释 + `69-75` `stop()` 同步 publish 零 | 好，消除了 stop/publish 竞态 |
| 服务端二次限速 | `src/follow_actuator.py:97-101` 拒绝越界数值（含 `bool` 伪装成 int 的检查）、`109` 再按电量钳制 | 好，纵深防御 |
| 低压保护 + 锁定 | `src/battery_state.py:26-30`（1 s 中位数滤波，低于 `battery_stop_v` 持续 3 s → `latched=True`）、`44-51`（无数据即 limit=0）；解锁必须人工 `arm()`：`src/battery_state.py:57-61`、`src/follow_actuator.py:84-85` | 好，且有"不可自动恢复"的正确设计 |
| 会话层故障锁存 | `src/follow_session.py:79-81`（`fault_latched` 后持续 `fault_paused`）、`src/follow_session.py:33`（仅 `start()` 清除） | 好 |
| 底盘订阅端存在性检查 | `src/follow_actuator.py:102-103`（`get_subscription_count() < 1` 拒动）、`146`（health ok 需 subscribers>0 且 odom 新鲜 <1 s） | 好，能发现"驱动节点挂了但桥还在" |
| 关机前静止确认 | `src/camera_viewer.py:459-475`：轮询 health，要求 `active=False`、下发为零、`odom.age_s<0.5`、\|v\|<0.01、\|w\|<0.02 **连续 0.3 s**，否则 `raise RuntimeError('未确认底盘持续零速度，未执行关机')` | 相当扎实，且有专门测试（`tests/test_follow.py:341-365`） |
| VIEW_ONLY 熔断 | `src/camera_viewer.py:26`、`152`、`407` | 好 |
| 试跑限时 | `src/follow_session.py:35` deadline、`74-75` 到时停车、`137-139` 额外钳到 0.07 m/s / ±0.12 rad/s | 好，但 `LOCAL_TRIAL_SECONDS=10`、`0.07`、`.12` 全是硬编码（`src/camera_viewer.py:73-74`、`src/follow_session.py:138-139`） |
| 制动未标定时降速 | `src/follow_motion.py:130-131`（`not cfg.braking_verified` → 钳到 `fallback_speed_mps`）；`config/follow_settings.json:4` `braking_verified: false` | 亮点：把"未完成实测"编码成运行期约束，并有测试守护（`tests/test_follow.py:142-150`） |

**缺失的（这是与产品级的核心差距）**：

1. **完全没有硬件急停回路。** 全仓搜索 `e-stop/emergency/急停` 只在 `docs/ros2-learning-notes-01.md:106` 命中一句概念描述，代码里没有任何实体按钮、无线急停、无继电器切断。唯一物理开关是电池总开关（`src/camera_viewer.py:481` 的提示文案"电池总开关仍需手动关闭"）。工业上室内移动机器人必须有 ISO 13850 类别的急停装置，且**急停路径不得经过软件**。
2. **安全功能与被保护功能在同一故障域内。** 看门狗跑在 `follow_actuator.py` 进程里，该进程与感知/Web 同在 Pi5 上、同受 Docker/内核 OOM 影响；Pi5 崩溃 → 桥进程死 → 不再发零速 → **底盘侧是否有自己的 cmd_vel 超时？代码里看不到任何依据**。`src/follow_actuator.py:102` 只检查订阅者数量，不验证 STM32 固件的通信丢失保护。这是 Cat 2 / PL c 都达不到的根本原因：没有独立于主控的安全通道。
3. **无碰撞检测/力矩监控**，无悬崖/台阶检测（2D 雷达原理性看不到，作者在 `docs/ros2-learning-notes-01.md:92` 自己指出了这点，但没实现对策），无地面材质假设变化处理。
4. **无安全速度曲线分区**（例如靠近人 0.5 m 内必须 ≤0.1 m/s 的强制低速带 —— 现有 `near_slowdown_m=0.60`/`approach_speed_mps=0.05`（`src/follow_motion.py:140-141`）是性能调参，不是安全额定值，且可被配置覆盖）。
5. **网络安全（IEC 62443）几乎为零**。做了本地回环绑定与 Origin 校验（`src/camera_viewer.py:22`、`85-91`、`94-98`），这很好；但：
   - `/shutdown` 允许远程触发主机关机，鉴权仅靠 `boot_id == START_SESSION`（一个 `secrets.token_hex(8)`，`src/camera_viewer.py:69`）+ 字面量 `'shutdown-host'`（`448`）。而这个 token **在每个 `/status` 响应里明文回传**（`src/camera_viewer.py:546` `start_status=[START_SESSION, now, start_generation]`），前端也原样回传（`web/templates/follow.html:499`）。任何能读页面的脚本即可关机。无认证、无审计、无速率限制。
   - 无 TLS、无鉴权中间件、无 CSRF token（仅同源检查）。`MAX_CONTENT_LENGTH = 4096`（`src/camera_viewer.py:23`）是唯一资源限制。
   - `sudo -n systemctl poweroff`（`src/camera_viewer.py:476`）需要 sudoers 免密授权，权限面扩大。
6. **异常吞掉过多类型**：`SENSOR_ERRORS = (OSError, HTTPException, ValueError, KeyError, TypeError)`（`src/camera_viewer.py:62`）——把 `KeyError`/`TypeError`（通常是程序 bug）当可恢复网络故障处理，会掩盖真实缺陷。
7. **执行器 HTTP 层有挂起风险**：`src/follow_actuator.py:181-202` 的 `do_POST` 只捕获 `ValueError/TypeError/JSONDecodeError/RuntimeError`；`int(self.headers.get("Content-Length","0"))`（`183`）遇畸形头抛 `ValueError` 尚可，但 OSError（断连）不在捕获列表内，且 `reply()`（`165-172`）写 socket 无 try（对比 `src/follow_sensors.py:257-260` 有捕获 BrokenPipe）。另外**GET /health 无 Content-Length 读取限制**（`174-179`）。
8. **无优雅退出**：`camera_viewer.py` 没有 SIGTERM handler（对比 `src/follow_actuator.py:217-221` 有），收到 systemd stop 时直接死，最后一帧速度由看门狗 280 ms 后才清零。

### 工业做法 / 差距

产品级需要：独立安全 PLC/安全 MCU（双通道急停、安全扭矩截止 STO、Cat 3 / PL d）、安全功能与被保护功能物理分离、看门狗心跳双向（底盘反过来监督主控存活）、安全速度额定值固化在只读存储不可配置、IEC 62443 的分区/认证/审计日志/最小权限。当前实现可评为"软件级安全约定完备、硬件级安全缺失"，属于研究原型阶段。

---

## 5. 感知链路

### 现状

**YOLO11n 车端 ONNX CPU 推理**：

- 输入尺寸由 ONNX 固定，代码不假设：`src/edge_detector.py:16-18` 从 session input shape 读 `height/width`，`21-25` 做 letterbox（灰边 114）。服务里注入模型路径 `ops/wheeltec-follow-edge.service:8`（`follow-yolo11n.onnx`）。文档称相机已改为 320×240@30 Hz 以匹配 320×320 输入（`docs/IMPLEMENTATION.md:27`），启动脚本确实显式设 `color_width:=320 color_height:=240`（`ops/robot-follow-boot.sh`）。
- **实测吞吐：FPS 中位数 13.67**（`validation/idle-30min.summary.json:median_reported_fps`），即 ~73 ms/帧端到端；P95 延迟峰值 230.6 ms（同文件）。RSS 138.6→139.7 MiB、峰值 140.2 MiB、单 PID 稳定（同文件 `rss_*`、`pids`），30 min 无 HTTP 错误——**内存无泄漏、无进程重启**，这是有价值的正面证据。
- 线程配置：`src/edge_detector.py:12-14` intra=2/inter=1 且 `session.intra_op.allow_spinning=0`（禁用自旋，避免占满核，正确）；Re-ID 更保守，`src/edge_reid.py:46-47` intra=inter=1 + 禁自旋。加上服务的 `OMP_NUM_THREADS=2`/`OPENBLAS_NUM_THREADS=1`，说明作者踩过核竞争的坑。
- 但**采集节奏与推理能力不匹配**：`EDGE_CAPTURE_PERIOD_S = 0.05`（`src/camera_viewer.py:67`、`236`）意味着尝试 20 Hz 拉帧，而推理只有 ~13.7 FPS。靠"只处理最新帧"隐式丢帧（`src/camera_viewer.py:249-250` 的 `wait_for(sequence != seen)` + `250` 直接取 `raw`），不会积压，但结果是**感知帧率随机抖动于 10–14 Hz**，控制环却按 20 Hz 消费"最后一次观测"，观测年龄均值≈73 ms、尾部可达 200+ ms。

**检测/跟踪/Re-ID 耦合方式**：

- 三者**同步串联在同一个 `worker` 线程**里：`src/camera_viewer.py:284`（`tracker.track(...)` 内部依次调 `reid.poll` → `detector.detect` → 关联 → `reid.submit`，见 `src/edge_tracking.py:115,126,161-165,201`）。只有 Re-ID 推理被异步化到独立线程 + 容量 1 的队列（`src/edge_reid.py:19-20`，`77` 满则丢任务，`69-71` 结果满则覆盖）——**有界、不堆积、自适应降频**（`src/edge_reid.py:68` `interval_s = max(.5, min(3., elapsed*10))`），这部分设计得好。
- 自研 ByteTrack 式两阶段关联：`src/edge_tracking.py:137-160`（high/low 两轮匈牙利匹配，`linear_sum_assignment` 在 `148`），代价函数 = IoU + 躯干颜色直方图相似度（`145-147`，直方图 64 bin 见 `20-25`）。卡尔曼 8 状态（`33-66`），但 `covariance -= gain @ self.covariance[:4,:]`（`63`）是**简化的协方差更新，不是标准 Joseph 形式**，长期数值对称性/正定性无保证（`covariance` 也不对称化）。
- 身份歧义处理很细：重叠框标 `ambiguous_boxes`（`132-133`）、选定目标遇歧义或超时则挂 `needs_reid_at`（`154-158`）、恢复候选需同时过相似度与 margin（`187-188`）、**恢复出的轨迹会被重新编号为原 selected_id**（`189-194`）。这些是真实遮挡场景的经验沉淀。
- **潜在故障**：`src/camera_viewer.py:324` 无条件访问 `tracker.reid_status`，而 `src/edge_reid.py:24-26` 在模型文件缺失时提前 `return`（不设 `self.path`），此时 `src/edge_tracking.py:94` 的 `dict(self.reid.status, ...)` 仍可用（status 已初始化），但若 `EDGE_MODEL` 为真而 `models/osnet_x0_25.onnx` 缺失，Re-ID 静默降级为 `ready=False`，`submit` 直接返回（`src/edge_reid.py:77`），**身份恢复能力无声消失**，只有 `/status` 的 `reid.ready` 能看到（`src/camera_viewer.py:544`）。
- **命名与实现不符**：`config/botsort_reid.yaml` 是 BoT-SORT 配置（`tracker_type: botsort`、`gmc_method: sparseOptFlow`、`with_reid: true`），但**只被非车端 ultralytics 分支使用**（`src/camera_viewer.py:42`、`295`）。车端走自研 `EdgeTracker`，完全不读这个 yaml。文件名容易误导。

**测距与标定验证**：

- 三种测距来源，优先级明确：雷达簇关联（`src/follow_control.py:181-223`）> 深度 ROI（`src/follow_control.py:253-262`，取 25–75 百分位、要求 IQR≤450 mm 且有效率≥60%）> 画面尺寸推断（`src/follow_motion.py:109,152`）。
- 歧义保护写得很好：`src/follow_control.py:205-206` 若前/后景候选差 >0.30 m 直接返回 None（注释 "Ambiguous foreground/background: wait for an unambiguous observation"）；`220-222` 用"人物宽度合理性"反Reject墙面远回波。
- **精度评估：基本没有量化闭环。** `validation/` 只有 5 个文件（`validation/README.md` 表格），且每个都自我标注 scope 不含物理测距精度。`docs/IMPLEMENTATION.md:45` 诚实列出待验收项：10 分钟连续跟随、20 次遮挡恢复、多人交叉、20 次 25–35 cm 无接触停车、实测制动——**全部未完成**。`config/calibration.json:10-11` 的 `stop_tests_completed: 0`、`brake_tests_completed: 0` 是同一事实。
- **数据集与再训练闭环：不存在。** `models/` 只有 `wheeltec-yolo11n.pt`（预训练 COCO，未微调）、OSNet 官方 Market1501 权重 + 导出脚本（`models/osnet_export_source.py`）+ 许可证。`assets/samples/` 是历史相机样本，但无标注、无训练脚本、无评测指标（mAP/IDF1/MOTA/HOTA 全无）。`docs/IMPLEMENTATION.md:64` 也坦白"本项目是针对小车的轻量实现，不是原项目完整评测管线"。
- 诊断工具存在且实用：`ops/diagnose_tracking.py`（逐帧打印 raw/out/hits/gate/reid/ranges，只读）、`ops/record_validation.py`（遥测 + RSS 采样）。

### 工业做法 / 差距

1. **算力选型错误是根因**：Pi5 CPU 跑 YOLO11n 天花板就是 ~15 FPS。产品会用 NPU/HAT（RKNN）、Coral、Jetson 或 GPU，把检测压到 <20 ms 并留余量；或用更小模型 + INT8 量化（当前是 FP32 ONNX，`src/edge_detector.py:26-27` 无量化痕迹）。
2. **感知-控制频率解耦不足**：工业做法是感知输出带 `header.stamp` + 协方差的 target state，控制环以固定周期做外推（EKF predict）后再决策。当前刻意禁止预测驱动（`src/edge_tracking.py:1-4`），等于用"更保守的行为"换"更少的代码风险"，可理解但限制了速度上限。
3. **缺离线评测管线**：产品团队会有固定的跟随评测集（MOTA/IDF1、ID switch/km、测距 RMSE vs. 激光测距仪真值、停车距离分布）。当前只有零散实车记录。
4. **缺标定验证自动化**：`geometry_verified` 是人工布尔标志（`config/follow_settings.json:3`），无自动一致性检查（例如定期用已知距离的标定板/墙角回归 `camera_to_front_m`）。
5. **正面评价**：`src/camera_viewer.py:223-224,254,316` 的四段延迟埋点 + `validation/*.summary.json` 的公开负面边界声明，比很多"号称工业级"的项目诚实且可运维。

---

## 6. 代码工程质量

### 现状

**类型注解：形同虚设。** `grep "def .*->" src/*.py` → **0 个返回类型注解**。仅 `src/follow_control.py:3` 有 `from __future__ import annotations`，其 dataclass 用了 `float | None`（`46`、`56-57`、`67`、`74-79`）、`np.ndarray`（`45`、`53`）、`tuple`（`48`、`79`、`181`）等；`src/follow_motion.py:11-18` 有字段注解；`src/edge_tracking.py:35-44` 有。**但 `src/camera_viewer.py`（574 行、最核心的编排层）几乎裸奔**（`: str`/`-> ` 计数 = 2），`src/follow_session.py` 计数 = 0。全仓无 mypy/pyright 配置，CI 只做 `compileall`（`.github/workflows/ci.yml` "Check Python syntax"）——**语法检查，不查类型、不查 lint**。也没有 ruff/flake8/black 配置（`.editorconfig` 存在，`pyproject.toml` 只有 pytest 段）。

**错误处理：分层清晰但有过度吞噬。**

- 好的方面：领域校验集中在配置加载，15 个 `raise`（`src/follow_settings.py:54-99`），包括"保护间隔不可缩短"这类安全下限（`97-98`）、速度层级链式校验（`74-77`）、角度单调性（`80-83`）；`load_settings` 会拒绝 `NaN/inf/负值/bool 冒充数值`（`58-71`）。测试覆盖非法配置（`tests/test_follow.py:218-224`）。
- 差的方面：`src/camera_viewer.py:148-150` 用 `except Exception` 兜住会话更新失败并转入 `fault_paused`（安全上正确，但只 `logging.exception`，无错误码/无上下文变量）；`src/camera_viewer.py:305-308` 把模型异常文本直接拼进面向用户的 status（`f'模型错误：{exc}'`）；`src/edge_reid.py:72-73` 捕获 `Exception` 后只把 `str(exc)` 存进 status dict，**线程静默死亡且不重投**，之后 Re-ID 永久不可用（因为 `jobs` 队列无人消费、`status['ready']` 保持 True 直到下次 poll 才发现——实际上 ready 停在 True，`submit` 会持续入队然后堆积，`77` 的 `self.jobs.full()` 才拦住）。

**日志：几乎只有事件日志。** 全仓 logging 调用 4 处：`src/camera_viewer.py:24`（压制 werkzeug）、`149`、`306`（exception）、`src/follow_session.py:59`（`logging.getLogger("follow.events").warning(json.dumps(event))`）。

- 亮点：状态迁移日志是**结构化 JSON**、带 session_id/state/reason/target/sensor ages/下发速度（`src/follow_session.py:55-59`），且只在状态真正变化时打（`54`）。这一条达到可运维水平。
- 缺陷：① 级别误用——正常状态迁移打在 `warning`；② 两个 ROS 桥通过 `rclpy` 记录就绪事件，传感器桥还会记录 HTTP 请求失败，但 HTTP 访问日志被显式静音，执行器缺少同等程度的故障日志；③ 无 log rotation、无 trace/request id、无统一 formatter。桥日志已有改善，但还不能替代完整可观测性。

**配置管理**：集中式 `Settings` frozen dataclass + JSON 覆盖 + 严格范围校验，质量高（`src/follow_settings.py:12-99`）。但：

- **配置不可热更新**：`SETTINGS = load_settings()` 在 import 时求值（`src/follow_settings.py:102`），改参数必须重启进程；`VERSION = "2.0.1"` 也硬编码在此（`src/follow_settings.py:9`），与 `CHANGELOG.md`、`releases/` 名字三处需同步。
- **大量参数绕过 Settings**：`src/follow_control.py:21-37`（`FRAME_TIMEOUT`、`SCAN_TIMEOUT`、`MIN_CLEARANCE_M`、`MAX_FORWARD_MPS`、`AUTO_MAX_FORWARD_MPS`、`VISUAL_CLOSE_WIDTH/HEIGHT`、`FAST_CLEARANCE_MM`…）、`src/camera_viewer.py:41,63-74`（端口、超时、周期、试跑秒数）、`src/follow_actuator.py:20-26`、`src/follow_motion.py:50-53,74,82,110,121,124,146,152`、`src/edge_tracking.py:119,146,172-173,184,186`、`src/lidar_geometry.py:15,22,35`、`src/edge_reid.py:59-60`（ImageNet 均值方差）。**同一类知识有两种存放位置**，运维时无法确定改哪里。
- 死常量：`src/follow_actuator.py:20` `LOW_FORWARD_MPS`、`22` `FAST_MIN_VOLTAGE` 全仓无引用。
- 环境开关散落 5 个：`WHEELTEC_EDGE_MODEL`/`VIEW_ONLY`/`VIEWER_PORT`/`YOLO_WEIGHTS`/`FOLLOW_CONFIG`（`src/camera_viewer.py:25-27,57`、`src/follow_settings.py:55`），无 schema 文档。

**模块边界**：整体设计得好。`follow_motion.py`/`battery_state.py`/`edge_tracking.py`/`edge_detector.py`/`lidar_geometry.py` 均无 HTTP/ROS 依赖，纯函数式可测；`follow_session.py:1` 声明"Only the control loop calls update; GET is read-only"并用测试锁死（`tests/test_follow.py:307-326` 断言 20 次 GET 后 snapshot 不变）。分层：传感器桥(:18767) → 感知 → 会话 → 运动 → 执行器桥(:18768)，与 `docs/architecture.md:3-14` 的 mermaid 图一致。

**全局状态：`camera_viewer.py` 是最大的结构性问题。** 模块级可变全局 20+：`state`(46)、`session`(48)、`follow = session.data`(49)、`performance/control_performance`(50-51)、`request_cache`(52)、`shutting_down`(53)、`observations`(54)、`observation_time`(55)、`observation_sensors`(56)、`raw`(59)、`sequence`(60)、`capture_epoch`(61)、`control_link`(77)、`last_page_heartbeat`(79)、`page_heartbeats`(80)、`page_actions`(81)、`start_generation`(82)、`_control_http`(75)。9 个 `global` 语句（`102,187,195,241,360,381,446,486`）。后果：

- **测试必须 monkeypatch 模块全局**才能跑（`tests/test_follow.py:294-304` 要 `sys.modules.pop('camera_viewer')` 再 import，替换 `viewer.actuator_request`、`viewer.control_link`）。
- **同名变量遮蔽**：`session` 既是模块级 `FollowSession`（`src/camera_viewer.py:48`），又在 `worker` 里被重赋值为 generation 整数/元组（`src/camera_viewer.py:272`）。虽因作用域规则不冲突（`worker` 内无 `global session`），但同一文件里 `session.update(...)`（`146`）与 `session = generation...`（`272`）语义完全不同，是可读性地雷。
- **私有方法被外部调用**：`src/camera_viewer.py:150` 直接调 `session._wait(...)`，破坏封装。
- `stop_follow(reason, distance=None)`（`src/camera_viewer.py:182`）的 `distance` 参数从未使用。

**重复代码**：

- 两套控制器（见第 3 节）：`src/follow_control.py:292-317,320-356` vs `src/follow_motion.py`。
- 两套 HTTP keep-alive 客户端：`src/camera_viewer.py:101-118` 与 `src/follow_control.py:94-117`，逻辑几乎相同（懒建连、异常则 close+置 None 再 raise）。
- 两套彩深配对：`src/follow_sensors.py:34-58` 与 `157-169`。
- 端口号散落：`18767` 出现在 `src/follow_control.py:20,104`、`src/follow_sensors.py:271`、`ops/boot_health.py:28,48`、`ops/run_short_trial.py:35`、`ops/deploy-on-robot.sh`；`18768` 在 `src/camera_viewer.py:64`、`src/follow_actuator.py:214`、`ops/*`；`18769` 只在 ops 里，而 viewer 默认端口是 `8769`（`src/camera_viewer.py:27`），车端靠 unit 文件覆盖为 18769（`ops/wheeltec-follow-edge.service:10`）——**默认值与部署值不一致，且无共享常量模块**。
- 速度上限三处重复定义：`src/follow_control.py:25-26`（0.10/0.25）、`src/follow_actuator.py:21`（0.25）、`src/follow_settings.py:22-23`（max/fallback）。执行器钳制 0.25 而活动配置允许 0.25，勉强自洽，但靠 `load_settings` 的 `<= .25` 断言维持（`src/follow_settings.py:74,76,95`），任一处改动都可能悄悄破掉不变式。

**可测试性**：核心决策层很好（`tests/test_follow.py:1-236` 无需 mock 即可测会话/运动/电池/几何/跟踪器，`FakeDetector`/`NoReid` 注入见 `tests/test_follow.py:238-249`）。**编排层与 IO 层覆盖仍不完整**：`capture()`、`worker()`、`control_loop()` 三个循环体没有主机单元测试直接执行（测试 fixture 只 import 模块、不调 `main`，见 `tests/test_follow.py:294-304`）。ROS 桥已补上轻量 stub 测试：`tests/test_sensor_tf.py` 覆盖 TF 直接/反向/多跳、动态/静态、无效直连或反向边回退和精确 180° 旋转；`tests/test_actuator.py` 已覆盖无请求体 `/arm`、`/stop`、keep-alive 非空请求体消费和 `/health` 的 `503/200` 语义。`aligned_pair()`、`project_scan()` 与 `Sensors.frame()` 尚未形成主机单元测试；彩色帧编码、扫描新鲜度和雷达投影由 Docker ROS 2 Humble SIL 运行时验证。2026-11-26 完整回归为 `50 passed, 2 skipped`，跳过项均因缺少可选 MuJoCo 依赖；`docs/IMPLEMENTATION.md` 的 24 项是 2026-11-12 历史记录，不代表当前数量。

**依赖管理**：分层 requirements 设计合理（`requirements/base.txt`/`edge.txt`/`desktop.txt`/`dev.txt`/`export.txt`，edge 精确 pin `onnxruntime==1.23.2`）。但：

- **无 lockfile**（无 uv.lock/poetry.lock/Pipfile.lock/constraints.txt），base 用宽区间（`numpy>=1.24.2,<2`、`Flask>=2.2.2,<4`）。本地 `.venv` 实际装的是 Flask 3.1.3 / numpy 1.26.4 / pillow 11.3.0，而 `docs/IMPLEMENTATION.md:66` 记录车端为 Flask 2.2.2 / NumPy 1.24.2 / Pillow 9.4.0 —— **开发环境与生产环境依赖不同版本，测试通过的组合不是车上跑的组合**。
- ROS 依赖完全不声明版本（`requirements/base.txt:2` 注释"robot package-manager versions remain documented separately"），astra_camera / lslidar_driver / turn_on_wheeltec_robot 只以镜像名 `ros2:wheeltec_V1.1` 隐含。
- `pyproject.toml` 故意不做 pip 包（`pyproject.toml:1` 注释），无 `[project] dependencies`，构建产物靠 `ops/build_release.py` 白名单（`ops/build_release.py:13-31`）——白名单是手写的，新增模块易漏（`docs/IMPLEMENTATION.md:52` 特别警告不要把整仓覆盖过去）。
- CI 矩阵 ubuntu/windows × py3.11/3.12（`.github/workflows/ci.yml`），但车端是 Python 3.11.2 in Docker，**CI 不覆盖 3.10/容器环境，也不跑 rclpy 相关测试**。

### 工业做法 / 差距

需要：类型注解全覆盖 + mypy strict 进 CI；ruff/black 强制；统一 logging（每模块 `getLogger(__name__)`、JSON formatter、级别规范、rotation）；常量收进 Settings 或 `constants.py` 单一真源；`camera_viewer.py` 拆为 App/Orchestrator 类，依赖注入替代模块全局；pytest-cov 门槛 + 对三个循环的集成测试（用 fake sensor bridge）；lockfile + 与车端一致的 CI 环境（容器化 CI）；`ops/` 脚本纳入静态检查（CI 已 `compileall ops`，但没有 lint）。

---

## 7. 接口与可观测性

### 现状

**HTTP 接口设计**：REST-ish、动词正确、错误语义清楚，明显高于随手之作。

- 路由：`GET /`（`src/camera_viewer.py:350`）、`POST /detection`（`358`）、`POST /follow`（`379`）、`POST /shutdown`（`444`）、`POST /heartbeat`（`484`）、`GET /healthz`（`502`）、`GET /status`（`514`）、`GET /video`（`549`）、`GET /preview.jpg`（`563`）。
- 幂等与重放防护做到位：`/follow` 用 `(owner, seq)` 键 + `request_cache`（`src/camera_viewer.py:411-417`、`438-440`，LRU 截断 256），旧快照在新 seq 下被拒（`418-419`），启动快照带 `START_SESSION + now + start_generation` 三元组且要求时间在 2.5 s 窗口内（`420-424`）。有专门测试验证"停止后旧请求不能复活运动"（`tests/test_follow.py:307-326`）。
- 状态码语义：400 参数、403 来源、409 状态冲突、503 依赖未就绪（`src/camera_viewer.py:403`、`428`、`435`）。
- 输入校验严格（`type(x) is bool`、`len(owner)==36`、target 范围）。
- 弱点：① **无 API 版本前缀**（`/v1/`），`/status` 返回巨型扁平 JSON（`src/camera_viewer.py:528-546`，十余个顶层键、深层嵌套），字段增删即破坏前端；② 无 OpenAPI/schema；③ 无鉴权（第 4 节已述）；④ `/video` 的 multipart 流在生成器里 `time.sleep(0.04)` 忙轮询全局 `state['frame']`（`src/camera_viewer.py:551-559`），且每条连接独占一个 Werkzeug 线程直到客户端断开——**车端页面用 snapshots 模式绕过了它**（`web/templates/follow.html:197`、`506`），所以 `/video` 在车端实质是死接口（`src/camera_viewer.py:549`）。

**状态机**：实现得不错，但**没有显式的转换表**。

- 状态集合：`idle / confirming / following / arrived / waiting_person / waiting_corridor / fault_paused / manual_stop`（产生点：`src/follow_session.py:39,45,81,84,87,103,109,115,121,129`；前端标签映射 `web/templates/follow.html:390`）。
- 事件日志环形缓冲 100、对外暴露最后 10 条（`src/follow_session.py:15`、`src/camera_viewer.py:546`）——可回溯，好。
- 缺口：转换关系靠 if/return 隐式表达，无法静态验证"是否存在非法转换"或"某状态是否可达"；无单元测试覆盖全部转换对；`_state()` 每次重置所有速度字段（`src/follow_session.py:49-53`）副作用大，容易与 `data.update` 顺序耦合出错。

**指标 / trace / rosbag**：

- 有**原始遥测**：`/status` 暴露 fps、P95 延迟及其 basis、四个分段延迟埋点、detect_ms、传感器年龄、计划/下发/里程计三列速度（`src/camera_viewer.py:537-543`；前端分别渲染 `web/templates/follow.html:388-397`）。`ops/record_validation.py` 以 0.5 Hz 落 JSONL 并汇总 RSS/PID/error count（`ops/record_validation.py:22-45`）——**这是自制的最小可行遥测系统，思路正确**。
- **没有 metrics 端点**（无 Prometheus/OTLP；grep `prometheus|metrics|opentelemetry` 只命中 CSS 类名 `web/templates/follow.html:128`）、**没有 trace/request id**、**没有 rosbag 记录**（grep `rosbag|rosbag2` 零命中）。`validation/*.jsonl` 被 gitignore（`.gitignore:11`），只留 summary。
- 前端轮询放大观测噪声：`setInterval(refresh, 500)` + `setInterval(heartbeat, 400)` + `setInterval(refreshPreview, 260)`（`web/templates/follow.html:506,514,515`），即每页 ~7 req/s，且 `/status` 在 `lock` 内构造大对象（`src/camera_viewer.py:516-546`）——**Web 负载与控制环争用同一把锁**，观测行为本身影响被测系统。
- 故障定位能力：`/healthz` 提供 image/inference/control 三个 age（`src/camera_viewer.py:507-510`）、`boot_health.py` 据此做逐项退避重启（`ops/boot_health.py:49-98`）、`ops/diagnose_tracking.py` 提供逐帧感知透视。这三件套让常见故障可查。ROS 桥已记录就绪事件，传感器桥也会记录 HTTP 请求失败，但 TF 缺失或无效仍只表现为无投影，执行器故障日志也偏少，是最难定位的一类盲区。
**接口与可观测性三项具体缺陷**：① UI"计划前进"绑的是下发值而非计划值，破坏了文档承诺的"计划 → 下发 → 里程计"三段回放诊断法（`web/templates/follow.html:388` 读 `follow.forward_mps`，而真计划在 `src/follow_session.py:132` 的 `planned_forward_mps`，前端从未引用）；② 试跑前置安全检查未接线——后端 `trial_block_reason` 恒为 `None`（`src/camera_viewer.py:534`），前端却围绕它做按钮禁用与提示（`web/templates/follow.html:406,423-425`），本该填它的 `src/follow_control.py:280` `trial_depth_block()` 是死代码，**试跑的场地确认实质不生效**；③ ROS 桥已有就绪日志，传感器桥也会记录 HTTP 请求失败，但访问日志被静音、TF 缺失仍无明确诊断，执行器故障日志不足。

### 工业做法 / 差距

产品级需要：版本化 API + schema 契约 + 认证；显式 FSM（转换表 + 覆盖率测试）；Prometheus/OTel 指标（histogram 而非手工 P95 分位）；rosbag2 自动黑匣子（异常触发前后 30 s 录制）；集中日志聚合；观测通道与控制通道资源隔离（独立线程池/进程，不给 `/status` 加控制锁）。

---

## 8. ROS2 使用方式：HTTP 桥 + 非 ROS 节点的利弊

### 现状（客观描述）

**重要更正：仓库当前分支 `feat/ros2-package` 已完成现有传感器/执行器的正式包化，并通过 ROS 2 Humble SIL 验收。** Flask、感知、会话和运动规划仍通过 HTTP 使用两个桥，这是当前刻意保留的兼容边界，不是 `runpy` shim。实际状态：

- **`follow_bridges` 是正式 `ament_python` 包**：节点实现位于 `ros2_ws/src/follow_bridges/follow_bridges/`，`actuator_bridge.py` 与 `sensor_bridge.py` 通过包相对 import 暴露 `main`，不使用 `runpy`。配置资源安装到包 share 目录，发布归档通过 `src/follow_bridges/` 携带同一份实现。
- **`follow_bringup` 与 `follow_sim` 已组成可运行 workspace**：launch 支持参数文件、命名空间以及传感器/执行器开关；SIL 启动 `follow_sim` 与 `actuator_bridge`。仿真订阅 `/cmd_vel`，发布 `/odom`、`/scan`、`/PowerVoltage` 和 `/tf`，并在检测到其他 `/PowerVoltage` 发布者时拒绝启动。
- **SIL 验收覆盖真实 ROS/DDS 边界**：三包构建、节点/话题发现、`/health` 从 `503` 到 `200`、无请求体 `/arm` 与 `/stop`、`0.05 m/s` 低速运动、执行器与仿真看门狗、传感器 HTTP、TF 雷达投影和干净关停均通过；详见 `docs/ros2-packages.md`。这不是实车验收。
- **自定义接口骨架仍在 `reference/ros2_ws/src/`**：`follow_msgs`、`follow_bringup`、`follow_motion`、`follow_session` 是可编译设计参考，不属于活动 workspace。`TrackArray.msg` 与 `FollowPerson.action` 尚未接入运行系统。
- **发布映射已包含包化桥实现**：`ops/build_release.py` 把 `src/follow_bridges/*.py` 映射到归档的 `follow_bridges/`；活动 `ros2_ws/` 与 `reference/` 仍不直接进入车端发布归档。

两个桥保留 ROS 节点身份：`Sensors(Node)` 订阅彩色/深度图像、相机信息、扫描和 TF，`Actuator(Node)` 发布 `/cmd_vel` 并订阅电压与里程计。应用层其余模块仍通过 localhost HTTP 通信，因此本次包化没有改变跟随算法和人工停止路径。传感器桥直接消费 `tf2_msgs/TFMessage`，执行器继续以独立进程运行看门狗和停止突发。
- 值得肯定的细节：`rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`（`src/follow_sensors.py:269`）避免 ROS 抢信号处理器；执行器自己接管 SIGTERM/SIGINT 并在退出时**继续主动发零速 1.5 s**（`src/follow_actuator.py:217-232`）；`TCP_NODELAY`（`src/follow_sensors.py:227`、`src/follow_actuator.py:163`）；`allow_nan=False` 防 NaN 出 JSON（`src/follow_sensors.py:251`、`src/follow_actuator.py:166`）。

### 利（这套做法的真实价值）

1. **可在无 ROS 的机器上开发与测试。** 这是最大收益且已被兑现：2026-11-26 主机回归为 `50 passed, 2 skipped`，两项跳过均因缺少可选 MuJoCo；ROS 消息与 OpenCV 用轻量 stub 隔离。工业界也常用类似手法（protobuf/gRPC 边界 + 硬件抽象层）。
2. **进程边界带来真实的容错与可重启性。** 桥与应用分离后，`ops/boot_health.py` 可以单独重启 astra/lidar/base/桥（`ops/boot_health.py:66-96`）而不打断跟随会话；这正是 ROS 节点"崩了不连坐"的收益，作者在学习笔记里想要的性质其实已经部分拿到了。
3. **调试成本极低。** `curl http://127.0.0.1:18767/frame`、`/health`、`/scan` 就能定位问题层，不需要 `ros2 topic echo` + QoS 排障。`docs/ros2-learning-notes-01.md:134` 记录的"QoS 不兼容时静默收不到数据"这一痛点被 HTTP 天然规避。
4. **序列化边界强制显式契约**。所有跨层数据必须是可 JSON 化的显式字段（`src/follow_control.py:141-168` 逐字段校验），反而避免了隐式共享复杂对象的耦合。
5. **只读姿态明确**：`src/follow_sensors.py:1` docstring "Read-only ROS sensor bridge"、`ops/record_validation.py:1` "never sends a motion/start/shutdown request"，传感器侧绝不写 ROS，降低了污染原厂栈的风险。
6. **迁移路径已验证包化边界**：正式 ROS 包保留原有 HTTP 契约，SIL 用真实 DDS 验证话题名、类型、健康状态与停车语义。说明应用层可以在不重写跟随算法的前提下接入 ROS；后续若引入原生消息和 action，仍需单独验证行为等价性。

### 弊（工程代价，具体到代码）

1. **每一帧都过一次 base64 + JSON + HTTP**。`src/follow_sensors.py:190,203,216` 把 JPEG/PNG base64 进 JSON 字符串，`src/follow_control.py:108` 读最多 512 KB，`127-135` 再解码回 PIL/numpy。**编解码开销与内存拷贝直接吃掉了 Pi5 的 CPU 预算**，这是 FPS 上不去的原因之一，也是延迟测量被污染的原因（第 2 节）。ROS 侧本来 zero-copy/共享内存（`rmw_iceoryx`/loaned messages）能省掉的开销全回来了。
2. **请求-响应模型不适合流式数据**。`capture()` 必须轮询并靠 `X-After-Frame` 头做"只要新帧"（`src/follow_control.py:99-100`、`src/follow_sensors.py:183-184`），本质是在 HTTP 上重造订阅语义。轮询周期 50 ms（`src/camera_viewer.py:67`）给链路加了平均 25 ms、最差 50 ms 的量化延迟。
3. **QoS/可靠性语义丢失**。桥用 `qos_profile_sensor_data`（best-effort，`src/follow_sensors.py:93`）收数据，但下游变成 TCP 可靠 + 应用层"重复包丢弃"（`src/follow_sensors.py:97`），既没有 deadline policy 也没有 liveliness 回调——**"话题死了"这件事只能靠 age 阈值猜**（`src/follow_control.py:226-229`、`src/follow_motion.py:40-46`）。
4. **TF/时间语义降级**。桥内查 latest transform（`src/follow_sensors.py:134`）而非按 color stamp 查，且传给应用层的只有标量 age，应用层无法做时间插值或不确定度传播。原生 ROS2 节点可以直接持有 `MessageFilter` + `ApproximateTimeSynchronizer` 保留完整 `header.stamp`。
5. **无法接入 ROS 生态（正在被逐阶段消解）**。当前 `src/` 应用层用不了 nav2、SMAC/MPPI controller、ros2_control，也没有 `ros2 action`（跟随任务本该是一个 Action：有 goal/feedback/cancel，`docs/ros2-learning-notes-01.md:76-78` 作者自己分析到了），也失去了 `ros2 bag` 黑匣子能力。但 `reference/ros2_ws/src/follow_msgs` 已定义 `FollowPerson.action` 并编译通过，`follow_bringup` 已有 launch + `config/follow_params.yaml`，对应路线图的 B/C/D/E/F 阶段——**这一项不再是"缺失"，而是"排期中"**。
6. **拓扑复杂度上升而非下降**。现在一台 Pi5 上并存：Docker 容器 + 原厂 3 个 launch + 2 个桥进程 + 2 个 HTTP 端口 + Flask 主进程 4 线程 + 外部 systemd 监督 + Windows 侧 SSH 隧道（`ops/watch-follow.sh:78`）。**排障要跨 5 个层次**，比原生 6 个 ROS 节点更难；桥日志已有就绪与部分错误信息，但仍缺少统一诊断。迁移完成后此项会收敛（launch 取代 pgrep 逻辑、话题取代端口）。
7. **配置一致性靠人维护**。`follow_settings.py` 需在容器内的 `/tmp` 副本保持一致，靠部署脚本手工 `docker cp` 七个文件（`ops/robot-follow-boot.sh`），`docs/IMPLEMENTATION.md:52` 特意警告"不要只复制个别 Python 文件"——**这类警告本身就是架构缺陷的症状**。阶段 B（ROS Parameters 取代 `config/follow_settings.json`）正是它的根治方案；注意 `reference/ros2_ws/src/follow_bringup/config/follow_params.yaml` 目前与 `config/follow_settings.json` 是两份平行真源，切换期间需防止二者漂移。
8. **骨架的话题契约与真车不一致，必须在阶段 F 之前修正**（新发现）。`reference/ros2_ws/src/follow_motion/follow_motion/motion_node.py:15,48` 订阅 `sensor_msgs/BatteryState` 的 `battery` 并读 `msg.voltage`（`motion_node.py:89`），而真车实际发布的是 `std_msgs/Float32` 的 `/PowerVoltage`（`src/follow_actuator.py:44`，取 `message.data`）；真车还订 `/odom`（`src/follow_actuator.py:45`）。既然 S1 已证明"话题名严格对齐"就是 sim-to-real 的全部机制，那么骨架里这个类型不匹配会让参考实现连上仿真时看似正常、连上实车时静默失效。另外 `motion_node.py:140` 用硬编码 `voltage < 11.4` 做低压判断，与现役 `SETTINGS.battery_slow_v = 10.8` / `battery_stop_v = 10.5`（`src/follow_settings.py:38-39`）语义不同，且丢掉了中位数滤波与 3 s 锁定语义（`src/battery_state.py:25-30`）——迁移时这两处都要显式对齐，不能"顺手简化"。

### 判断

**作为"从既有 Wheeltec 原厂 ROS 栈里安全地长出一个新应用"的过渡架构，这个取舍是合理的；而且它已经不是终态——`feat/ros2-package` 上的 strangler-fig 迁移已经启动并逐段验证。** 我的结论相应调整为三层：

1. **该保留、不要"主流化"掉的两项资产**（与项目既定决策一致，我独立读码后同样认为它们是加分项而非缺陷）：
   - **执行器作为独立进程 + 自带看门狗**（`src/follow_actuator.py:24-25,46,116-131`）。工业上这恰好是"保护通道与被保护功能分离"的雏形，比把一切塞进同一个 DDS executor 更接近安全架构。DDS 化之后也必须保持它是**另一个进程**。
   - **感知逻辑可在无 ROS 环境下单元测试**（本机 `.venv` 无 rclpy，`50 passed, 2 skipped`；ROS 消息与 OpenCV 用轻量 stub 隔离）。这是真实工程收益，不能被"必须用 ros2 test/launch_testing"的形式主义换掉。
   - 附带一条：**硬件缺席时必须拒绝运动而不是假装可用**。当前 `/health` 在无底盘时报 `available_forward_mps: 0.0`（依据 `src/follow_actuator.py:106-107,146`），这是正确行为，不是待修的 bug。
2. **HTTP 桥本身确实不适合作为终态**，理由不变且可量化：每帧 base64+JSON 的 CPU/拷贝成本（第 1、2 节）、请求-响应模型重造订阅语义（`X-After-Frame`，`src/follow_control.py:99-100`）、QoS/deadline/liveliness 语义丢失、TF 只能传标量 age。这些是结构性损失，参数调不出来。
3. **建议路径（不改代码，仅评估）**：按既定 B→F 推进即可，无需另起炉灶。具体优先级我会这样排：
   - **先修第 8 弊的契约不一致**（BatteryState vs Float32、硬编码 11.4 V），否则阶段 E/F 会把一个静默失效的接口固化下来。
   - **B（Parameters）**：顺带解决第 6 节的"配置不可热更新"和第 7 节的双真源漂移风险；注意保留 `load_settings()` 的范围校验语义（`src/follow_settings.py:54-99`），ROS Parameters 的 range 检查覆盖不到跨字段不变式（如 `stop_m < resume_m`、`fallback ≤ max ≤ .25`）。
   - **C（follow_bringup launch）** 取代 `ops/robot-follow-boot.sh` 的 pgrep 逻辑 —— 直接消除第 6 弊与"docker cp 七文件"的一致性风险。
   - **D（rosbag2 + launch_testing）** 补第 7 节的黑匣子缺口，同时给三个未测循环（`capture`/`worker`/`control_loop`）提供集成测试载体。
   - **E/F（FollowPerson action + TrackArray.msg）** 才动控制行为，必须留实车验收窗口。
   - 编排层 `camera_viewer.py` 的模块级全局状态（第 6 节）建议在同一次重构里改成节点类 + 依赖注入，否则它会成为迁移后最难测的一块。
   - 实时性（SCHED_FIFO / 绑核 / 绝对周期）与硬件急停回路（第 1、4 节）**不属于 ROS 化能解决的问题**，需要单独立项；前者是调度策略，后者是电气改造。

---

## 附：本次调研发现的具体缺陷清单（可直接开工）

| # | 严重度 | 问题 | 证据 |
| --- | --- | --- | --- |
| 1 | 高 | 无硬件急停回路；安全功能与被保护功能同故障域 | 全仓无 e-stop 实现；`src/follow_actuator.py:24-25,116-131` |
| 2 | 高 | `motion_lock` 临界区内含 0.22 s 网络 I/O，可阻塞人工停止 | `src/camera_viewer.py:142-161`、`65` |
| 3 | 高 | `/shutdown` 凭 `/status` 明文回传的 token 即可触发 | `src/camera_viewer.py:448`、`546`；`web/templates/follow.html:499` |
| 4 | 高 | 遗留的第二套控制器是死代码，参数语义已与现役分叉 | `src/follow_control.py:292-317,320-356` |
| 5 | 中 | UI"计划前进"显示的是下发值，破坏三段回放诊断 | `web/templates/follow.html:388` vs `src/follow_session.py:132` |
| 6 | 中 | 试跑前置深度安全检查未接线，`trial_block_reason` 恒 None | `src/camera_viewer.py:534`；`src/follow_control.py:280` |
| 7 | 中 | ROS 桥日志仍不完整；TF 缺失或无效只表现为无投影 | `src/follow_bridges/follow_sensors.py` 的 `projected_lidar()`/`transform_between()`；`src/follow_bridges/follow_actuator.py` |
| 8 | 中 | Re-ID worker 线程异常后永久失效且不重启 | `src/edge_reid.py:72-73` |
| 9 | 中 | `camera_to_front_m` 在 6 处手工相减；`calibration.json` 从未被代码读取（溯源与实际参数无代码绑定） | `src/lidar_geometry.py:13,21`；`src/follow_control.py:148,161`；`src/follow_motion.py:47,56`；`src/camera_viewer.py:521`；`config/calibration.json` 仅被 `ops/build_release.py:22` 打包 |
| 10 | 中 | 彩深配对两套实现；HTTP keep-alive 客户端两套实现 | `src/follow_sensors.py:34-58` vs `157-169`；`src/camera_viewer.py:101-118` vs `src/follow_control.py:94-117` |
| 11 | 中 | 开发/生产依赖版本不同且无 lockfile；CI 不覆盖容器环境 | `requirements/base.txt` vs `docs/IMPLEMENTATION.md:66`；`.github/workflows/ci.yml` |
| 12 | 中 | `capture/worker/control_loop` 三个循环仍无主机单元测试；`Sensors.frame()` 编码与投影只有 ROS SIL 运行时覆盖 | `tests/` 无三个循环的直接调用；`tests/test_sensor_tf.py` 覆盖 TF 图变换，传感器帧路径见 `docs/ros2-packages.md` |
| 13 | 低 | 无返回类型注解（全仓 0 处）；CI 仅 `compileall` | `grep "def .*->" src/*.py` = 0；`.github/workflows/ci.yml` |
| 14 | 低 | 死常量/死参数/死接口 | `src/follow_actuator.py:20,22`；`src/camera_viewer.py:182`；`src/camera_viewer.py:549`；`src/camera_viewer.py:79,486,498` |
| 15 | 低 | `Nice=5` 给控制进程降权；无绑核/实时策略 | `ops/wheeltec-follow-edge.service:22` |
| 16 | 低 | `worker` 内 `selected` 仅在 `enabled` 分支赋值，依赖上方 `continue` 跳出（脆弱控制流） | `src/camera_viewer.py:260-262,310-314,325,335` |
| 17 | 低 | `session` 名称在模块级与 `worker` 内语义冲突；`camera_viewer` 调私有 `session._wait` | `src/camera_viewer.py:48,272`；`150` |
| 18 | **高（迁移期）** | 骨架参考实现的电池契约与真车不符：类型 `BatteryState` vs `Float32`、话题 `battery` vs `/PowerVoltage`、阈值硬编码 11.4 V 且丢失中位数滤波与 3 s 锁定语义。**注意：项目自身已记录该问题并标注"fix before Stage F"，此处为独立复核确认，非新发现** | `reference/ros2_ws/src/follow_motion/follow_motion/motion_node.py:15,48,89,140` vs `src/follow_actuator.py:44,49-53`、`src/battery_state.py:25-30`、`src/follow_settings.py:38-40` |
| 19 | 中（迁移期） | 阶段 B 切换期间存在两份参数真源（`config/follow_settings.json` 与 `follow_params.yaml`），且 ROS Parameters 的 range 检查覆盖不到跨字段不变式 | `reference/ros2_ws/src/follow_bringup/config/follow_params.yaml` vs `config/follow_settings.json`；校验逻辑 `src/follow_settings.py:72-98` |
