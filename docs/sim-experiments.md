# 批量跟随仿真实验

目标：自动运行场景、按仿真真值评分、保存原始记录，复现某次失败。
第一版是教学控制基线；不是已经标定的 C30D 数字孪生，也不代表真机验收。
现有交互教学脚本、实车控制器、车端配置和发布映射不变。

## 安装与启动

使用独立环境安装可选依赖，不需要 ROS、相机硬件或小车连接：

~~~console
python -m venv .venv-sim
.venv-sim/bin/python -m pip install -r requirements/sim.txt
.venv-sim/bin/python sim/run_experiments.py
~~~

Windows 使用 .venv-sim\Scripts\python。本机已有 /Users/mac/mujoco-sim：

~~~console
~/mujoco-sim/bin/python sim/run_experiments.py --output sim/results/baseline
~~~

默认 seed=42，五类场景各6次，总计30次；每次24秒仿真时间。无窗口时不按真实时间等待。
输出目录必须不存在，避免覆盖已有实验。默认使用带时间戳的新目录。

| 场景前缀 | 行为 |
| --- | --- |
| static | 静止目标，不同初始距离与侧向位置 |
| straight | 直线匀速运动后停止 |
| turn | 沿圆弧移动后停止 |
| stop_start | 前进、暂停、再次前进、停止 |
| dropout | 直行期间暂时屏蔽观测，目标继续运动 |

人物初始 x 为1.0–1.8m、y 为−0.35–0.35m，运动速度为0.06–0.12m/s。
轨迹在总时长60%处结束，留出最终保持距离的评测窗口。
人物使用 mocap 脚本驱动，非真实步态与人体动力学；车体按物理引擎推进，不强行扶正。

## 输入模式与控制器

~~~console
~/mujoco-sim/bin/python sim/run_experiments.py --observation truth
~/mujoco-sim/bin/python sim/run_experiments.py --observation camera --repeats 1
~~~

- truth：使用仿真人物位置计算距离和方位，单独测试控制；不评估视觉检测。
- camera：渲染320×240 RGB，复用 follow_p_control.py 的橙色像素识别；单束测距无有效返回时使用像素面积估距。不是YOLO。
- 两种模式均用独立仿真真值评分。dropout为显式观测屏蔽，不冒充实体遮挡。
- 控制器采用教学脚本的P控制形式，观测缺失时改为左右轮零指令，实际轮指令均限幅。
- 轮指令单位为rad/s；不是实车的 linear.x / angular.z。真机控制器接入是后续独立步骤。
- 控制周期34ms，为2ms物理步长的17倍；不是精确30Hz。每个物理子步检查接触。

## 指标与成功条件

评分距离定义为 range_front site的XY投影到人物胶囊平面轮廓的欧氏距离。
它不是沿车头方向的单束雷达读数，不是ROS原厂车头净距，也不使用相机估距给相机自己评分。
同时记录车体到人物胶囊的三维最小物理距离，两个口径分列。

目标间距0.45m，死区±0.05m。成功条件是：

1. 完成整段轨迹，没有物理警告、数值异常或车体倾覆。
2. 无车体/车轮与人物的物理接触；地面与车轮正常接触不计入。
3. 观测缺失时轮速指令归零。
4. 最后2秒持续处于0.40–0.50m距离带、方位偏差≤8°。

这是最终收敛与无接触判定，**不单独保证移动阶段跟随质量**。
移动阶段应结合下列连续指标评估，不只看最终通过率：

- 全程平均/P95带外误差：max(0.40-gap, 0, gap-0.50)，包含起步。
- 全程平均绝对方位误差、同时在距离带且对齐的采样比例。
- 人物最终停下后，首次连续保持2秒合格的起点，相对停下时刻的延迟；未收敛为null。
- 停下后低于距离带下界的最大偏移（近侧超调）。
- 观测恢复后重新连续保持合格距离的耗时；这是恢复跟随耗时，不是首次检测耗时。
- 轮指令总变差/仿真时长，单位rad/s²；不是车身加速度。
- 接触事件在物理子步统计，从无接触到接触算一次；接触持续不重复计数。

报告的Wilson区间是固定小场景集的描述信息，不能外推到真实环境总体。
没有配对实车数据时，Sim-to-Real gap显示“待测”。

## 复现与参数对照

每次实验保存：

~~~text
manifest.json                场景、控制/评测参数、版本与代码校验
model.xml                    输入模型快照
summary.csv / summary.json   所有场景汇总
report.html                  离线报告与可展开曲线
static-01/
  scenario.json
  summary.json
  telemetry.csv              真值、观测、轮指令、实际v/ω与接触
~~~

~~~console
# 原参数和原场景重跑一例
~/mujoco-sim/bin/python sim/run_experiments.py --replay sim/results/baseline/manifest.json --case turn-01
# macOS可视化复现同一例；其他平台使用python --viewer
~/mujoco-sim/bin/mjpython sim/run_experiments.py --replay sim/results/baseline/manifest.json --case turn-01 --viewer
# 新实验只改转向增益，seed/时长/场景数量保持一致
~/mujoco-sim/bin/python sim/run_experiments.py --seed 42 --turn-gain 3 --output sim/results/turn-gain-3
~~~

复现校验代码、模型和MuJoCo版本；有差异时直接报错，不静默换参数。
保留原代码版本与完整输出目录；跨硬件不承诺逐位一致。
窗口提前关闭也保留记录，但该例标记为未完成。报告无需服务器即可双击打开。
结果目录已加入Git忽略；需要共享结论时在文档中记录条件与摘要，保留原始实验包另行归档。

## 工程上怎样权衡真实性

按验证对象分层投入。测试控制逻辑时可用真值观测；测试完整软件时渲染RGB，
运行与真机相同的检测/跟踪程序并模拟雷达；最后用硬件测量确认模型预测是否成立。
仿真器提供图像，YOLO负责识别图像，两者可以直接串联。

Isaac Sim官方提供将虚拟相机RGB/深度发布到ROS 2的流程：
[相机数据发布](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/advanced_sensors/tutorial_ros2_camera_publishing.html)。
视觉保真度不足时，可以改变人物外观、光照、纹理、相机位置等：
[域随机化](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-sim/latest/synthetic-data-generation-for-perception-model-training-in-isaac-sim/03-domain-randomization-with-replicator.html)。
随机化范围应来自真实分布或实测不确定性，并在未用于调参的真机条件下验证。

当前主要差异：教学车几何与轮型未核对；执行器/摩擦未标定；单束测距替代二维雷达；
橙色像素替代YOLO/Re-ID；没有车端网络、推理与执行器延迟链路。
在这个卡通人物模型上跑YOLO也不等于感知真实性已验证。

### 评测gap的顺序

| 层次 | 实测与仿真的对应实验 | 记录与比较 |
| --- | --- | --- |
| 几何 | 外廓、轮径轮距、相机/雷达外参 | 尺寸/角度偏差；轮子最外缘与车壳分开 |
| 开环响应 | 相同v/ω指令时序，不同速度和电压 | 速度曲线RMSE、稳态误差、起步延迟、上升时间、制动距离 |
| 感知 | 覆盖相似距离/角度/光照/遮挡，使用同一模型与阈值 | 漏检/误检、方位与测距误差、身份切换、连续丢失时长 |
| 时序 | 采集曝光/收到/推理完成/下发/反馈时间戳 | 帧龄、端到端延迟和抖动P50/P95；确认时钟基准 |
| 闭环任务 | 同控制器与任务协议，相近初始条件和人物轨迹 | 成功率百分点差、距离误差分布、最小间距、接管次数 |

先用一部分数据辨识/标定模型，留出其他速度、地面和轨迹验证，避免用同一批数据调参再报成绩。
实车里程计可用于速度反馈，但里程计自身误差要用外部位姿/距离测量检查；不能用轮速里程计证明没有打滑。
开放环回放测动力学；用真实录像回放测感知。录像回放不随新控制动作改变，不能当闭环迁移测试。

“足够真”需要先定用途和误差预算。比如在0.25m/s下，额外100ms时延约对应2.5cm位移；
若停车距离总误差预算为5cm，这一个因素就占了一半。这里是预算示例，非本车已达成指标。
分别记录成功率差的置信区间与连续误差分布，不用一个“真实性百分数”概括。
只有在留出的真机条件下满足预定预算、且仿真中的方案优劣在真机上也稳定复现，
才有依据说该模型对这一用途足够可信。

## 验证

~~~console
python -B -m pytest -q -p no:cacheprovider
# 在仿真环境额外安装pytest后运行物理集成与复现测试
~/mujoco-sim/bin/python -B -m pytest tests/test_sim_experiments.py -q -p no:cacheprovider
~~~

未安装MuJoCo的普通开发环境会跳过物理集成测试，其余场景与指标测试照常运行。
