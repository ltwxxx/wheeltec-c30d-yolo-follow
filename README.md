# Wheeltec C30D 人物跟随

基于树莓派 5、YOLO11n、二维激光雷达和 OSNet Re-ID 的室内人物跟随系统，提供指定人物跟随和不认主跟随两种模式。

**当前状态：2.0.0，实车验收进行中。** 目标车头净距 30 cm，制动实测未完成，实际配置最高暂限 0.10 m/s。详细限制见[实施记录](docs/IMPLEMENTATION.md)。

[控制页入口（待配置）](docs/control-page.md) · [架构说明](docs/architecture.md) · [开发指南](docs/development.md) · [部署与排障](docs/deployment.md) · [验证数据](validation/README.md) · [变更记录](CHANGELOG.md)

> 局域网控制地址暂未配置。计划使用 `http://<控制电脑局域网IP>:<端口>/`，实际 IP 和端口将在小车恢复连接后填写。当前链接为占位说明，不代表控制服务已经开放。

## 目录结构

```text
.
├── src/                 # 控制、感知、会话、ROS 桥接与资源路径
├── config/              # 跟随参数、几何标定与跟踪器配置
├── web/templates/       # 网页控制台
├── models/              # 推理模型、来源校验与第三方许可证
├── ops/                 # 连接、部署、开机服务、诊断及发布工具
├── requirements/        # 开发、车端、桌面推理与模型导出依赖
├── tests/               # 无需小车的自动化测试
├── validation/          # 已采集的验收摘要与脱敏数据
├── docs/                # 架构、开发、部署及实施记录
├── assets/samples/      # 历史相机样本
├── tools/legacy/        # 历史测试工具，非当前运行入口
└── .github/             # CI 与问题、PR 模板
```

仓库目录与车端安装目录由发布工具映射，车端现有的服务入口保持兼容。`src/` 是可直接运行的应用模块，不作为 pip 库发布。

## 运行结构

- 车端：`src/camera_viewer.py` 提供网页和控制循环，`src/follow_session.py` 管理会话，`src/follow_motion.py` 计算运动指令。
- 感知：`src/edge_*.py` 处理检测和身份关联；`src/follow_sensors.py`、`src/lidar_geometry.py` 提供雷达和相机信息。
- 底盘：`src/follow_actuator.py` 提供 ROS 指令接口、指令看门狗和电压保护；`src/battery_state.py` 估算电量。
- 电脑：Windows 通过 WSL `Ubuntu-F` 中的 SSH 隧道访问车端网页。页面关闭后，已启动的会话继续在车端运行；车端控制服务重启后回到待机。

| 入口 | 地址 |
| --- | --- |
| 局域网控制页（待配置） | 见[控制页入口](docs/control-page.md) |
| 电脑内部转发 | 本机回环地址 `127.0.0.1:8769` |
| 车端网页服务 | 车端回环地址 `127.0.0.1:18769` |
| 车端传感器接口 | 车端回环地址 `127.0.0.1:18767` |
| 车端底盘接口 | 车端回环地址 `127.0.0.1:18768` |

`127.0.0.1` 指访问设备自身，仅用于本机转发和诊断，手机不能用该地址连接电脑。局域网访问入口尚待配置。当前本机启动脚本使用 **8769**，旧的 8765 地址不再作为入口。

## 本机连接现有小车

在项目根目录的 PowerShell 中运行：

```powershell
& .\ops\start-follow-console.ps1
```

脚本需要已配置的 `Ubuntu-F`、SSH 主机校验记录和受限转发密钥。密钥、密码不包含在本仓库中。现有车端模式还依赖 WSL 用户目录中的 `~/.local/share/wheeltec-follow/use-robot-ui` 标记；这是连接现有部署的脚本，不是新电脑的一键安装器。脚本中的用户名、路径和局域网地址需要按实际环境调整。

如果车已开机但网页打不开，先检查 `Ubuntu-F` 和转发进程是否运行。小车重启会断开 SSH，只有重连脚本保持运行才能自动恢复。电脑上的计划任务需要另外配置；仓库中的脚本本身不会注册 Windows 计划任务。

## 配置与模型

- [config/follow_settings.json](config/follow_settings.json)：本车参数覆盖；默认参数及校验在 `src/follow_settings.py`。
- [config/calibration.json](config/calibration.json)：原厂安装几何、距离基准和未完成的标定项目。
- `models/wheeltec-yolo11n.pt`：已有 YOLO 权重。车端使用 `WHEELTEC_EDGE_MODEL` 指向的 ONNX 文件；当前部署器沿用车端已有的 `follow-yolo11n.onnx`，该文件未包含在本次仓库快照中。
- `models/osnet_x0_25.onnx`：Re-ID 推理模型；来源、校验值和导出误差见 `models/osnet_x0_25.json`，许可证见 `models/OSNET-LICENSE`。
- `ops/export_reid.py`：Re-ID 导出工具。原始训练权重可按元数据重新获取，不重复提交。

## 本地验证与部署

使用 Python 3.11 或 3.12，在独立虚拟环境中执行：

```console
python -m pip install -r requirements/dev.txt
python -B -m pytest tests -q -p no:cacheprovider
python ops/build_release.py
```

本地测试使用传感器数据和 ROS 替身，不会驱动小车。CI 在 Linux 和 Windows、Python 3.11 和 3.12 上运行测试，检查发布包校验和、资源加载及 Shell 语法。开发环境说明见[开发指南](docs/development.md)。

`ops/deploy-on-robot.sh` 用于更新已有车端环境，并非空系统安装器。现有静止性能和短时跟随记录不代表全部实车验收通过；`ops/run_short_trial.py` 会实际驱动车辆，使用条件见[部署文档](docs/deployment.md)。

生成的部署包、运行缓存、原始日志和凭据不纳入 Git。贡献前请阅读 [CONTRIBUTING.md](CONTRIBUTING.md)。第三方 OSNet 许可证仅覆盖相应第三方文件，不表示整个项目采用同一许可证。
