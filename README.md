# Wheeltec C30D 人物跟随

基于树莓派 5、YOLO11n、二维激光雷达和 OSNet Re-ID 的室内人物跟随控制页。当前代码版本为 **2.0.0**，保留指定人物跟随和不认主跟随两种模式。

已实现功能、部署步骤和实车验收边界见 [IMPLEMENTATION.md](IMPLEMENTATION.md)。验证数据见 [validation/README.md](validation/README.md)。

## 运行结构

- 车端：`camera_viewer.py` 提供网页和控制循环，`follow_session.py` 管理会话，`follow_motion.py` 计算运动指令。
- 感知：`edge_detector.py`、`edge_tracking.py`、`edge_reid.py` 处理检测和身份关联；`follow_sensors.py`、`lidar_geometry.py` 提供雷达和相机信息。
- 底盘：`follow_actuator.py` 提供 ROS 指令接口、指令看门狗和电压保护；`battery_state.py` 估算电量。
- 电脑：Windows 通过 WSL `Ubuntu-F` 中的 SSH 隧道访问车端网页。页面关闭后，已启动的会话继续在车端运行；车端控制服务重启后回到待机。

| 入口 | 地址 |
| --- | --- |
| 电脑控制页 | `http://127.0.0.1:8769/` |
| 车端网页服务 | 车端回环地址 `127.0.0.1:18769` |
| 车端传感器接口 | 车端回环地址 `127.0.0.1:18767` |
| 车端底盘接口 | 车端回环地址 `127.0.0.1:18768` |

`127.0.0.1` 指访问设备自身。当前启动脚本使用 **8769**，旧的 8765 地址不再作为入口。

## 连接现有小车

在项目根目录的 PowerShell 中运行：

```powershell
& .\ops\start-follow-console.ps1
```

脚本需要已配置的 `Ubuntu-F`、SSH 主机校验记录和受限转发密钥。密钥、密码不包含在本仓库中。现有车端模式还依赖 WSL 用户目录中的 `~/.local/share/wheeltec-follow/use-robot-ui` 标记；这是连接现有部署的脚本，不是新电脑的一键安装器。脚本中的用户名、路径和局域网地址需要按实际环境调整。

如果车已开机但网页打不开，先检查 `Ubuntu-F` 和转发进程是否运行。小车重启会断开 SSH，只有重连脚本保持运行才能自动恢复。电脑上的计划任务需要另外配置；仓库中的脚本本身不会注册 Windows 计划任务。

## 配置与模型

- [follow_settings.json](follow_settings.json)：本车参数覆盖；默认参数及校验在 `follow_settings.py`。
- [calibration.json](calibration.json)：原厂安装几何、距离基准和未完成的标定项目。
- `wheeltec-yolo11n.pt`：已有 YOLO 权重。车端使用 `WHEELTEC_EDGE_MODEL` 指向的 ONNX 文件；当前部署器沿用车端已有的 `follow-yolo11n.onnx`，该文件未包含在本次仓库快照中。
- `models/osnet_x0_25.onnx`：Re-ID 推理模型；来源、校验值和导出误差见 `models/osnet_x0_25.json`，许可证见 `models/OSNET-LICENSE`。
- `ops/export_reid.py`：Re-ID 导出工具。原始训练权重可按元数据重新获取，不重复提交。

## 本地验证与部署

在已安装依赖的 Python 环境中执行：

```console
python -B -m pytest tests -q -p no:cacheprovider
python ops/build_release.py
```

本地测试使用传感器数据和 ROS 替身，不会驱动小车。2026-11-13 推送前复核：**24 项测试通过**。车端已验证的依赖版本和部署、恢复步骤见 [IMPLEMENTATION.md](IMPLEMENTATION.md)。`ops/deploy-on-robot.sh` 用于更新已经安装好的小车环境，并非空系统安装器。

本次上传保留当前实际配置：目标车头净距 0.30 m、恢复距离 0.40 m；制动实测尚未完成，`braking_verified=false`，最高速度因此暂限 0.10 m/s。现有静止性能和短时跟随记录不代表全部实车验收通过。`ops/run_short_trial.py` 会实际驱动车辆，须按实施文档的现场条件使用。

生成的部署包、运行缓存、原始日志和凭据不纳入 Git。原有 `stop_retest_*` 文件保留作历史参考，不作为新版执行器验收入口。
