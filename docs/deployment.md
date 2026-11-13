# 部署、连接与排障

局域网控制入口当前为[待配置占位页](control-page.md)，以下回环地址用于控制电脑本机连接与诊断，不是手机访问地址。

## 连接现有小车

在项目根目录的 PowerShell 运行 `& .\ops\start-follow-console.ps1`。控制页为 `http://127.0.0.1:8769/`，SSH 转发目的地为小车回环地址 `127.0.0.1:18769`。

当前脚本使用 `Ubuntu-F`、已校验的主机记录和受限转发密钥。车端页面模式需要 WSL 中的 `~/.local/share/wheeltec-follow/use-robot-ui` 标记。`ops/watch-follow.sh` 没有该标记时走兼容的本地推理路径；源码入口已调整为 `src/camera_viewer.py`，权重位于 `models/`。

连接脚本保留现有安装的主机、用户和路径设置，移到其他电脑前需要相应调整。密码和私钥不入库。

## 页面打不开

依次检查电脑入口、隧道和车端服务：

```powershell
wsl.exe --list --verbose
Get-NetTCPConnection -State Listen -LocalPort 8769 -ErrorAction SilentlyContinue
Invoke-RestMethod http://127.0.0.1:8769/healthz
```

若 `Ubuntu-F` 已停止或 8769 没有监听，重新运行连接脚本。小车重启会断开 SSH，重连进程必须持续运行才能恢复页面；车端开机不等于电脑端转发程序也会启动。Windows 计划任务由电脑单独配置，脚本不会自动注册任务。

若隧道存在但健康检查仍失败，在小车 SSH 会话中只读查看：

```console
systemctl --user status wheeltec-follow-edge.service
systemctl status wheeltec-follow-boot.service
journalctl --user -u wheeltec-follow-edge.service -n 50 --no-pager
curl --noproxy '*' http://127.0.0.1:18769/healthz
```

## 构建与更新

1. 从仓库根目录运行 `python -B -m pytest -q`。
2. 核对 `config/follow_settings.json`、`config/calibration.json`，运行 `python ops/build_release.py`。
3. 发布包输出到 `releases/wheeltec-2.0.0.tar.gz`；也可通过 `--output-dir` 指定其他输出目录。相同输入产生相同归档，内部 `SHA256SUMS` 覆盖发布文件。
4. 上传归档到车端，解包后运行其中的 `ops/deploy-on-robot.sh`，参数为归档绝对路径。该脚本要求已有运行目录、Docker 容器、ROS 环境和服务，不是首次安装器。
5. 部署器检查当前会话已停止，备份运行目录和服务，更新后回到待机；失败时尝试恢复备份。备份位置和历史版本见[实施记录](IMPLEMENTATION.md)。

车端 YOLO ONNX 文件仍由现有安装提供，本发布包不包含它。新版资源定位模块已加入 ROS 桥的部署清单；更新时使用完整归档，不要只复制个别 Python 文件。

## 实车验收

`ops/record_validation.py` 用于记录运行数据；`ops/run_short_trial.py` 会驱动车辆，必须有人在车旁、通道空旷且可及时停止。`tools/legacy/` 只保留历史工具，不能替代当前执行器验收。

当前仍需完成实际制动距离、30 cm 无接触停车、连续跟随、遮挡和多人测试。不要把目录整理、单元测试通过或 CI 通过当作实车验收完成。
