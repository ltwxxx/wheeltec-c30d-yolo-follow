# 本地开发

开发与 CI 使用 Python 3.11 / 3.12。推荐创建独立环境：

```console
python -m venv .venv
```

Windows PowerShell 激活命令为 `.\.venv\Scripts\Activate.ps1`，Linux / WSL 为 `source .venv/bin/activate`。然后执行：

```console
python -m pip install -r requirements/dev.txt
python -B -m pytest -q -p no:cacheprovider
python ops/build_release.py
```

`pyproject.toml` 将测试范围限制为 `tests/`，并把 `src/` 加入测试模块搜索路径，避免误收集需要连接 ROS 的运维脚本。发布测试会检查真实资源路径、模型校验值、归档校验和以及从独立目录加载解包后的页面。

## 依赖分组

| 文件 | 用途 |
| --- | --- |
| `requirements/base.txt` | 共享 Python 依赖 |
| `requirements/dev.txt` | 基础依赖和 pytest；不安装 ROS 或 PyTorch |
| `requirements/edge.txt` | ONNX 车端推理依赖 |
| `requirements/desktop.txt` | 可选的 WSL / 桌面 PyTorch 后端 |
| `requirements/export.txt` | 离线模型导出工具 |

依赖范围用于开发环境安装，尚不是锁定全部传递依赖的环境快照。车端已验证的包版本见[实施记录](IMPLEMENTATION.md)。ROS 2 Humble、消息包和 TF 来自车端已有 ROS 镜像，不通过这些 requirements 安装；不要直接用开发依赖覆盖正在运行的车端系统环境。

## 启动入口

- 访问已有小车：根目录运行 `ops/start-follow-console.ps1`。
- 源码方式运行控制页：安装相应推理后端，配置传感器、执行器连接后运行 `python src/camera_viewer.py`。
- 车端 ONNX 后端：设置 `WHEELTEC_EDGE_MODEL` 为实际模型路径；不设置时使用可选的桌面 PyTorch 后端。
- 覆盖配置：设置 `WHEELTEC_FOLLOW_CONFIG` 为配置 JSON 的绝对路径，重启进程后生效。

模型导出：在安装 `requirements/export.txt` 的独立环境中运行 `python ops/export_reid.py`。导出会写入 `models/`，更新后需核对元数据和许可证。

## CI

`.github/workflows/ci.yml` 对 push 和 pull request 执行 Linux / Windows、Python 3.11 / 3.12 测试矩阵，以及 Python 和 Bash 语法检查。CI 不连接小车、不使用 SSH 密钥，也不执行实车运动脚本。自动化结果与现场验收记录分别维护。
