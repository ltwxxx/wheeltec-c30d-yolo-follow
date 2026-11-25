"""Resolve application resources in a ROS install or source checkout."""
import os
from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:
    get_package_share_directory = None


def resource_root():
    override = os.environ.get("WHEELTEC_FOLLOW_RESOURCE_ROOT")
    if override:
        return Path(override)
    try:
        return Path(get_package_share_directory("follow_bridges")) / "resources"
    except Exception:
        module_dir = Path(__file__).resolve().parent
        if module_dir.name == "follow_bridges" and ((module_dir.parent / "pyproject.toml").is_file()
                                                   or (module_dir.parent.parent / "pyproject.toml").is_file()):
            return module_dir.parent.parent if (module_dir.parent / "pyproject.toml").is_file() else module_dir.parent
        if (module_dir.parent / "follow_settings.json").is_file():
            return module_dir.parent
        raise RuntimeError("无法定位 follow_bridges 资源目录")


def config_path(name):
    override = os.environ.get("WHEELTEC_FOLLOW_CONFIG")
    if override:
        return Path(override)
    root = resource_root()
    installed = root / "config" / name
    return installed if installed.is_file() else root / name
