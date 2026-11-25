"""Locate the unchanged application modules under ``src/`` so a bridge can be launched from them."""
import os
from pathlib import Path

ENV_VAR = 'WHEELTEC_FOLLOW_SRC'


def legacy_source_dir():
    override = os.environ.get(ENV_VAR)
    if override:
        return Path(override)
    for ancestor in Path(__file__).resolve().parents:
        if (ancestor / 'pyproject.toml').is_file():
            return ancestor / 'src'
    raise RuntimeError('未找到 src/，请设置 %s 指向应用模块目录' % ENV_VAR)
