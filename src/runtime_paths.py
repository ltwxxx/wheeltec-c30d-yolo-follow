"""Resolve resources in a source checkout and in the flat robot release."""
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent
SOURCE_CHECKOUT = MODULE_DIR.name == 'src' and (MODULE_DIR.parent / 'pyproject.toml').is_file()
RESOURCE_ROOT = MODULE_DIR.parent if SOURCE_CHECKOUT else MODULE_DIR
TEMPLATES_DIR = RESOURCE_ROOT / 'web' / 'templates' if SOURCE_CHECKOUT else MODULE_DIR / 'templates'


def config_path(name):
    return RESOURCE_ROOT / 'config' / name if SOURCE_CHECKOUT else MODULE_DIR / name
