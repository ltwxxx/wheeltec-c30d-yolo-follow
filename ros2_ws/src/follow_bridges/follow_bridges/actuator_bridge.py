"""Launch the existing cmd_vel bridge without modifying or copying its source."""
import runpy
import sys

from .legacy_paths import legacy_source_dir


def main():
    source_dir = legacy_source_dir()
    if str(source_dir) not in sys.path:
        sys.path.insert(0, str(source_dir))
    runpy.run_path(str(source_dir / 'follow_actuator.py'), run_name='__main__')


if __name__ == '__main__':
    main()
