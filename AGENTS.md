# Repository Guidelines

## Project Structure

- `src/` contains the runnable Python application modules. Keep the application modules flat; `src/follow_bridges/` carries the ROS node implementations used by both the active workspace and the established flat robot release. ROS 2 nodes live in `ros2_ws/src/follow_bridges/` and must not use `runpy` to execute `src/`.
- `ros2_ws/src/` contains active ROS 2 bridge, bringup, and lightweight simulation packages. `reference/ros2_ws/` is a non-runnable design skeleton; do not treat it as part of the active workspace.
- `config/`, `web/templates/`, `models/`, and `assets/samples/` contain runtime configuration, the Flask page, model files and metadata, and development samples.
- `ops/` contains deployment, diagnostics, release, watchdog, and field-recording tools. `sim/` contains MuJoCo scenarios and batch experiments; optional dependencies are in `requirements/sim.txt`.
- `tests/` contains hardware-free pytest tests. `validation/` stores field-test summaries. `docs/` holds architecture, development, deployment, tuning, and research notes; `docs/html/` contains standalone visual guides.
- `releases/` and `output/pdf/` are output locations. Keep generated archives, caches, credentials, raw logs, and raw field images out of commits. `tools/legacy/` is historical and is not a current runtime entry point.

## Build, Test, and Development Commands

```console
python -m venv .venv
python -m pip install -r requirements/dev.txt
python -B -m pytest -q -p no:cacheprovider
python ops/build_release.py
python -m compileall -q src ops tests
```

Use `python src/camera_viewer.py` only with the required local backends and hardware configuration. To connect to an existing vehicle from PowerShell, use `& .\ops\start-follow-console.ps1`. On Linux, check shell syntax with `for script in ops/*.sh; do bash -n "$script"; done`.

## Coding Style & Naming

Follow `.editorconfig`: four spaces for Python, two for JSON/YAML, LF endings, and a final newline. Use `snake_case` for modules, functions, and variables; `PascalCase` for classes; and `test_*.py` / `test_*` for tests. No formatter or linter is configured, so rely on focused tests and syntax checks.

## Testing Guidelines

Pytest discovers tests under `tests/`; CI runs them on Linux and Windows with Python 3.11 and 3.12. There is no configured coverage threshold. Changes to speed, voltage, distance, or identity recovery should add or update control tests. Real-vehicle results belong in `validation/` with dates and conditions; state when hardware testing was not performed.

## Commit & Pull Request Guidelines

Recent human commits use short imperative summaries such as `Add ...`, `Document ...`, and `Organize ...`; keep automated checkpoints clearly prefixed. Pull requests should describe the trigger and changed behavior, list automated and hardware validation, and call out configuration, model, resource-path, ROS bridge, deployment, and rollback impacts. For UI changes, include a screenshot or a clear reproduction path.

## Configuration & Release Notes

When changing resources, model metadata, or deployment files, check the release mapping, ROS bridge copy lists, checksums, and extracted-runtime tests together. Preserve safety checks and the manual stop path in both follow modes.
