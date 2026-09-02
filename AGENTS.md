# Repository Guidelines

## Project Structure & Module Organization

This repository develops three ROSMASTER X3 demos using an RDK X5, MS200P LiDAR, and IMX219 camera. Put ROS packages in `src/`, shared parameters in `config/`, tests in `tests/`, scripts in `scripts/`, and runbooks in `docs/`. Keep vendor links and redistributable material in `reference/`; do not commit protected downloads or system images.

Keep package responsibilities narrow. Planned boundaries are bringup, base control, navigation, perception, and sorting. Hardware access should sit behind ROS topics or services so logic can be tested without a connected robot.

## Build, Test, and Development Commands

The target is ROS 2/TROS Humble on Ubuntu 22.04. Keep dependencies compatible with aarch64 and the vendor workspace.

- `./scripts/collect_system_info.sh` — collect read-only board, device, and ROS diagnostics.
- `git diff --check` — detect whitespace errors before committing.
- `python3 -m pytest tests` — run Python tests once the first package lands.
- `colcon build --symlink-install` — build the workspace after ROS packages are created.

On the robot, source `/opt/tros/humble/setup.bash` and the Yahboom workspace before building.

## Coding Style & Naming Conventions

Use four spaces in Python, PEP 8 naming, type hints on public APIs, and `snake_case` ROS package, node, topic, and parameter names. Use `PascalCase` for C++ types and `snake_case` for files and functions. Prefer Python unless profiling proves a C++ implementation is necessary. Format and lint with repository-pinned tools once introduced; do not mix drive-by formatting with behavior changes.

## Testing Guidelines

Use `pytest` for pure Python logic and ROS launch tests for node integration. Name tests after behavior, such as `test_stops_when_command_times_out`. Mock time, sensors, and motion outputs in automated tests. Every motion feature must include a zero-velocity shutdown path and a test for stale or missing input.

## Commit & Pull Request Guidelines

Use short Conventional Commit subjects, for example `feat: add figure-eight controller` or `docs: record lidar bringup`. This small project commits directly to `main`; keep each commit focused, tested, and safe to revert. Before pushing, review `git diff`, run relevant tests, and confirm no credentials, machine-specific logs, or large generated files are staged.

## Robot Safety

Never issue motion commands during environment discovery. For real-robot tests, lift the wheels or clear the test area first, keep an operator near the power control, begin at low speed, and verify that timeout, shutdown, and sensor-loss paths command zero velocity.
