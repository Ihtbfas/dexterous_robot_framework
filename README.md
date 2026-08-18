# Dexterous Robot Framework

A cross-backend framework for composing robot arms, dexterous hands, controllers, skills, tasks, and runtime backends.

## M1 scope

The first milestone intentionally implements one vertical slice only: WAM7 + Linker Hand L20 + Isaac Sim tabletop grasp and lift. MuJoCo, real hardware, ROS2, tactile sensing, RL, and additional devices are later milestones.

## Repository boundary

Production source and formal tests are tracked. Local/private robot assets, local absolute-path configuration, experiments, run outputs, evidence archives, and scratch work are not part of the public source repository.

Architecture and implementation plan live under `docs/superpowers/`.

## Development test isolation

Use `python tools/run_tests.py ...` rather than invoking `pytest` directly when working in a shell that may expose unrelated ROS or system pytest plugins. The project runner disables third-party pytest entry-point autoload before pytest starts; backend-specific integration tests may explicitly opt into required plugins later.

Install the declared project dependencies with `python -m pip install -e '.[dev]'` in your preferred virtual environment. One-click migration runners may create a local ignored `.venv/` when the selected Python is missing project dependencies.
