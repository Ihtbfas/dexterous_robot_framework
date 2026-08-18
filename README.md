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

## M1 Golden Isaac example

After M1-R8 is accepted, the tracked example `examples/isaac/tabletop_grasp_lift.py` is the public framework entry point for the first vertical slice:

`TabletopGraspLiftTask -> Skills -> Controllers -> RuntimeSession -> IsaacBackend`.

Local/private USD paths are deliberately not tracked. Start from `configs/assets/local_assets.example.yaml` or create an ignored `configs/local/*.yaml` with `wam_runtime` and `l20_runtime`. The one-click M1 migration runner creates the local file automatically from `DRF_M1_WAM_RUNTIME` and `DRF_M1_L20_RUNTIME` (or the frozen local defaults) and verifies the configured asset SHA256 values before and after the single Isaac run.

The M1 Golden acceptance contract requires WAM7+L20 load, grasp-lock completion, the cuboid leaving the table, at least 25 mm cuboid-center rise, at least 0.5 s suspended hold, and consistent Tensor/PhysX/USD/Fabric object transforms at the pre-lift, post-lift, and hold-end checkpoints.
