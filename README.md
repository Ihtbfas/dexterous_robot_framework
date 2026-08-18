# Dexterous Robot Framework

A cross-backend framework for composing robot arms, dexterous hands, controllers, skills, tasks, and runtime backends.

## M1 scope

The first milestone intentionally implements one vertical slice only: WAM7 + Linker Hand L20 + Isaac Sim tabletop grasp and lift. MuJoCo, real hardware, ROS2, tactile sensing, RL, and additional devices are later milestones.

## Repository boundary

Production source and formal tests are tracked. Local/private robot assets, local absolute-path configuration, experiments, run outputs, evidence archives, and scratch work are not part of the public source repository.

Architecture and implementation plan live under `docs/superpowers/`.
