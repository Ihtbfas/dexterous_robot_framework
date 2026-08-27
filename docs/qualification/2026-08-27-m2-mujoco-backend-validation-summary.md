# MuJoCo Backend Validation Summary

**Validated setup:** WAM7 + Linker Hand L20 + MuJoCo  
**Task:** Tabletop grasp, lift, and suspended hold  
**Status:** Validated for the current rigid-object vertical slice

## Result

The MuJoCo backend completed the same upper-layer task sequence used by the Isaac Sim vertical slice:

```text
APPROACH
→ GRASP
→ LIFT
→ HOLD
→ SUCCESS
```

Validated result:

| Metric | Result |
|---|---:|
| Commanded lift | +80 mm |
| Maximum actual object rise | ≈ 71.39 mm |
| Final net object rise | ≈ 67.39 mm |
| Suspended hold | ≈ 1.01 s |
| Final table normal force | 0 N |
| Final object-to-hand relative drift | ≈ 10.91 mm |
| Final opposing-Y squeeze telemetry | ≈ 5.49 N |

The accepted run also completed the interactive MuJoCo Viewer demonstration without visually apparent explosion, teleport, or severe jitter.

## Shared vs backend-specific semantics

Shared upper layers:

```text
Device Model
Controller
Skill
Task
RuntimeSession
Tabletop task configuration
Motion profiles and limits
```

MuJoCo-specific implementation:

```text
MjSpec robot/model assembly
MuJoCo scene construction
temporary object weld and dynamic release
contact-force telemetry
WAM actuator/servo policy
physics substep timing
```

## Assets

The repository does not embed the robot asset library. Configure:

```bash
export ROBOT_ASSETS_ROOT=/path/to/robot_assets
```

MuJoCo asset IDs:

```text
arm.wam7.mujoco.canonical_geometry_v2
hand.linker_l20.mujoco.right_v1
```

## Current scope notes

- The current tabletop object is rigid.
- `hand_tcp` currently follows the validated MuJoCo L20-base TCP policy used by this vertical slice.
- `opposing_y_squeeze_n` is a task-oriented lateral-grasp telemetry signal based on opposing world-Y contact components; it is not yet a general arbitrary-grasp squeeze metric.
- MuJoCo model attachment may emit non-blocking `znear`, `njmax`, and `nconmax` conflict warnings. They were present during the accepted model, contact, full-task, and Viewer validations.
- Research diagnostics, qualification scripts, and sealed evidence are intentionally retained outside the public release delta.
