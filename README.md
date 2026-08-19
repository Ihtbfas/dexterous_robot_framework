# Dexterous Robot Framework

> 面向**机械臂 + 灵巧手**组合的跨 Backend 机器人操控框架。
>
> 目标是在尽量保持 Device Model、Controller、Skill、Task 与 Runtime 语义一致的前提下，逐步支持 **Isaac Sim → MuJoCo → Real Robot**。

## Demo

### WAM7 + Linker Hand L20 · Isaac Sim Tabletop Grasp & Lift

[![WAM7 + Linker L20 Grasp & Lift Demo](docs/media/demos/m1_6_wam7_l20_grasp_lift_preview.gif)](https://github.com/Ihtbfas/dexterous_robot_framework/releases/tag/m1.6-motion-pacing-height-v1)

```text
Approach
→ Grasp
→ GraspLock
→ Lift
→ Hold
```

当前最新验证里程碑：

```text
m1.7-motion-profile-auto-timing-v1
```

当前真实 Isaac Sim 验证结果：

| 指标 | 结果 |
|---|---:|
| 完整流程 | ≈ 14.42 s |
| Lift command | +80 mm |
| 最大实际抬升 | ≈ 62.47 mm |
| Hold 后最终净抬升 | ≈ 55.13 mm |
| Suspended hold | ≈ 1.01 s |
| Final table normal | 0 N |

> 当前 GIF / 视频来自上一冻结 Demo 版本，用于展示相同的 WAM7 + L20 tabletop grasp/lift 任务。M1.7 的核心变化是 Motion Profile 与 Auto Timing，而不是抓取几何或任务流程。

---

## Key Features

- **Device-first**：机械臂与灵巧手作为独立 Device Model 接入。
- **Robot Composition**：通过 `ManipulatorSystem` 组合 Arm、Hand、Mount、Frame 与 TCP。
- **Task / Skill / Controller 分层**：任务语义、局部行为和控制逻辑保持清晰边界。
- **Backend-neutral Runtime**：上层逻辑通过 typed command 与 `RuntimeSnapshot` 运行，不直接依赖具体仿真器 API。
- **Asset Registry**：机器人资产独立于源码项目，通过逻辑 Asset ID 与 `ROBOT_ASSETS_ROOT` 管理。
- **Motion Profile + Auto Timing**：基于 joint / Cartesian limits 与 profile scaling 自动计算 minimum-jerk 运动时间。
- **Real Isaac Golden**：当前 WAM7 + Linker L20 tabletop grasp/lift 已通过真实 Isaac Sim 验证。

当前已支持 / 规划中的 Backend：

```text
Isaac Sim   ✅
MuJoCo      🚧
Real Robot  ⏳
```

---

## Architecture

核心运行链路：

```text
Task
 │
 ▼
Skill
 │
 ▼
Controller
 │
 ▼
Typed Command
 │
 ▼
Runtime / Session
 │
 ▼
Backend
 │
 ├── Isaac Sim
 ├── MuJoCo      # planned
 └── Real Robot  # planned
```

运动时间由独立 Motion 层解析：

```text
Device / Software Limits
          ×
Motion Profile
          ↓
Effective Limits
          ↓
Minimum-Jerk Auto Timing
          ↓
Executable Motion
```

基本原则：

- Task 描述**做什么**；
- Skill 描述可复用的局部行为；
- Controller 描述**如何产生控制命令**；
- Motion 层负责约束与轨迹时间语义；
- Runtime 统一 snapshot、clock、dispatch 与 backend lifecycle；
- Backend 负责连接具体仿真器或真实设备。

---

## Quick Start

### Python 环境

项目要求：

```text
Python >= 3.10
```

开发模式安装：

```bash
python -m pip install -e '.[dev]'
```

运行测试：

```bash
python tools/run_tests.py -q
```

建议使用项目自带 `tools/run_tests.py`，以减少 ROS、系统 Python 或第三方 pytest plugin 对测试环境的干扰。

### Robot Assets

机器人资产通过独立 Asset Registry 管理。设置本地资产根目录：

```bash
export ROBOT_ASSETS_ROOT=/path/to/robot_assets
```

主要 Asset ID：

```text
arm.wam7.isaac.canonical_geometry_v2
hand.linker_l20.isaac.dynamic_v1
```

Registry：

```text
configs/assets/registry.yaml
```

### Isaac Sim Demo

准备好兼容的 Isaac Sim 与 Robot Assets 后：

```bash
ISAAC_PY=/path/to/isaac-sim/python.sh

"$ISAAC_PY" examples/isaac/tabletop_grasp_lift.py \
  --backend-config configs/backends/isaac.yaml \
  --task-config configs/tasks/tabletop_grasp_lift.yaml \
  --robot-config configs/robots/wam7_linker_l20.yaml \
  --joint-limits configs/devices/arms/wam7_kinematic_limits.yaml \
  --cartesian-limits configs/motion/cartesian_limits.yaml \
  --motion-profiles configs/motion/profiles.yaml \
  --asset-registry configs/assets/registry.yaml \
  --asset-selection configs/assets/wam7_linker_l20_isaac.yaml \
  --asset-root-config configs/assets/robot_assets.example.yaml \
  --output runs/tabletop_grasp_lift_summary.json
```

添加 `--headless` 可使用无界面模式运行。

---

## Current Validated Setup

当前第一条真实验证的 vertical slice：

```text
WAM7
  +
Linker Hand L20
  +
Isaac Sim
```

任务入口：

```text
examples/isaac/tabletop_grasp_lift.py
```

M1.7 将原有 Task-level free-space duration 参数替换为：

```text
Kinematic Limits
+
Motion Profile
+
Minimum-Jerk Auto Timing
```

当前 Approach 使用实际 segment 起点状态计算 joint-space synchronized timing；Lift 使用 Cartesian software limits 自动计算运动时间。

---

## Project Structure

```text
.
├── configs/
│   ├── assets/
│   ├── backends/
│   ├── devices/
│   ├── motion/
│   ├── robots/
│   └── tasks/
│
├── examples/
│   └── isaac/
│
├── src/dexterous_robot/
│   ├── assets/
│   ├── backends/
│   │   └── isaac/
│   ├── config/
│   │   └── tasks/
│   ├── control/
│   ├── core/
│   ├── devices/
│   ├── golden/
│   ├── motion/
│   ├── robots/
│   ├── runtime/
│   ├── skills/
│   └── tasks/
│
├── tests/
├── tools/
├── docs/
├── pyproject.toml
└── README.md
```

---

## Roadmap

```text
✅ M1      Isaac vertical slice
✅ M1.5    Asset Registry / device-first robot_assets
✅ M1.6    Motion pacing + higher lift
✅ M1.7    Motion Profile + Auto Timing

🚧 M2      MuJoCo Backend

⏳ Real Backend
⏳ Sensors / Tactile
⏳ RL / Benchmark
⏳ More Devices / Tasks
```

M2 的目标是复用现有 Device / Robot / Controller / Skill / Task / Runtime 语义，仅新增 MuJoCo-specific backend 与必要的模型映射。

---

## Known Limitations

- 当前 Cartesian Lift 已使用 Cartesian limits 自动定时，但 **Cartesian → joint** 的完整 joint-space retiming 尚未实现。
- 当前正式验证范围仍以 **WAM7 + Linker Hand L20 + Isaac Sim tabletop grasp/lift** 为主。
- Robot Assets 由独立资产目录提供，不随源码仓库自动分发。

---

## Documentation

详细设计、实现计划与阶段性 review 文档位于：

```text
docs/
```

Demo 媒体约定：

```text
docs/media/demos/
```

当前 README 只保留项目定位、使用方式、最新验证状态与总体路线；更细的设计决策、实验记录和验证证据放在对应文档与测试中。
