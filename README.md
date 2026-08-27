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
- **Real MuJoCo Golden**：同一上层 Task / Skill / Controller 语义已通过真实 MuJoCo grasp/lift/hold 验证。

当前已支持 / 规划中的 Backend：

```text
Isaac Sim   ✅
MuJoCo      ✅
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
 ├── MuJoCo
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
arm.wam7.mujoco.canonical_geometry_v2
hand.linker_l20.mujoco.right_v1
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


### MuJoCo Demo

安装 MuJoCo 可选依赖：

```bash
python -m pip install -e '.[mujoco,dev]'
```

准备独立 Robot Assets：

```bash
export ROBOT_ASSETS_ROOT=/path/to/robot_assets
```

无界面运行完整 tabletop grasp/lift：

```bash
python examples/mujoco/tabletop_grasp_lift.py
```

打开交互式 MuJoCo Viewer：

```bash
python examples/mujoco/tabletop_grasp_lift.py --viewer
```

MuJoCo 使用与 Isaac Sim 相同的 `TabletopGraspLiftTask`、Approach / Grasp / Lift / Hold Skill、任务配置、Motion Profile 与 Runtime 语义；模型装配、接触求解、actuator/servo 与 scene plumbing 由 MuJoCo Backend 独立实现。

---

## Current Validated Setup


#### Known limitation: MuJoCo Viewer teardown

在当前验证环境 `Python 3.14.6 + MuJoCo 3.11.0 + glfw 2.10.2`（Linux）下，`mujoco.viewer.launch_passive()` 在任务已经完成并输出 `SUCCESS` 后，进程退出阶段可能触发 GLFW/native teardown abort 或 segmentation fault。该现象同时可在此前已验收的 Task10 Viewer runner 上复现，因此当前将其记录为 **Viewer teardown 的已知非阻塞环境问题**，而不是抓取、抬升、保持或 MuJoCo backend 功能失败。

正式功能验证以 headless Demo 的终态与数值指标为硬门槛；Viewer 用于辅助视觉确认。若 Viewer 已完成 `APPROACH → GRASP → LIFT → HOLD → SUCCESS`、`viewer_closed_early=False`，且结果文件已经写出，则任务功能视为完成，即使解释器退出阶段随后发生上述 native teardown 崩溃。

### MuJoCo Golden

当前 WAM7 + Linker Hand L20 MuJoCo tabletop grasp/lift 已完成真实数值验证与 Viewer 视觉确认：

| 指标 | MuJoCo 结果 |
|---|---:|
| Phase sequence | `APPROACH → GRASP → LIFT → HOLD → SUCCESS` |
| Lift command | +80 mm |
| 最大实际抬升 | ≈ 71.39 mm |
| Hold 后最终净抬升 | ≈ 67.39 mm |
| Suspended hold | ≈ 1.01 s |
| Final table normal | 0 N |
| Final relative drift | ≈ 10.91 mm |
| Final opposing-Y squeeze | ≈ 5.49 N |

> Isaac Sim 与 MuJoCo 共享上层 Device / Controller / Skill / Task / Runtime 语义，但物理引擎、机器人资产、接触遥测与 actuator/servo plumbing 保持 backend-specific。



当前已完成两条真实验证的 vertical slice：

```text
WAM7 + Linker Hand L20
├── Isaac Sim
└── MuJoCo
```

任务入口：

```text
examples/isaac/tabletop_grasp_lift.py
examples/mujoco/tabletop_grasp_lift.py
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
│   ├── isaac/
│   └── mujoco/
│
├── src/dexterous_robot/
│   ├── assets/
│   ├── backends/
│   │   ├── isaac/
│   │   └── mujoco/
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

✅ M2-B1   MuJoCo Model / Backend Qualification
✅ M2-B2   MuJoCo Tabletop Grasp & Lift

⏳ Real Backend
⏳ Sensors / Tactile
⏳ RL / Benchmark
⏳ More Devices / Tasks
```

M2-B1 已完成 WAM7 + Linker Hand L20 的 MuJoCo Model / Backend Qualification：模型装配、28-DOF 路由、typed position command、deterministic Runtime timing、WAM7 七轴受控运动以及 L20 Active16 → Physical21 coupling 均已通过。M2-B2 进一步复用现有 Device / Robot / Controller / Skill / Task / Runtime 语义，已在 MuJoCo 中完成 backend-neutral `TabletopGraspLiftTask` 的真实 grasp / lift / suspended hold 验证。下一步进入 Real Backend、Sensors / Tactile 与后续任务扩展。

---

## Known Limitations

- 当前 Cartesian Lift 已使用 Cartesian limits 自动定时，但 **Cartesian → joint** 的完整 joint-space retiming 尚未实现。
- Isaac Sim 与 MuJoCo 的 WAM7 + Linker Hand L20 tabletop grasp/lift vertical slice 均已完成真实验证；当前物体仍为刚体。
- MuJoCo 当前使用 broad robot-internal collision filtering；robot↔table/cube 外部接触已在当前 tabletop 任务中验证，但后续若任务依赖真实 self-contact，仍需重新审视该策略。
- MuJoCo `hand_tcp` 与 `opposing_y_squeeze_n` 目前仍是当前 lateral-grasp vertical slice 的任务化语义，尚不是任意抓取场景的通用 TCP / squeeze 定义。
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
