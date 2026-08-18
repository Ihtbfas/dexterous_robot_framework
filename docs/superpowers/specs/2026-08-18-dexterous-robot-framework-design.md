# Dexterous Robot Framework — 架构设计规范 V0

- 日期：2026-08-18
- 状态：SPEC APPROVED / IMPLEMENTATION AUTHORIZED
- 目标仓库：`/home/lyf/dexterous_robot_framework`
- Python package：`dexterous_robot`
- 旧项目冻结参考：`/home/lyf/worktrees/wam_linkerhand_sim/phase2b0-implementation`
- M1 Golden Reference：旧项目 R15U WAM7 + Linker Hand L20 tabletop grasp & lift

---

## 1. 项目定位

本项目定位为**通用机械臂 + 通用灵巧手 + 多运行后端**的机器人操控与实验框架。

长期目标：

1. 同一上层 Task / Skill / Controller 能够运行在 Isaac Sim、MuJoCo 与 Real 后端上；
2. 支持不同机械臂与灵巧手组合，而不是绑定 WAM7 或 Linker Hand L20；
3. 允许后续加入触觉、视觉、RL、行为树、高级任务与更多设备；
4. 将研究实验代码与正式产品代码严格分离；
5. 保留正式、可执行、可回归的 tests，并避免历史实验脚本侵入 production source tree。

当前第一阶段只实现一条最小但完整的竖向切片：

- Backend：Isaac Sim
- Arm：WAM7
- Hand：Linker Hand L20
- Task：Tabletop Grasp + Lift
- Golden Reference：旧项目 R15U

MuJoCo、Real、ROS2、Direct SDK、触觉、RL、其他机械臂/灵巧手不属于 M1 实现范围，但架构不得明显阻碍后续接入。

---

## 2. 冻结旧项目与迁移原则

旧项目从本规范生效后作为：

> Research Legacy / Evidence Authority / Golden Reference

原则：

- 不在旧项目上继续新增功能；
- 不为了“整洁”对旧项目做大规模清理；
- 不删除历史实验、sealed archive、诊断证据与 qualification 工具；
- 新项目遇到问题时可以只读查询旧项目与 R15U 行为；
- 新项目是唯一新的开发主线。

迁移遵循：

> 迁移经过验证的知识、接口语义与行为，不复制实验历史组织方式。

旧项目中值得优先迁移/重实现的知识包括：

- Linker L20 16 active / 20 protocol / 21 physical 三空间定义；
- 16 leaders + 5 followers coupling 语义；
- Active16 → Physical21 mapping；
- Active16 → Protocol20 codec；
- joint limits 与 device-specific command/state 语义；
- minimum-jerk 等纯数学工具；
- WAM Cartesian IK 的已验证行为；
- R15U 的 grasp-lock 与 Cartesian carry 控制思想；
- R15U 中已验证的 Isaac PhysX / USD / Fabric 动态 transform 同步处理知识。

明确不迁移到正式主源码：

- R* / D* / V* 历史实验编号体系；
- 一次性 probe / hotfix / finalizer；
- 历史 one-click runner；
- 大量 run logs / screenshots / archives；
- R15U 2800+ 行的一体化 runner 结构。

---

## 3. 顶层架构

```text
                         Task
             全局目标 / 编排 / Recovery
                          │
                    SkillResult
          RUNNING / SUCCESS / FAILURE
                          │
                         Skill
              可复用行为 / 局部状态机
                          │
                      Controller
                单周期控制计算 / 数学状态
                          │
                    Typed Command
                          │
                   Runtime / Session
       clock / snapshot / routing / safety / hooks
                          │
                       Backend
              ┌───────────┼───────────┐
            Isaac       MuJoCo       Real
                                    /    \
                                 Direct   ROS2

横向基础：
Device Model ───── Robot Composition ───── Asset Registry
```

核心约束：

> 仿真器/真机差异只能向下渗透，不能向上渗透。

禁止 Task / Skill / Controller 出现针对 backend 的业务分支，例如：

```python
if backend == "isaac":
    ...
elif backend == "mujoco":
    ...
```

若上层业务逻辑需要判断具体 backend，优先视为接口边界设计失败。

---

## 4. Core Layer

### 4.1 职责

Core 只定义真正跨机器人、跨后端的数据类型和基础语义。

初始候选：

- `Pose`
- `Twist`
- `JointState`
- `Timestamp`
- `FrameId`
- `RuntimeSnapshot`
- `SkillStatus`
- `SkillResult`
- `FailureReason`
- 通用 typed command 基类/协议

### 4.2 禁止内容

Core 不允许依赖：

- Isaac Sim API；
- MuJoCo API；
- ROS2；
- WAM 专有 joint 名；
- Linker L20 的 16/20/21 专有语义；
- 具体 USD / MJCF / SDK 路径。

---

## 5. Device Model Layer

### 5.1 定位

Device Model 描述：

> “这个设备是什么、有哪些状态和可接受命令”

而不是：

> “这个设备在某个任务里应该怎么动作”。

### 5.2 初始目录

```text
src/dexterous_robot/devices/
├── arms/
│   └── wam7/
├── hands/
│   └── linker_l20/
└── sensors/
```

### 5.3 Linker L20

L20 专属定义只存在于 `devices/hands/linker_l20/` 内，例如：

- `L20ActiveCommand16`
- `L20ProtocolCommand20`
- `L20PhysicalTarget21`
- `L20PhysicalState21`
- leader/follower mapping
- coupling profiles
- joint limits
- official protocol codec
- grasp presets

框架最高层禁止建立在“16 通道”假设上。

未来 Allegro、Shadow、Inspire 等设备可以有完全不同的 command space。

---

## 6. Robot Composition

### 6.1 定位

`ManipulatorSystem` 负责组合：

- Arm Device
- Hand Device
- Sensors
- Mount
- TCP / frame relationships

### 6.2 职责

`ManipulatorSystem` 可以负责：

- 设备组合关系；
- base / flange / hand / TCP frames；
- mount transform；
- joint/device routing；
- state aggregation；
- backend physical topology 与 logical device topology 的映射入口。

### 6.3 明确禁止

`ManipulatorSystem` 不提供任务动作 API：

- `move_ee()`
- `close_hand()`
- `grasp()`
- `lift()`

这些动作属于 Skill / Controller。

---

## 7. Controller Layer

### 7.1 设计原则

Controller 尽量满足：

```text
state + target + dt
        ↓
      command
```

Controller 不直接驱动 backend。

禁止：

```python
physx_view.set_joint_position_targets(...)
mj_step(...)
publisher.publish(...)
```

Controller 必须返回 typed command，由 Runtime 统一 dispatch。

### 7.2 初始候选

- `JointPositionController`
- `JointTorqueController`
- `CartesianPoseController`
- `GraspLockController`
- `CartesianCarryController`

未来：

- `TactileForceController`
- RL policy adapter
- Whole-arm-hand coordination controllers

---

## 8. Skill Layer

### 8.1 定位

Skill 是可复用的局部机器人行为。

初始候选：

- `ApproachSkill`
- `GraspSkill`
- `LiftSkill`
- `HoldSkill`

未来：

- `PlaceSkill`
- `ReleaseSkill`
- `ReorientSkill`
- `TactileAdjustSkill`

### 8.2 SkillResult Contract

每个 Skill 对 Task 只暴露：

- `RUNNING`
- `SUCCESS`
- `FAILURE`

并可附带语义化失败原因，例如：

- `OBJECT_SLIPPED`
- `TARGET_UNREACHABLE`
- `GRASP_NOT_ESTABLISHED`
- `TIMEOUT`

Task 不直接读取 Skill 的底层判据，例如 finger-level force threshold 或某个 Isaac tracking metric。

### 8.3 Recovery 边界

Skill 只负责回答：

> “这个技能完成了吗？为什么失败？”

Task 负责决定：

> “失败以后下一步做什么？”

Skill 不允许偷偷跳回其他 Skill 或实现跨任务阶段的隐藏 recovery。

---

## 9. Task Layer

### 9.1 定位

Task 负责：

- 全局任务流程；
- 全局成功/失败定义；
- 阶段转换；
- recovery policy；
- task timeout；
- task-level metrics。

M1 Task：

```text
TabletopGraspLiftTask
└── Sequence
    ├── ApproachSkill
    ├── GraspSkill
    ├── LiftSkill
    └── HoldSkill
```

### 9.2 Behavior Tree 决策

M1 不引入完整 Behavior Tree 第三方框架。

但 Skill API 必须 BT-compatible：

- `RUNNING`
- `SUCCESS`
- `FAILURE`

因此未来真正需要 Retry / Fallback / complex recovery 时可以升级为正式 BT engine，而无需破坏 Skill contract。

---

## 10. Runtime / Session Layer

### 10.1 定位

Runtime/Session 是统一跨 Isaac / MuJoCo / Real 的控制循环和时间语义层。

负责：

- clock；
- `dt`；
- reset / lifecycle；
- backend stepping；
- state sampling；
- command dispatch；
- logging hooks；
- safety hooks；
- session-level telemetry。

### 10.2 单周期数据流

```text
Backend state
     ↓
Runtime
     ↓
RuntimeSnapshot(t)
     ↓
Task → Skill → Controller
     ↓
Typed Command
     ↓
Runtime
     ↓
Backend
```

### 10.3 Context / Blackboard 决策

不采用全局任意可写 Blackboard。

Runtime 每周期生成尽量 immutable 的 `RuntimeSnapshot`。

Task / Skill / Controller 主要读取 snapshot；目标变化和动作请求通过显式 typed goal / command 传递，而不是任意修改全局 context。

---

## 11. Backend Layer

### 11.1 最小 Backend Contract

第一版只设计实际需要的最小接口，例如：

```python
class Backend:
    def initialize(...): ...
    def reset(...): ...
    def read_state(...): ...
    def apply(commands): ...
    def step(...): ...
    def shutdown(...): ...
```

不在 M1 阶段预先设计“万能后端 API”。

具体能力通过 device/backend adapter 或 capability contract 增量扩展。

### 11.2 Isaac Backend

Isaac-specific 内容只能存在于 `backends/isaac/`：

- USD
- PhysX
- Tensor API
- Fabric / USDRT
- Articulation / RigidBodyView
- contact reports
- viewport / render synchronization

WAM7 + L20 在 Isaac 内部即使表现为 combined articulation，也必须被 backend 隐藏，上层仍然看到独立 logical Arm / Hand devices。

### 11.3 MuJoCo Backend

M1 不实现，只预留模块边界。

未来内部负责：

- `MjModel`
- `MjData`
- `mj_step`
- actuator
- contact
- sensor

### 11.4 Real Backend

正式选择：Core 不依赖 ROS2，Real backend 同时允许两种 transport：

```text
RealBackend
├── Direct Adapter
│   └── SDK / CAN / native API
└── ROS2 Adapter
    └── Topic / Service / Action
```

ROS2 是 integration/transport adapter，而不是 framework 基础依赖。

---

## 12. Asset Strategy

### 12.1 源码与资产分离

源码仓库：

`/home/lyf/dexterous_robot_framework`

未来本地私有资产库：

`/home/lyf/robot_assets`

源码仓库不直接包含不确定许可、体积过大的 WAM/L20 private/restricted assets。

### 12.2 Asset Registry

未来通过逻辑资产 ID 解析真实路径，例如：

```yaml
wam7:
  backend: isaac
  asset: canonical_v2
```

由 Asset Registry 解析为机器本地真实路径。

正式代码禁止硬编码：

`/home/lyf/isaacsim_projects/...`

### 12.3 M1 过渡策略

M1 不迁移资产。

M1 新项目通过 `.gitignore` 的 local/private config **只读引用现有成功资产**。

M1 Golden Demo 复现后，再单独整理 `/home/lyf/robot_assets`，避免代码重构与 USD 资产迁移同时发生。

---

## 13. Configuration Strategy

采用：

> YAML + validated/frozen dataclass

不在 M1 引入 Hydra/OmegaConf。

建议目录：

```text
configs/
├── devices/
│   ├── arms/
│   └── hands/
├── robots/
├── backends/
├── tasks/
└── assets/
```

示例：

- `wam7.yaml`
- `linker_l20.yaml`
- `wam7_linker_l20.yaml`
- `isaac.yaml`
- `tabletop_grasp_lift.yaml`

业务代码不长期依赖深层 dict access，而是在入口把 YAML 校验/解析成 typed config。

本机旧资产路径使用 `configs/local/` 或等价 private config，并由 `.gitignore` 排除。

---

## 14. Repository Layout V0

```text
/home/lyf/dexterous_robot_framework
│
├── pyproject.toml
├── README.md
├── .gitignore
│
├── src/
│   └── dexterous_robot/
│       ├── core/
│       ├── devices/
│       │   ├── arms/
│       │   │   └── wam7/
│       │   ├── hands/
│       │   │   └── linker_l20/
│       │   └── sensors/
│       ├── robots/
│       ├── control/
│       │   ├── arm/
│       │   ├── hand/
│       │   └── coordination/
│       ├── skills/
│       ├── tasks/
│       ├── runtime/
│       ├── backends/
│       │   ├── isaac/
│       │   ├── mujoco/
│       │   └── real/
│       │       ├── direct/
│       │       └── ros2/
│       └── assets/
│
├── configs/
├── tests/
│   ├── unit/
│   ├── contract/
│   ├── integration/
│   └── golden/
├── examples/
├── experiments/
├── runs/
├── evidence/
├── scratch/
└── docs/
    └── superpowers/
        └── specs/
```

`experiments/`、`runs/`、`evidence/`、`scratch/` 默认不上传公开仓库；正式 `tests/` 必须进入 Git。

空目录不会为了“好看”全部预建；bootstrap 只创建当前阶段确实需要的文件与目录。

---

## 15. Testing Strategy

### 15.1 Unit Tests

测试纯算法/数据转换：

- pose math
- minimum jerk
- mapping
- codec
- controller math

### 15.2 Contract Tests

M1 最重要：

- L20 Active16 → Physical21
- L20 Active16 → Protocol20
- follower coupling semantics
- Backend command/state contracts
- SkillResult semantics
- RuntimeSnapshot immutability / ownership boundaries

### 15.3 Integration Tests

逐步加入：

- IsaacBackend + WAM7
- IsaacBackend + L20
- WAM7 + L20 Robot Composition
- Runtime + Isaac Backend loop

### 15.4 Golden Tests

M1 最终 golden scenario：

> WAM7 + Linker L20 + tabletop lateral grasp + lift + hold

它是 R15U 的正式产品化继承者，而不是 R15U 文件本身的复制。

---

## 16. Experiments → Production Promotion

所有探索性修改首先进入：

`experiments/`

成功后必须经过：

```text
Experiment
   ↓
确认机制有效
   ↓
提炼 production behavior
   ↓
src/
+
tests/
```

禁止把实验脚本简单改名后直接放入 production source tree。

新项目主源码不得重新演化出无限增长的 R15A/R15B/R15C... 实验版本链。

---

## 17. M1 Implementation Slices

M1 不一次性实现完整未来框架，而按最小可验证竖向切片推进：

```text
M1-R0  Project bootstrap
M1-R1  Core types + config loader
M1-R2  L20 Device Model + 16/20/21 mapping
M1-R3  Backend + Runtime minimal contracts
M1-R4  WAM7 Device + ManipulatorSystem
M1-R5  IsaacBackend minimal implementation
M1-R6  GraspLock / CartesianCarry controllers
M1-R7  Skills + TabletopGraspLiftTask
M1-R8  Golden Demo
```

每个 slice 都必须：

1. 有 focused tests；
2. 有 contract/regression evidence；
3. 能生成 review/sealed evidence bundle；
4. 不依赖手工复制单个文件。

用户交互继续采用：

```text
Assistant 提供完整包
→ 用户下载
→ 用户一次性执行命令块
→ 自动测试/运行
→ 自动生成 sealed archive
→ 用户回传 archive
```

---

## 18. M1 Golden Demo Acceptance Contract

M1 固定：

- Backend：Isaac Sim
- Arm：WAM7
- Hand：Linker Hand L20
- Object：50 × 50 × 65 mm, 50 g cuboid
- Task：Tabletop lateral grasp + lift

硬性 PASS：

1. WAM7 + L20 能通过新 framework 正常加载并进入任务；
2. 能完成 lateral grasp 并进入稳定 grasp-lock；
3. cuboid 真实离开桌面；
4. cuboid center Z rise ≥ 25 mm；
5. 离桌后连续 hold ≥ 0.5 s，不掉落；
6. Physics / USD / viewport 三者状态与显示一致。

Golden Reference（非硬阈值）：

- R15U commanded hand lift ≈ 50 mm
- actual hand rise ≈ 45 mm
- cuboid rise ≈ 34 mm
- final table normal = 0
- hold success

M1 不要求逐项复制 R15U 数值；目标是行为等价和架构干净。

---

## 19. M1 Explicit Non-Goals

M1 不实现：

- MuJoCo backend
- Real backend
- ROS2 adapter
- Direct SDK adapter
- Tactile pipeline
- RL integration
- 其他机械臂
- 其他灵巧手
- 正式 Behavior Tree engine
- 正式 `/home/lyf/robot_assets` 资产迁移
- 抓取性能优化
- 云端 CI 的完整 Isaac Sim GPU 集成测试

这些是后续 milestone，不得阻塞 M1。

---

## 20. Error Handling Principles

1. **Semantic errors upward, backend details downward**：Task/Skill 接收语义化 failure reason，不直接接触 PhysX/MuJoCo/SDK exception details。
2. **No hidden recovery in Skill**：Skill 不跨阶段自行恢复；Task 决定 retry/fallback/abort。
3. **Runtime owns lifecycle**：Backend initialization/reset/step/shutdown 异常由 Runtime 统一收口并生成 evidence。
4. **Fail closed only for structural/runtime integrity**：M1 调试过程中不重新引入旧 Demo 阶段那种过多 physics telemetry hard-gate；任务判据与结构完整性分开。
5. **Evidence before promotion**：实验结果只有通过 tests + runtime evidence 才能 promotion 到 production modules。

---

## 21. Public Repository / Private Asset Boundary

未来 GitHub/Gitee 仓库应包含：

- source code
- configs/examples
- formal tests
- docs
- asset manifests
- license / source attribution metadata
- asset placement/download instructions

默认不公开：

- restricted/private CAD/mesh/USD
- vendor manuals with uncertain redistribution rights
- local absolute-path config
- generated runtime cache
- logs/videos/sealed evidence archives
- experiments/scratch outputs

在第一次公开 push 前必须单独执行：

- license audit
- large-file audit
- secret/token scan
- absolute-path scan
- generated/cache scan

---

## 22. Design Invariants

以下原则后续修改必须显式讨论，不允许无意破坏：

1. Core 不依赖具体 backend/device；
2. Task/Skill/Controller 不直接调用 Isaac/MuJoCo/ROS2/SDK；
3. Controller 输出 typed command，不直接驱动 backend；
4. Runtime 统一 clock、snapshot、dispatch 与 lifecycle；
5. RuntimeSnapshot 尽量 immutable，不使用任意可变全局 Blackboard；
6. Skill 返回 RUNNING/SUCCESS/FAILURE + semantic reason；
7. Task 负责全局 recovery；
8. Arm 与 Hand 是独立 Device Model，由 ManipulatorSystem 组合；
9. ManipulatorSystem 不承担任务动作；
10. L20 Active16/Protocol20/Physical21 只属于 L20 设备域；
11. 源码仓库与 private robot assets 分离；
12. experiments 与 production source 分离；
13. 正式 tests 必须进入 Git；
14. M1 以 R15U 行为为 oracle，但不复制 R15U 实验架构。

---

## 23. Spec Review Checklist

### Placeholder scan

- 无 TBD / TODO / 未决顶层架构项。

### Internal consistency

- Task/Skill/Controller/Runtime/Backend 职责与数据流一致；
- Real backend C 方案与 Core 无 ROS2 依赖一致；
- Asset Registry 与 M1 只读旧资产策略一致；
- L20 device-specific 语义没有泄露到 Core。

### Scope

- M1 仅实现 Isaac + WAM7 + L20 + Golden Demo；
- MuJoCo/Real/触觉等明确延后；
- 范围适合拆成 M1-R0 ~ M1-R8 多个 implementation slices，而不是单个超大提交。

### Ambiguity

- `ManipulatorSystem` 不提供动作 API；
- Skill 不负责跨阶段 recovery；
- M1 不引第三方 BT engine；
- M1 不迁正式 robot_assets；
- 正式 `tests/` 上传 Git，实验输出默认不上传。

---

## 24. Approval State

截至 2026-08-18，以下设计方向已在对话中逐项获得用户确认：

- 通用机械臂 + 通用灵巧手（B 型定位）；
- Core 无 ROS2 依赖，Real 同时支持 Direct 与 ROS2（C 方案）；
- 独立 Runtime / Session 层统一控制循环（C 方案）；
- Task + Skill + Controller 三层分工（C 方案）；
- SkillStatus / typed command / immutable snapshot 方向；
- BT-compatible semantics，但 M1 不引完整 BT engine；
- 独立 Arm/Hand Device Model + ManipulatorSystem 组合；
- ManipulatorSystem 不承担动作逻辑；
- 源码与 `/home/lyf/robot_assets` 私有资产库分离；
- M1 暂时只读引用旧成功 assets；
- YAML + validated/frozen dataclass 配置；
- repo 路径 `/home/lyf/dexterous_robot_framework`；
- M1 Golden Demo 六项验收标准。

用户已于 2026-08-18 明确确认本规范；implementation plan 与 M1-R0 bootstrap 已获授权。
