# Dexterous Robot Framework M1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `/home/lyf/dexterous_robot_framework` as a clean cross-backend manipulator framework and reproduce the frozen R15U WAM7 + Linker L20 Isaac Sim tabletop grasp-and-lift behavior without copying the legacy experiment-runner architecture.

**Architecture:** Implement one vertical M1 slice: immutable core types → Linker L20 device semantics → runtime/backend contracts → WAM7 + ManipulatorSystem → minimal Isaac backend → backend-independent controllers → BT-compatible skills/task → golden Isaac demo. The frozen legacy project remains read-only behavioral authority. Production code never imports legacy `scripts.phase2` modules; legacy behavior is transferred through contract fixtures, documented parameters, and golden runtime evidence.

**Tech Stack:** Python 3.10+ (Isaac execution uses Isaac Sim 6.0 Python 3.12), stdlib `dataclasses`/`enum`/`abc`, NumPy, PyYAML, pytest, NVIDIA Isaac Sim 6.0 APIs for the Isaac backend only.

**Spec:** `docs/superpowers/specs/2026-08-18-dexterous-robot-framework-design.md`

## Global Constraints

- Repository root is exactly `/home/lyf/dexterous_robot_framework`.
- Python package is exactly `dexterous_robot`.
- Legacy project `/home/lyf/worktrees/wam_linkerhand_sim/phase2b0-implementation` is read-only reference; never clean/reset/delete/commit there as part of M1.
- M1 implements only Isaac Sim + WAM7 + Linker L20 + tabletop grasp/lift.
- Core, Task, Skill, and Controller must not import Isaac, MuJoCo, ROS2, or vendor SDK APIs.
- Controller returns typed commands and never dispatches directly to a backend.
- Runtime owns clock, immutable snapshot creation, command dispatch, backend lifecycle, and evidence hooks.
- Arm and Hand remain independent device models and are composed by `ManipulatorSystem`.
- L20 Active16 / Protocol20 / Physical21 semantics exist only under the Linker L20 device package.
- M1 uses YAML + validated/frozen dataclasses; no Hydra/OmegaConf.
- M1 reads existing private WAM/L20 assets through gitignored local configuration; it does not migrate assets to `/home/lyf/robot_assets` yet.
- `tests/` is tracked; `experiments/`, `runs/`, `evidence/`, `scratch/`, `configs/local/` are ignored.
- M1 golden PASS: load WAM7+L20, establish lateral grasp lock, object leaves table, cuboid center Z rises at least 0.025 m, suspended hold lasts at least 0.5 s, and Physics/USD/viewport transforms agree.
- Structural/runtime integrity may fail closed; physics telemetry such as historical B2 measurements is evidence-only for this Demo milestone unless it indicates non-finite/corrupt state.
- User interaction remains one-click: each M1 slice is delivered as a complete archive + single runner and emits a sealed review archive.

---

## File Structure Locked by This Plan

```text
/home/lyf/dexterous_robot_framework/
├── pyproject.toml
├── README.md
├── .gitignore
├── src/dexterous_robot/
│   ├── __init__.py
│   ├── core/
│   │   ├── __init__.py
│   │   ├── geometry.py           # backend-neutral Pose/Twist
│   │   ├── joints.py             # immutable JointState
│   │   ├── commands.py           # typed backend-neutral commands
│   │   └── skills.py             # SkillStatus/SkillResult/FailureReason
│   ├── config/
│   │   ├── __init__.py
│   │   ├── models.py             # frozen config dataclasses
│   │   └── loader.py             # YAML -> typed config
│   ├── devices/
│   │   ├── arms/wam7/
│   │   │   ├── __init__.py
│   │   │   └── model.py          # WAM7 logical device model
│   │   └── hands/linker_l20/
│   │       ├── __init__.py
│   │       ├── types.py          # Active16/Protocol20/Physical21
│   │       ├── mapping.py        # Active16 -> Physical21
│   │       ├── protocol.py       # Active16 -> official20
│   │       └── model.py          # L20 device metadata/presets
│   ├── robots/
│   │   ├── __init__.py
│   │   └── manipulator.py        # Arm+Hand+mount composition only
│   ├── runtime/
│   │   ├── __init__.py
│   │   ├── snapshot.py           # immutable RuntimeSnapshot
│   │   └── session.py            # lifecycle/clock/dispatch loop
│   ├── backends/
│   │   ├── __init__.py
│   │   ├── base.py               # minimal Backend ABC + BackendState
│   │   └── isaac/
│   │       ├── __init__.py
│   │       ├── backend.py        # IsaacBackend implementation
│   │       ├── topology.py       # combined-28 routing hidden here
│   │       ├── scene.py          # WAM/L20/table/cuboid authoring/loading
│   │       ├── contacts.py       # normalized contact summary
│   │       └── transform_sync.py # PhysX/USD/Fabric sync incl. R15U fix
│   ├── control/
│   │   ├── math/
│   │   │   └── minimum_jerk.py
│   │   ├── arm/
│   │   │   ├── kinematics.py     # WAM7 FK/IK pure math
│   │   │   └── cartesian_carry.py
│   │   └── hand/
│   │       └── grasp_lock.py
│   ├── skills/
│   │   ├── approach.py
│   │   ├── grasp.py
│   │   ├── lift.py
│   │   └── hold.py
│   └── tasks/
│       └── tabletop_grasp_lift.py
├── configs/
│   ├── devices/arms/wam7.yaml
│   ├── devices/hands/linker_l20.yaml
│   ├── robots/wam7_linker_l20.yaml
│   ├── backends/isaac.yaml
│   ├── tasks/tabletop_grasp_lift.yaml
│   └── local/.gitkeep.example    # directory documentation only; real local files ignored
├── examples/isaac/tabletop_grasp_lift.py
├── tests/
│   ├── unit/
│   ├── contract/
│   ├── integration/
│   └── golden/
├── tools/review/
│   ├── build_review_bundle.py
│   └── verify_repo_boundary.py
└── docs/superpowers/{specs,plans}/
```

---

### Task 1: M1-R0 Repository Bootstrap and Guardrails

**Files:**
- Create: `pyproject.toml`
- Create: `.gitignore`
- Create: `README.md`
- Create: `src/dexterous_robot/__init__.py`
- Create: `tests/contract/test_repository_boundary.py`
- Create: `tools/review/verify_repo_boundary.py`
- Create: `tools/review/build_review_bundle.py`
- Copy: `docs/superpowers/specs/2026-08-18-dexterous-robot-framework-design.md`
- Copy: `docs/superpowers/plans/2026-08-18-dexterous-robot-framework-m1-implementation-plan.md`

**Interfaces:**
- Consumes: approved design spec and this plan.
- Produces: importable package skeleton, repository-boundary verifier, sealed-review utility used by all later slices.

- [ ] **Step 1: Write the failing repository-boundary test**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_required_gitignore_boundaries_are_declared():
    text = (ROOT / ".gitignore").read_text(encoding="utf-8")
    for pattern in ("configs/local/", "experiments/", "runs/", "evidence/", "scratch/"):
        assert pattern in text


def test_production_tree_contains_no_legacy_revision_names():
    forbidden = ("r15", "phase2b", "p2b2")
    offenders = []
    for path in (ROOT / "src").rglob("*"):
        if path.is_file() and any(token in path.name.lower() for token in forbidden):
            offenders.append(str(path.relative_to(ROOT)))
    assert offenders == []
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
python -m pytest tests/contract/test_repository_boundary.py -q
```

Expected: FAIL because the new repository files do not exist yet.

- [ ] **Step 3: Create the minimal package/bootstrap files**

`pyproject.toml` must contain:

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "dexterous-robot-framework"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["numpy>=1.24", "PyYAML>=6.0"]

[project.optional-dependencies]
dev = ["pytest>=8"]

[tool.pytest.ini_options]
pythonpath = ["src"]
testpaths = ["tests"]
```

`.gitignore` must contain at least:

```gitignore
configs/local/
experiments/
runs/
evidence/
scratch/
__pycache__/
.pytest_cache/
*.pyc
```

`src/dexterous_robot/__init__.py`:

```python
"""Cross-backend manipulator + dexterous-hand framework."""

__version__ = "0.1.0"
```

- [ ] **Step 4: Implement repository verification**

`tools/review/verify_repo_boundary.py` must fail non-zero when tracked production source contains legacy experiment revision names, hard-coded `/home/lyf/isaacsim_projects/` paths, or files under ignored research-output directories.

Core scanner shape:

```python
FORBIDDEN_SOURCE_TOKENS = ("phase2b", "p2b2", "r15u")
FORBIDDEN_ABSOLUTE_PREFIX = "/home/lyf/isaacsim_projects/"
IGNORED_TOP_LEVEL = {"experiments", "runs", "evidence", "scratch"}
```

- [ ] **Step 5: Implement sealed review bundling**

`build_review_bundle.py` accepts:

```text
--review-dir PATH
--output PATH.tar.xz
--classification STRING
--status PASS|BLOCKED
```

It writes `review_summary.json`, `SHA256SUMS`, then creates the `.tar.xz`. It must never modify the legacy project.

- [ ] **Step 6: Run GREEN verification**

Run:

```bash
python -m pytest tests/contract/test_repository_boundary.py -q
python tools/review/verify_repo_boundary.py
python -m compileall -q src tools
```

Expected: PASS.

- [ ] **Step 7: Initialize Git and commit M1-R0**

```bash
git init
git add .
git commit -m "chore: bootstrap dexterous robot framework"
```

- [ ] **Step 8: Produce M1-R0 review archive**

The one-click M1-R0 runner records Python version, git status/log, tree, pytest output, repo-boundary output, and SHA256 of spec/plan; classification `M1_R0_BOOTSTRAP_READY_FOR_REVIEW`.

---

### Task 2: M1-R1 Immutable Core Types and Typed YAML Configuration

**Files:**
- Create: `src/dexterous_robot/core/geometry.py`
- Create: `src/dexterous_robot/core/joints.py`
- Create: `src/dexterous_robot/core/commands.py`
- Create: `src/dexterous_robot/core/skills.py`
- Create: `src/dexterous_robot/config/models.py`
- Create: `src/dexterous_robot/config/loader.py`
- Create: `tests/unit/test_core_types.py`
- Create: `tests/unit/test_config_loader.py`

**Interfaces:**
- Produces: `Pose`, `JointState`, `JointPositionCommand`, `JointEffortCommand`, `SkillStatus`, `FailureReason`, `SkillResult`, and frozen config loaders.
- Consumed by: every later task.

- [ ] **Step 1: Write RED tests for immutable validated core types**

```python
import dataclasses
import pytest
from dexterous_robot.core.geometry import Pose
from dexterous_robot.core.joints import JointState
from dexterous_robot.core.skills import FailureReason, SkillResult, SkillStatus


def test_pose_is_frozen_and_exact_width():
    pose = Pose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), "world")
    with pytest.raises(dataclasses.FrozenInstanceError):
        pose.frame_id = "other"
    with pytest.raises(ValueError, match="POSE_POSITION_INVALID"):
        Pose((1.0, 2.0), (0.0, 0.0, 0.0, 1.0), "world")


def test_joint_state_rejects_width_mismatch():
    with pytest.raises(ValueError, match="JOINT_STATE_WIDTH_MISMATCH"):
        JointState(("j1", "j2"), (0.0,), (0.0, 0.0), None)


def test_skill_result_is_semantic_only():
    result = SkillResult(SkillStatus.FAILURE, FailureReason.OBJECT_SLIPPED, "lost object")
    assert result.status is SkillStatus.FAILURE
    assert result.reason is FailureReason.OBJECT_SLIPPED
```

- [ ] **Step 2: Run RED**

```bash
pytest tests/unit/test_core_types.py -q
```

Expected: import failure.

- [ ] **Step 3: Implement core dataclasses**

Required public signatures:

```python
@dataclass(frozen=True)
class Pose:
    position_xyz_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    frame_id: str

@dataclass(frozen=True)
class JointState:
    names: tuple[str, ...]
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    effort_nm: tuple[float, ...] | None = None

@dataclass(frozen=True)
class JointPositionCommand:
    device_id: str
    joint_names: tuple[str, ...]
    position_rad: tuple[float, ...]
    profile: str | None = None

@dataclass(frozen=True)
class JointEffortCommand:
    device_id: str
    joint_names: tuple[str, ...]
    effort_nm: tuple[float, ...]

class SkillStatus(Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"

class FailureReason(Enum):
    NONE = "NONE"
    OBJECT_SLIPPED = "OBJECT_SLIPPED"
    TARGET_UNREACHABLE = "TARGET_UNREACHABLE"
    GRASP_NOT_ESTABLISHED = "GRASP_NOT_ESTABLISHED"
    TIMEOUT = "TIMEOUT"
    RUNTIME_ERROR = "RUNTIME_ERROR"

@dataclass(frozen=True)
class SkillResult:
    status: SkillStatus
    reason: FailureReason = FailureReason.NONE
    message: str = ""
```

Validation rejects non-finite numeric values and duplicate joint names.

- [ ] **Step 4: Write RED config tests**

Test exact schema and environment expansion without allowing arbitrary dict access:

```python
from dexterous_robot.config.loader import load_local_asset_config


def test_local_asset_config_expands_environment(tmp_path, monkeypatch):
    monkeypatch.setenv("LEGACY_WAM_RUNTIME", "/tmp/wam.usda")
    p = tmp_path / "assets.yaml"
    p.write_text("wam_runtime: ${LEGACY_WAM_RUNTIME}\nl20_runtime: /tmp/l20.usda\n", encoding="utf-8")
    cfg = load_local_asset_config(p)
    assert cfg.wam_runtime.as_posix() == "/tmp/wam.usda"
```

- [ ] **Step 5: Implement frozen config models and loader**

Initial exact model:

```python
@dataclass(frozen=True)
class LocalAssetConfig:
    wam_runtime: Path
    l20_runtime: Path
```

`load_local_asset_config(path)` uses `yaml.safe_load`, expands `${VAR}` using `os.path.expandvars`, requires exact keys `wam_runtime` and `l20_runtime`, and raises `ConfigError` for missing/unexpanded values.

- [ ] **Step 6: Run GREEN and full M1-R0 regression**

```bash
pytest tests/unit/test_core_types.py tests/unit/test_config_loader.py tests/contract/test_repository_boundary.py -q
python tools/review/verify_repo_boundary.py
```

- [ ] **Step 7: Commit and seal M1-R1**

```bash
git add src tests
 git commit -m "feat: add immutable core and typed config"
```

Classification: `M1_R1_CORE_CONFIG_READY_FOR_REVIEW`.

---

### Task 3: M1-R2 Linker L20 Device Model and 16/20/21 Contracts

**Files:**
- Create: `src/dexterous_robot/devices/hands/linker_l20/types.py`
- Create: `src/dexterous_robot/devices/hands/linker_l20/mapping.py`
- Create: `src/dexterous_robot/devices/hands/linker_l20/protocol.py`
- Create: `src/dexterous_robot/devices/hands/linker_l20/model.py`
- Create: `configs/devices/hands/linker_l20.yaml`
- Create: `tests/contract/test_l20_types.py`
- Create: `tests/contract/test_l20_mapping.py`
- Create: `tests/contract/test_l20_protocol.py`
- Create: `tests/fixtures/l20_legacy_golden_vectors.json`

**Legacy authority to read only:**
- `scripts/phase2/l20_dynamic_types.py`
- `scripts/phase2/l20_active_mapper.py`
- `scripts/phase2/l20_official20_codec.py`
- `configs/phase2/l20_command_mapping_v1.json`
- `tests/phase2/test_l20_active_mapper.py`
- `tests/phase2/test_l20_official20_codec.py`

**Interfaces:**
- Produces: exact L20 device types and pure mapping/codec functions.
- Consumed by: ManipulatorSystem, Isaac topology routing, GraspLock controller.

- [ ] **Step 1: Generate fixed legacy golden vectors before production implementation**

Create `tests/fixtures/l20_legacy_golden_vectors.json` from the frozen legacy functions for at least these Active16 inputs:

```text
all_open              = yaw/roll 0.5, all flexion 0.0
all_min               = all 0.0
all_max               = all 1.0
mixed_direction_probe = deterministic values 0.05,0.10,...,0.80 in canonical Active16 order
```

Fixture stores the source legacy file SHA256 plus expected Physical21 radians for both `mujoco_equal_v1` and `urdf_mimic_v1`, and expected Protocol20 normalized slots + encoded bytes.

- [ ] **Step 2: Write tests that consume the fixture and fail before implementation**

Assertions include:

```text
16 Active names exact canonical order
21 Physical names exact canonical order
20 protocol slots exact width
protocol slots 11-14 always zero
reversed protocol slots = {0,1,2,3,4,5,10,15,16,17,18,19}
thumb_joint4 follower multiplier 1.0
index/middle/ring/little joint3 multiplier 1.06399 for urdf_mimic_v1
index/middle/ring/little joint3 multiplier 1.0 for mujoco_equal_v1
```

- [ ] **Step 3: Run RED**

```bash
pytest tests/contract/test_l20_types.py tests/contract/test_l20_mapping.py tests/contract/test_l20_protocol.py -q
```

- [ ] **Step 4: Port device types without legacy Phase naming**

Required public names remain semantically explicit:

```python
L20_ACTIVE_CHANNELS: tuple[str, ...]
L20_PHYSICAL_JOINTS: tuple[str, ...]

@dataclass(frozen=True)
class L20ActiveCommand16: ...

@dataclass(frozen=True)
class L20ProtocolCommand20: ...

@dataclass(frozen=True)
class L20PhysicalTarget21: ...

@dataclass(frozen=True)
class L20PhysicalState21: ...
```

Do not import from the legacy project at runtime.

- [ ] **Step 5: Port pure mapping and protocol codec**

Required public functions:

```python
def map_active_to_physical(
    command: L20ActiveCommand16,
    *,
    coupling_profile: str,
) -> L20PhysicalTarget21: ...


def adapt_active_to_protocol20(command: L20ActiveCommand16) -> L20ProtocolCommand20: ...


def encode_official20(command: L20ActiveCommand16) -> tuple[int, ...]: ...
```

M1 omits historical target-rate limiting from the public mapper unless required by a production controller; rate limiting belongs in controller/runtime, not device semantic mapping.

- [ ] **Step 6: Add `LinkerL20Model`**

```python
@dataclass(frozen=True)
class LinkerL20Model:
    device_id: str = "hand"
    active_channels: tuple[str, ...] = L20_ACTIVE_CHANNELS
    physical_joints: tuple[str, ...] = L20_PHYSICAL_JOINTS
    coupling_profile: str = "mujoco_equal_v1"
```

The Isaac M1 composition explicitly selects the profile verified by the R15U physical topology instead of relying on the class default.

- [ ] **Step 7: Run GREEN against legacy golden vectors**

```bash
pytest tests/contract/test_l20_types.py tests/contract/test_l20_mapping.py tests/contract/test_l20_protocol.py -q
```

Expected: bit/exact-value equality to fixture where the legacy contract is exact, floating comparison `abs <= 1e-12` for radian mapping.

- [ ] **Step 8: Commit and seal M1-R2**

Classification: `M1_R2_L20_DEVICE_CONTRACT_READY_FOR_REVIEW`.

---

### Task 4: M1-R3 Minimal Backend Contract, RuntimeSnapshot, and Runtime Session

**Files:**
- Create: `src/dexterous_robot/backends/base.py`
- Create: `src/dexterous_robot/runtime/snapshot.py`
- Create: `src/dexterous_robot/runtime/session.py`
- Create: `tests/contract/test_backend_contract.py`
- Create: `tests/contract/test_runtime_snapshot.py`
- Create: `tests/unit/test_runtime_session.py`

**Interfaces:**
- Consumes: core commands, Pose, JointState.
- Produces: `Backend`, `BackendState`, `RuntimeSnapshot`, `RuntimeSession`.
- Consumed by: Isaac backend and all Tasks/Skills.

- [ ] **Step 1: Write a fake backend test first**

```python
class FakeBackend(Backend):
    def __init__(self):
        self.applied = []
        self.steps = []
    def initialize(self): pass
    def reset(self): pass
    def read_state(self): return BackendState(device_states={}, body_poses={}, signals={})
    def apply(self, commands): self.applied.extend(commands)
    def step(self, dt_s): self.steps.append(dt_s)
    def shutdown(self): pass
```

Test that `RuntimeSession.cycle(commands)` orders calls as `apply -> step -> read_state`, advances time by exactly `dt_s`, and returns a frozen snapshot.

- [ ] **Step 2: Run RED**

```bash
pytest tests/contract/test_backend_contract.py tests/contract/test_runtime_snapshot.py tests/unit/test_runtime_session.py -q
```

- [ ] **Step 3: Implement exact minimal contract**

```python
@dataclass(frozen=True)
class BackendState:
    device_states: Mapping[str, JointState]
    body_poses: Mapping[str, Pose]
    signals: Mapping[str, float | int | bool | str | None]

class Backend(ABC):
    @abstractmethod
    def initialize(self) -> None: ...
    @abstractmethod
    def reset(self) -> None: ...
    @abstractmethod
    def read_state(self) -> BackendState: ...
    @abstractmethod
    def apply(self, commands: Sequence[JointPositionCommand | JointEffortCommand]) -> None: ...
    @abstractmethod
    def step(self, dt_s: float) -> None: ...
    @abstractmethod
    def shutdown(self) -> None: ...
```

`BackendState` defensively freezes mappings with `MappingProxyType`.

- [ ] **Step 4: Implement immutable RuntimeSnapshot**

```python
@dataclass(frozen=True)
class RuntimeSnapshot:
    time_s: float
    dt_s: float
    device_states: Mapping[str, JointState]
    body_poses: Mapping[str, Pose]
    signals: Mapping[str, float | int | bool | str | None]
```

- [ ] **Step 5: Implement RuntimeSession**

```python
class RuntimeSession:
    def __init__(self, backend: Backend, dt_s: float): ...
    def initialize(self) -> RuntimeSnapshot: ...
    def reset(self) -> RuntimeSnapshot: ...
    def cycle(self, commands: Sequence[Command]) -> RuntimeSnapshot: ...
    def shutdown(self) -> None: ...
```

Runtime is the only owner of session time. Task/Skill cannot call `backend.step()`.

- [ ] **Step 6: Run GREEN and regression**

```bash
pytest tests/contract tests/unit -q
```

- [ ] **Step 7: Commit and seal M1-R3**

Classification: `M1_R3_RUNTIME_BACKEND_CONTRACT_READY_FOR_REVIEW`.

---

### Task 5: M1-R4 WAM7 Device Model and ManipulatorSystem Composition

**Files:**
- Create: `src/dexterous_robot/devices/arms/wam7/model.py`
- Create: `src/dexterous_robot/robots/manipulator.py`
- Create: `configs/devices/arms/wam7.yaml`
- Create: `configs/robots/wam7_linker_l20.yaml`
- Create: `tests/contract/test_wam7_model.py`
- Create: `tests/contract/test_manipulator_system.py`

**Interfaces:**
- Produces: `Wam7Model`, `MountTransform`, `ManipulatorSystem`.
- Consumed by: Isaac topology mapper and controllers.

- [ ] **Step 1: Write RED tests for exact WAM7 logical names and non-action composition**

```python
EXPECTED = (
    "wam_j1_joint", "wam_j2_joint", "wam_j3_joint", "wam_j4_joint",
    "wam_j5_joint", "wam_j6_joint", "wam_j7_joint",
)


def test_wam7_joint_order_is_frozen():
    assert Wam7Model().joint_names == EXPECTED


def test_manipulator_system_does_not_expose_task_actions():
    robot = ManipulatorSystem(...)
    for forbidden in ("move_ee", "grasp", "close_hand", "lift"):
        assert not hasattr(robot, forbidden)
```

- [ ] **Step 2: Run RED**

```bash
pytest tests/contract/test_wam7_model.py tests/contract/test_manipulator_system.py -q
```

- [ ] **Step 3: Implement logical device and mount types**

```python
@dataclass(frozen=True)
class Wam7Model:
    device_id: str = "arm"
    joint_names: tuple[str, ...] = WAM7_JOINT_NAMES
    base_frame: str = "wam_base"
    flange_frame: str = "wam_j7"

@dataclass(frozen=True)
class MountTransform:
    parent_frame: str
    child_frame: str
    pose: Pose

@dataclass(frozen=True)
class ManipulatorSystem:
    system_id: str
    arm: Wam7Model
    hand: LinkerL20Model
    hand_mount: MountTransform
    tcp_frame: str
```

No action methods.

- [ ] **Step 4: Encode frozen M1 WAM7+L20 composition config**

The M1 mount uses the already-accepted legacy mount candidate values as configuration, not hard-coded production logic. The local asset paths remain in `configs/local/` and are not tracked.

- [ ] **Step 5: Run GREEN**

```bash
pytest tests/contract/test_wam7_model.py tests/contract/test_manipulator_system.py -q
```

- [ ] **Step 6: Commit and seal M1-R4**

Classification: `M1_R4_MANIPULATOR_COMPOSITION_READY_FOR_REVIEW`.

---

### Task 6: M1-R5 Minimal Isaac Backend with Combined-Articulation Routing and Correct Dynamic Transform Sync

**Files:**
- Create: `src/dexterous_robot/backends/isaac/topology.py`
- Create: `src/dexterous_robot/backends/isaac/scene.py`
- Create: `src/dexterous_robot/backends/isaac/contacts.py`
- Create: `src/dexterous_robot/backends/isaac/transform_sync.py`
- Create: `src/dexterous_robot/backends/isaac/backend.py`
- Create: `configs/backends/isaac.yaml`
- Create: `configs/tasks/tabletop_grasp_lift.yaml`
- Create: `tests/unit/test_isaac_topology.py`
- Create: `tests/integration/test_isaac_m1_load.py`
- Create: `tests/integration/test_isaac_transform_sync.py`

**Legacy authority to read only:**
- R15U runner/spec files and R15U sealed archive.
- WAM runtime: existing canonical runtime selected by local config.
- L20 runtime: existing dynamic L20 runtime selected by local config.

**Interfaces:**
- Implements: `Backend`.
- Hides: combined 28-DOF PhysX articulation and all USD/PhysX/Fabric details.
- Exposes: logical device states keyed `arm` and `hand`; body pose `object`; normalized signals such as table contact and opposing squeeze.

- [ ] **Step 1: Write pure RED tests for combined topology routing**

Given backend names `WAM7_JOINT_NAMES + L20_PHYSICAL_JOINTS`, assert:

```text
arm indices = 0..6
hand indices = 7..27
no duplicates
all 28 backend lanes consumed exactly once
```

`build_joint_routing(backend_joint_names, robot)` returns immutable name/index maps.

- [ ] **Step 2: Run topology RED and implement `topology.py`**

No Isaac imports are allowed in `topology.py`; it is pure mapping logic.

- [ ] **Step 3: Write Isaac integration smoke before backend implementation**

`test_isaac_m1_load.py` is marked `@pytest.mark.isaac` and runs only through Isaac Sim Python. It must assert:

```text
backend initializes
one combined articulation resolves
logical arm state width = 7
logical hand state width = 21
object pose is finite
```

- [ ] **Step 4: Implement lazy Isaac application/backend initialization**

`dexterous_robot.backends.isaac` must remain importable in normal Python without importing `omni` at module import time. Isaac imports occur inside backend initialization after `SimulationApp` exists.

`IsaacBackend` constructor consumes typed robot/task/backend/local-asset configs; it does not contain `/home/lyf/...` literals.

- [ ] **Step 5: Implement M1 scene creation/loading**

M1 reproduces only required behavior:

```text
load existing WAM canonical runtime
load/mount existing L20 runtime
resolve combined articulation
create table
create 0.05 x 0.05 x 0.065 m, 0.05 kg cuboid
static/dynamic friction = 1.0/1.0
```

Object/task geometry values come from `tabletop_grasp_lift.yaml`.

- [ ] **Step 6: Implement command routing**

For `JointPositionCommand(device_id="arm")`, route seven logical positions into WAM lanes. For `device_id="hand"`, require Physical21 joint order and route into hand lanes. Profile names are resolved through backend configuration, e.g. `arm_carry_position_drive` and `hand_grasp_lock`; backend-specific stiffness/damping/max-force live in Isaac config rather than controller code.

- [ ] **Step 7: Implement normalized state/contact readout**

`read_state()` returns:

```text
device_states["arm"]  -> JointState width 7
device_states["hand"] -> JointState width 21
body_poses["hand_tcp"]
body_poses["object"]
signals["object_table_normal_n"]
signals["opposing_y_squeeze_n"]
```

Raw contact report structures never leave the Isaac package.

- [ ] **Step 8: Implement R15U transform synchronization as a named policy**

`transform_sync.py` contains the production equivalent of the proven R15U fix:

1. initial rigid-body pose is loaded by PhysX;
2. seed the dynamic cuboid pose into a weaker/root layer before releasing stronger session transform opinions;
3. remove only session `xformOp:translate` and `xformOp:orient` opinions for the dynamic object, preserve scale/order;
4. call PhysX transform update with both fast-cache/Fabric and USD writeback enabled;
5. audit Tensor/direct-PhysX/composed-USD/Fabric pose agreement at configured checkpoints.

The policy is covered by `test_isaac_transform_sync.py`, which fails if the composed object position remains at its initial pose while PhysX changes by more than 1 mm.

- [ ] **Step 9: Run Isaac smoke + transform integration**

Via the one-click Isaac runner:

```text
pytest pure tests first
launch exactly one Isaac runtime
run integration load/reset/10 physics steps
record 4-way transform checkpoint
shutdown
```

Expected: READY, no physics motion task yet.

- [ ] **Step 10: Commit and seal M1-R5**

Classification: `M1_R5_ISAAC_BACKEND_READY_FOR_CONTROLLER_INTEGRATION`.

---

### Task 7: M1-R6 Pure WAM Kinematics, GraspLock, and CartesianCarry Controllers

**Files:**
- Create: `src/dexterous_robot/control/math/minimum_jerk.py`
- Create: `src/dexterous_robot/control/arm/kinematics.py`
- Create: `src/dexterous_robot/control/arm/cartesian_carry.py`
- Create: `src/dexterous_robot/control/hand/grasp_lock.py`
- Create: `tests/unit/test_minimum_jerk.py`
- Create: `tests/unit/test_wam7_kinematics.py`
- Create: `tests/unit/test_grasp_lock_controller.py`
- Create: `tests/unit/test_cartesian_carry_controller.py`

**Legacy authority to read only:**
- `scripts/control/minimum_jerk.py`
- `p2b2_wam_dynamic_l20_tabletop_demo_v3_numerical_ik_grasp_lift.py::solve_wam_ik`
- R15Q `build_locked_grasp_target`, Cartesian carry pose generation, and WAM pose IK wrapper.
- R15R/R15U position-drive carry behavior.

**Interfaces:**
- Produces backend-neutral `JointPositionCommand`s only.
- Consumed by Skills.

- [ ] **Step 1: Port minimum jerk with legacy-equivalence tests first**

Exact functions:

```python
def minimum_jerk_fraction(u: float) -> float: ...
def minimum_jerk_position(start: float, target: float, elapsed_s: float, duration_s: float) -> float: ...
```

Test legacy sample points and endpoint clamping.

- [ ] **Step 2: Freeze WAM FK/IK golden vectors before implementation**

From the legacy accepted numerical IK code, generate fixture vectors for the R15U lateral-ready, transit, pregrasp, grasp, and lift targets. Store target pose, seed q, solved q, position error, orientation error, and legacy source SHA256.

- [ ] **Step 3: Implement pure `Wam7Kinematics`**

Public contract:

```python
class Wam7Kinematics:
    def forward(self, q_rad: Sequence[float]) -> Pose: ...
    def solve_pose(self, target: Pose, seed_q_rad: Sequence[float]) -> tuple[float, ...]: ...
```

No Isaac calls. Match frozen legacy golden q within the established IK tolerance, not bit exact if the solver is iterative.

- [ ] **Step 4: Write GraspLock RED tests**

Given a base Physical21 target, assert controller builds exactly one locked target, adds the approved thumb/four-side trim, enforces follower relation, then returns the same target on subsequent compute calls.

Public contract:

```python
@dataclass(frozen=True)
class GraspLockGoal:
    base_target: L20PhysicalTarget21

class GraspLockController:
    def compute(self, goal: GraspLockGoal) -> JointPositionCommand: ...
```

Profile is semantic string `hand_grasp_lock`; force/stiffness numbers remain Isaac backend config.

- [ ] **Step 5: Implement GraspLock GREEN**

No online squeeze reshaping during carry; preserve the R15Q/R15U behavioral decision.

- [ ] **Step 6: Write CartesianCarry RED tests**

Public goal:

```python
@dataclass(frozen=True)
class CartesianCarryGoal:
    locked_tcp_pose: Pose
    delta_world_m: tuple[float, float, float]
    duration_s: float
```

Controller method:

```python
class CartesianCarryController:
    def compute(self, *, elapsed_s: float, current_q_rad: Sequence[float], goal: CartesianCarryGoal) -> JointPositionCommand: ...
```

Assert X/Y/quaternion remain fixed, world Z uses minimum jerk, and command profile is `arm_carry_position_drive`.

- [ ] **Step 7: Run controller GREEN**

```bash
pytest tests/unit/test_minimum_jerk.py tests/unit/test_wam7_kinematics.py tests/unit/test_grasp_lock_controller.py tests/unit/test_cartesian_carry_controller.py -q
```

- [ ] **Step 8: Commit and seal M1-R6**

Classification: `M1_R6_CONTROLLERS_READY_FOR_SKILLS`.

---

### Task 8: M1-R7 BT-Compatible Skills and TabletopGraspLiftTask

**Files:**
- Create: `src/dexterous_robot/skills/approach.py`
- Create: `src/dexterous_robot/skills/grasp.py`
- Create: `src/dexterous_robot/skills/lift.py`
- Create: `src/dexterous_robot/skills/hold.py`
- Create: `src/dexterous_robot/tasks/tabletop_grasp_lift.py`
- Create: `tests/unit/test_skill_contracts.py`
- Create: `tests/unit/test_tabletop_grasp_lift_task.py`

**Interfaces:**
- Consumes: `RuntimeSnapshot`, controllers, semantic signals.
- Produces: `SkillResult` and command batches for Runtime.

- [ ] **Step 1: Write a deterministic fake-snapshot sequence test**

The test drives the Task through:

```text
APPROACH -> GRASP -> LIFT -> HOLD -> SUCCESS
```

without any Isaac imports. It verifies Task sees only SkillStatus/FailureReason and does not inspect finger-specific PhysX metrics.

- [ ] **Step 2: Run RED**

```bash
pytest tests/unit/test_skill_contracts.py tests/unit/test_tabletop_grasp_lift_task.py -q
```

- [ ] **Step 3: Implement common Skill step result shape**

Each Skill exposes:

```python
def step(self, snapshot: RuntimeSnapshot) -> tuple[SkillResult, tuple[Command, ...]]: ...
```

No Skill calls Runtime or Backend directly.

- [ ] **Step 4: Implement M1 semantic criteria**

`GraspSkill` success: stable grasp-lock established according to semantic squeeze/contact state provided by backend normalization, not raw PhysX APIs.

`LiftSkill` failure: `OBJECT_SLIPPED` if object leaves the grasp according to task-level relative-pose/slip bounds; it does not secretly restart GraspSkill.

`HoldSkill` success: object remains off table for configured duration at least 0.5 s.

- [ ] **Step 5: Implement Task orchestration**

`TabletopGraspLiftTask` owns current phase and recovery policy. M1 policy is fail/abort on Skill failure; no retry tree yet. It never contains backend-name branches.

- [ ] **Step 6: Run GREEN and architecture scans**

```bash
pytest tests/unit/test_skill_contracts.py tests/unit/test_tabletop_grasp_lift_task.py -q
python tools/review/verify_repo_boundary.py
```

Additionally grep `src/dexterous_robot/{tasks,skills,control,core}` and fail if it imports `omni`, `isaacsim`, `mujoco`, `rclpy`, or legacy `scripts.phase2`.

- [ ] **Step 7: Commit and seal M1-R7**

Classification: `M1_R7_TASK_SKILLS_READY_FOR_GOLDEN_RUNTIME`.

---

### Task 9: M1-R8 Isaac Golden Demo, Evidence, and Freeze

**Files:**
- Create: `examples/isaac/tabletop_grasp_lift.py`
- Create: `tests/golden/test_tabletop_grasp_lift_acceptance.py`
- Create: `tools/review/finalize_m1_golden.py`
- Modify: `README.md`
- Modify: `configs/tasks/tabletop_grasp_lift.yaml`
- Modify: `configs/backends/isaac.yaml`

**Interfaces:**
- Consumes all prior M1 components.
- Produces the first production golden scenario and sealed M1 acceptance archive.

- [ ] **Step 1: Write acceptance finalizer tests before runtime execution**

Given synthetic summary JSON, finalizer PASS requires exactly:

```python
assert summary["wam_l20_loaded"] is True
assert summary["grasp_lock_success"] is True
assert summary["object_left_table"] is True
assert summary["cuboid_center_z_rise_m"] >= 0.025
assert summary["suspended_hold_s"] >= 0.5
assert summary["transform_consistency_pass"] is True
```

Any missing field is BLOCKED, not silently defaulted.

- [ ] **Step 2: Run RED then implement finalizer GREEN**

Classification on success: `M1_GOLDEN_WAM7_L20_ISAAC_TABLETOP_GRASP_LIFT_ACCEPTED`.

- [ ] **Step 3: Implement one formal example entry point**

`examples/isaac/tabletop_grasp_lift.py`:

1. loads typed config, including gitignored local asset config;
2. creates WAM7/L20 `ManipulatorSystem`;
3. creates `IsaacBackend` and `RuntimeSession`;
4. runs `TabletopGraspLiftTask` until SUCCESS/FAILURE/timeout;
5. records task summary + transform checkpoints + contact/pose metrics;
6. shuts down once;
7. never imports legacy code.

- [ ] **Step 4: First golden runtime attempt: no tuning outside existing R15U authority**

Use the R15U-approved M1 geometry and control profiles. If the clean architecture initially fails, compare new evidence against R15U and change only the module that owns the mismatch. Do not reintroduce R15 revision chaining in production code.

- [ ] **Step 5: Verify all six M1 hard gates**

Record:

```text
WAM7+L20 loaded
lateral grasp lock success
object leaves table
cuboid center Z rise
suspended hold duration
direct PhysX / composed USD / Fabric / viewport consistency
```

For transform consistency, checkpoints include at minimum pre-lift and post-lift; maximum position disagreement must remain below 1 mm unless a stricter backend-specific tolerance is established by tests.

- [ ] **Step 6: Run full non-Isaac suite plus one exact Isaac golden run**

```text
pytest unit/contract pure integration
repo boundary scan
py_compile/compileall
one Isaac runtime, no retry hidden inside runner
golden finalizer
```

- [ ] **Step 7: Update README with public-facing M1 usage**

README presents architecture and the single supported M1 example. It does not document historical R15 scripts as user APIs. Private asset setup is described via `configs/local/assets.yaml.example` without redistributing assets.

- [ ] **Step 8: Commit and freeze M1**

```bash
git add .
git commit -m "feat: accept WAM7 L20 Isaac tabletop grasp lift M1"
git tag m1-isaac-wam7-l20-grasp-lift
```

The sealed archive stores commit, tag, config hashes, relevant private asset SHA256 values, final task summary, transform checkpoints, and test logs.

---

## M1 Slice Delivery Contract

Every M1-R0 ... M1-R8 assistant delivery must provide one archive and one copyable shell block. The runner must:

```text
1. verify expected parent state / refuse destructive overwrite
2. apply complete slice
3. run focused pure tests
4. run runtime test only when required by that slice
5. collect review evidence
6. seal a tar.xz + SHA256
7. print the exact archive path to return
```

A slice must not require the user to copy individual `.py` files, edit YAML manually, or assemble logs.

## Legacy Reference Files Required on Demand

Do not upload the whole legacy asset tree. Request only the exact missing source/asset when implementation reaches it. Current known authorities already available from the source snapshot include:

```text
scripts/phase2/l20_dynamic_types.py
scripts/phase2/l20_active_mapper.py
scripts/phase2/l20_official20_codec.py
configs/phase2/l20_command_mapping_v1.json
scripts/control/minimum_jerk.py
scripts/phase2/p2b2_wam_dynamic_l20_tabletop_demo_v3_numerical_ik_grasp_lift.py
scripts/phase2/p2b2_wam_dynamic_l20_tabletop_demo_v3_r15q_grasp_lock_cartesian_orientation_lock_carry.py
scripts/phase2/run_p2b2_wam_dynamic_l20_tabletop_demo_v3_r15u_root_layer_dynamic_xform_seed.py
```

Private runtime assets remain on the user's machine and are referenced through `configs/local/`.

## Plan Self-Review

- Spec coverage: all 14 design invariants and all six M1 hard acceptance gates map to Tasks 1-9.
- Placeholder scan: no implementation placeholder language is used; deferred MuJoCo/Real/tactile work is explicitly outside M1, not an unfinished M1 step.
- Type consistency: Controllers produce `JointPositionCommand`; Skills produce `(SkillResult, commands)`; Runtime consumes commands and creates `RuntimeSnapshot`; Backend produces `BackendState`.
- Dependency direction: Core has no device/backend imports; L20 semantics remain device-local; Isaac details remain under `backends/isaac`; Task/Skill/Controller do not import Isaac.
- Scope: M1 is one sequential vertical slice with nine reviewable stages; later backends are separate milestones.
