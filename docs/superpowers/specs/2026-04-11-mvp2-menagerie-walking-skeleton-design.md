# NormaCore 仿真集成 MVP-2 — Menagerie Walking Skeleton

| | |
|---|---|
| **日期** | 2026-04-11 |
| **状态** | Draft — 待 review |
| **前置** | MVP-1 已完成并 merge 到本地 `main`（see `2026-04-10-simulation-integration-design.md`）|
| **范围** | 补完 MVP-1 的物理保真度，搭出未来 policy training 用的 sim 环境基础 |
| **驱动场景** | 无硬件开发 / 纯 sim 演示 / 个人研究 |
| **设计原则** | YAGNI + walking skeleton + fork 业界最佳实践（Menagerie）|

---

## 0. 背景：MVP-1 留下的物理债

MVP-1 完成了 Station + `sim-runtime` + `st3215-compat-bridge` + `norma_sim` 的端到端架构集成，143 个 tests 绿，merge 到本地 `main`（`ca48d4d`）。但 manual browser smoke test 暴露了**物理保真度债**：

- **M1（Shoulder Pitch）滑块拖动时基本不响应**——详见 `sim_starting_point.md` 的 "Smoke test 真实结果" 段落
- 根因链（从浅到深）：
  1. `forcerange = 2.94 N·m`（URDF `<limit effort="2.94"/>` 复制）和重力拉的 ~1.2 N·m 一抵消，PD 几乎没余量
  2. 尝试关重力发现 URDF `base_link` mesh **自碰撞**（5 个 `dist < 0` 的接触）
  3. 尝试加 passive joint damping 发现 **gripper primary joint 有效惯量 = 2.5e-7 kg·m²**（比 M1 小 5 个数量级，mass matrix 病态，数值即刻 NaN）
  4. 尝试修 gen.py 发现 `<default><joint damping=…/>` 被 append 到 MJCF 末尾（`<worldbody>` / `<actuator>` 之后），**MuJoCo 在解析 joint 时静默忽略了默认值**——这是一个 latent bug

**核心诊断**：MVP-1 的 URDF → MJCF pipeline 从根子上**漏了 `armature`**（真实 ST3215 servo 的 reducer reflected inertia）。URDF schema 不记录这个量，真实 ST3215 ~300:1 reducer 的反射惯量比 link 本身大 2-3 个数量级——没有 armature 的 MJCF 从物理上就是错的。

**MVP-1 的选择**：承认这是物理保真度问题而非单点 bug，把根因记录在案（`sim_starting_point.md`），不在 MVP-1 内修复，push 给 MVP-2。

---

## 1. TL;DR

**Fork Google DeepMind 的 [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) 里的 `trs_so_arm100`（SO-ARM100 的 hand-tuned MJCF）作为 ElRobot 的 sim 物理基线**。ElRobot 硬件本身就是 SO-ARM100 的 7+1 DOF 变体（README 注明 Handle_SO101 / Trigger_SO101 / Wrist_Roll_SO101 来自 SO-ARM100），Menagerie 的 SO-ARM100 模型的 `armature` / `damping` / `inertial` / `default class` / solver 配置直接可用，只需要 adapt 到 ElRobot 多出的 2-3 个关节。

**架构决策**：MJCF 成为 sim 的 single source of truth。URDF 降级为 ROS/MoveIt 兼容的 reference artifact。`gen.py` 的 URDF → MJCF pipeline **删除**。MVP-1 的 `world.yaml` schema 大幅简化为运行时 scene config。

**Walking skeleton 策略**：分两阶段，Phase 1 把 Menagerie 的 SO-ARM100 MJCF **零修改**地跑在 MVP-1 的全栈上（station + sim-runtime + norma_sim + bridge + web UI），证明 infra 是 robot-agnostic 的；Phase 2 才构造 ElRobot 的 8-joint MJCF。这个分阶段独立验证了两个假设：

- **假设 A**：MVP-1 的 Rust/IPC/bridge 架构对任何 MuJoCo MJCF 都能正常工作（Phase 1 测试）
- **假设 B**：Menagerie 的物理参数在 ElRobot 拓扑下依然成立（Phase 2 测试）

如果 Phase 1 失败，问题必然在 infra 的 ElRobot 硬编码——这反过来是 MVP-1 "bridge 抽象是否足够通用" 的 **延迟 acceptance test** 实打实兑现。

**非目标**：不做 gym/env wrapper、不做 policy training、不做真机集成、不做 leader arm、不做视觉/camera、不动 Rust 的 `sim-runtime` crate。

---

## 2. Goals / Non-Goals

### 2.1 Goals

1. **ElRobot follower 在 sim 中所有 8 个 motor 都能平滑响应滑块输入**——对应 acceptance Floor 的 6 条 + Ceiling 的 2 条（§3）
2. **MJCF 成为 sim 的 single source of truth**——删除 gen.py，架构上消除 URDF/MJCF dual-source-of-truth 问题
3. **保留 MVP-1 的全部架构成果**：sim-runtime / bridge / normfs / station integration 零改动
4. **建立永久的 Menagerie walking skeleton 回归 fixture**——Phase 1 的测试在 Phase 2 之后继续运行，作为 "infra is robot-agnostic" 的持续保证
5. **为未来 policy training 提供可用的物理基础**——不做 training 本身，但把物理层搞对，不给后续 env wrapper 埋坑

### 2.2 Non-Goals（明确不做）

**功能性 non-goals**：
- ❌ gym / gymnasium env wrapper（`reset()` / `step(action)` / `observation_space` / `action_space` 全部不碰）
- ❌ 引入 `lerobot` / `stable_baselines3` / `gymnasium` 依赖
- ❌ policy training、episode recording、dataset format、Hugging Face Hub
- ❌ 真 ElRobot 硬件测试 / serial bus / 真 `st3215` driver 改动
- ❌ shadow mode 专项测试（MVP-1 的 scenario C 继续工作但不作为 MVP-2 验收项）
- ❌ leader arm 的 sim（ElRobot leader/follower 不同 gear ratio，leader 是 teleop 模式的事）
- ❌ vision / camera / `usbvideo-compat-bridge`
- ❌ URDF 反向生成工具（`mjcf_to_urdf.py`）

**架构 non-goals**：
- ❌ 重构 Rust 的 `sim-runtime` / `st3215-wire` / `st3215-compat-bridge` / `station_iface` / `station` 任何代码
- ❌ 改 `world.proto` schema（`ActuationCommand` / `WorldSnapshot` / `ActuatorState` / capability kind 枚举全部不变）
- ❌ 加新 bridge crate（继续只有 `st3215-compat-bridge`，Menagerie 通过 preset 扩展）
- ❌ 改 normfs queue 协议 / data type 枚举
- ❌ 改 station 的 subsystem 结构 / inference aggregator
- ❌ 版本升级（唯一例外：若 `mujoco` python package 版本影响 Menagerie MJCF 加载，可升级）

### 2.3 Deferred（MVP-3+，按优先级）

| # | 工作项 | 触发条件 |
|---|---|---|
| 1 | Env wrapper + gym compatibility | 决定要训第一个 policy |
| 2 | Leader arm sim（teleop 数据生成） | MVP-3 env wrapper 确定要 teleop 路径 |
| 3 | Real hardware 集成测试（scenario C 真走） | 有物理机器人到手 |
| 4 | Camera / vision bridge（`usbvideo-compat-bridge`） | 训视觉 policy |
| 5 | MJCF → URDF 反向工具 | 仅 (3) 启动时才考虑 |
| 6 | Gravity compensation feedforward 控制层 | 强烈希望不用走到这一步；仅当 Menagerie 参数下 M1 仍然不灵敏时 fallback |

---

## 3. Acceptance Criteria

### 3.1 Floor — 客观，自动化，CI 可跑

1. **无自碰撞**：`mj_forward(model, data)` 后 `data.ncon == 0`
2. **有效惯量下限**：每个 DOF 满足 `M[i, i] + model.dof_armature[i] >= 1e-4 kg·m²`（意味着 M8 / M5 / M7 必须靠 armature 补足）
3. **数值稳定性**：10,000 步随机 `ctrl` 输入（每 100 步 resample `uniform(ctrlrange_lo, ctrlrange_hi)`）全程 `np.isfinite(data.qpos).all()`
4. **所有 8 个 motor 的阶跃响应**：对每个 motor，`ctrl = 0.9 × ctrlrange_hi`，2 秒内 `qpos` 到达目标 80%，overshoot ≤ 15%（`pytest.parametrize` 每个 motor 一个独立 test）
5. **P0 回归保留**：`test_mimic_gripper_equality_works` + `test_mimic_gripper_zero_setpoint_holds_zero` 继续过
6. **MVP-1 测试套全绿**：现有 143 tests 中 6-10 个因 schema 迁移被改写（§9.1），但总数不降反升，零 failure

### 3.2 Ceiling — 主观，人肉验证

7. **浏览器 scenario A smoke test**：启动 station + norma_sim 子进程，浏览器打开 `http://localhost:8889`，拖动每个 slider（M1-M8），**视觉上都能响应**（平滑、不飘、不振荡，释放后能保持位置而非下垂）。gripper 多 motor 同时驱动无干扰。Kill + restart 后 arm 回到 home pose。
8. **MuJoCo 原生 viewer side-by-side 对比**：`python -m mujoco.viewer hardware/elrobot/simulation/elrobot_follower.xml` 和 `python -m mujoco.viewer hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml` 同时打开，手动拖动 joint，视觉质感相当（没有"我们的明显更差"的感觉）

### 3.3 Floor 故意省略的

- ❌ "像 YouTube demo 视频（[youtu.be/WXRG1KnzKv4](https://youtu.be/WXRG1KnzKv4)）"——视频是真机的 free-form 动作，不是 step response，数值对比不上
- ❌ "物理上完美"——不做 gravity compensation、不做 friction 精细 calibration，只到"能用"
- ❌ "所有 8 motor 调参 identical"——armature 因 motor 位置不同而不同，接受异质参数

### 3.4 Exit criteria 综述

MVP-2 complete 的定义 =
- ✅ Phase 0 reconnaissance gate 通过（§6.1）
- ✅ Phase 1 walking skeleton 在浏览器里正常操作，Phase 1 测试套绿（§9.2）
- ✅ Phase 2 ElRobot MJCF 手写完成，所有 8 motor 滑块可操作
- ✅ Floor §3.1 的 6 条全部通过
- ✅ Ceiling §3.2 的 2 条手动确认
- ✅ 143 tests（加新测试后更多）全绿
- ✅ `make check-arch-invariants` 全过
- ✅ Rust clippy 零新增 warning（MVP-1 已清零）
- ✅ spec + plan commit 到 main
- ✅ `sim_starting_point.md` memory 更新反映 MVP-2 完成状态

---

## 4. 架构决策：URDF-first → MJCF-first

### 4.1 为什么是 MJCF-first

MVP-1 走的是 URDF-first：URDF 是 source of truth，`gen.py` 把它翻译成 MJCF。这在"参数都能在 URDF schema 里表达"的前提下合理，但对于仿真必须的 `armature` / `frictionloss` / `<default>` class hierarchy 等 MJCF-specific 构造，URDF 没地方放，只能在 gen.py 里硬编码或加 override 层。

**Menagerie 已经提供了 ElRobot 这类低成本 servo arm 的 hand-tuned MJCF**。fork 它比从 URDF 重新调参更快、更准确、更符合业界实践（lerobot / gym-lowcostrobot / 整个 Menagerie 生态都是 MJCF-first）。

### 4.2 URDF 不删，但降级

URDF 保留在 `hardware/elrobot/simulation/elrobot_follower.urdf`，作为 **ROS/MoveIt 兼容的 reference artifact**。头部 comment 明确标注：

```xml
<!-- This URDF is a reference artifact for ROS/MoveIt tooling.
     It is NOT the source of truth for the MuJoCo simulation.
     See elrobot_follower.xml for the sim model. -->
```

若将来真机集成路径启动（MVP-4+），需要手工或工具化对齐 URDF 和 MJCF。该工作明确 defer。

### 4.3 `gen.py` 删除

`hardware/elrobot/simulation/worlds/gen.py`（~400 行 URDF → MJCF 翻译逻辑）完全删除。`Makefile` 的 `regen-mjcf` target 同步删除。`elrobot_follower.xml`（之前在 `worlds/` 子目录）移到 `simulation/` 根目录下手写维护。

---

## 5. 文件布局

### 5.1 MVP-2 后的 `hardware/elrobot/simulation/` 结构

```
hardware/elrobot/simulation/
├── elrobot_follower.urdf              # 保留，降级 reference artifact
├── elrobot_follower.xml                # 新：手写 MJCF（Phase 2 产出）
├── elrobot_follower.scene.yaml        # 新：大幅简化的 scene config
├── assets/                             # 不变，STL meshes
│   ├── base_link.stl
│   └── ... (所有现有 STL)
├── vendor/                             # 新
│   └── menagerie/
│       ├── VENDOR.md                  # 源 URL + commit hash + 拉取日期 + 修改说明
│       ├── LICENSE                     # Menagerie 的 Apache 2.0 副本
│       └── trs_so_arm100/
│           ├── scene.xml              # Menagerie 原文件，未修改
│           ├── trs_so_arm100.xml
│           └── assets/                # Menagerie 自己的 meshes
                                        # worlds/ 整个目录删除（见 §5.2）
```

### 5.2 删除的文件

```
hardware/elrobot/simulation/worlds/gen.py                      # ~400 行
hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml # 迁移到 simulation/ 根
hardware/elrobot/simulation/worlds/elrobot_follower.xml        # 迁移到 simulation/ 根
```

### 5.3 Phase 1 临时（+永久 regression fixture）文件

以下文件在 Phase 1 新增，**Phase 2 结束后保留作为永久 walking skeleton 回归 fixture**：

```
hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml
software/sim-bridges/st3215-compat-bridge/src/menagerie_so_arm100.preset.yaml
software/station/bin/station/station-sim-menagerie.yaml
software/sim-server/tests/integration/test_menagerie_walking_skeleton.py
```

### 5.4 新增的 Station scenario

```
software/station/bin/station/
├── station-sim.yaml                 # MVP-1 scenario A，Phase 2 后指向 ElRobot scene yaml
├── station-sim-external.yaml        # MVP-1 scenario B，不变
├── station-shadow.yaml              # MVP-1 scenario C，不变
└── station-sim-menagerie.yaml       # 新，Phase 1 的 walking skeleton 入口
```

---

## 6. Phase 结构

### 6.1 Phase 0 — Reconnaissance + Vendor（前置 gate）

**目的**：在 spec review 之前验证 Menagerie 假设成立，避免后续工作建立在幻想的前提上。

**步骤**：

1. `git clone https://github.com/google-deepmind/mujoco_menagerie /tmp/menagerie`
2. 确认存在 `trs_so_arm100/` 或等价目录
3. 读 `trs_so_arm100/trs_so_arm100.xml` 和 `scene.xml`，确认：
   - MJCF 包含 `<default>` class、`<position>` actuator、tendon/equality（若有 gripper mimic）
   - joint 数量 5-6，和我们的预期一致
   - `armature` / `damping` 属性存在且数值合理（非零）
   - mesh path 指向 `trs_so_arm100/assets/*.stl`（或类似）
4. 检查 `LICENSE` 文件确认是 Apache 2.0（或其他 permissive license）
5. **vendor 文件**到 `hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/`，包含 scene.xml / trs_so_arm100.xml / assets/ / LICENSE
6. 写 `VENDOR.md` 记录：源 URL、clone commit SHA、日期、"未修改，仅作为参考"

**Phase 0 Gate**：以上 6 步全通过才能进 spec review。若 (1)(2)(4) 不成立（没 SO-ARM100、license 不兼容），spec 废弃重写，换 fallback 方案（看 `gym-lowcostrobot` 或其他 Menagerie arm 如 `trossen`/`franka_panda`）。

**note**：Phase 0 **不写代码、不改 MVP-1 的任何文件**，只是 vendor 操作 + 验证。完成耗时 < 30 分钟。

### 6.2 Phase 1 — Walking Skeleton（Menagerie 原生跑栈）

**目的**：证明 MVP-1 的 infra（Rust + Python + bridge + web UI）对 Menagerie SO-ARM100 原生 MJCF 是 robot-agnostic 的。

**步骤**（每步对应一个 implementation task，写进 plan 时拆成小 chunk）：

1. **写 `menagerie_so_arm100.scene.yaml`**：新 schema（§8.1），`mjcf_path` 指向 vendor 的 `scene.xml`，`actuator_annotations` 声明 Menagerie 的 gripper（若有）
2. **写 `menagerie_so_arm100.preset.yaml`**（`st3215-compat-bridge` 的 preset）：把 Menagerie 的 joint 名（`shoulder_pan_joint` 等）映射到 fake `motor_id = 1..N`、`actuator_id = rev_motor_01..N`（逻辑映射，不改 bridge 代码）
3. **写 `station-sim-menagerie.yaml`**：station 的 sim scenario yaml，`sim_runtime` 节点的 `world_manifest` 指向 `menagerie_so_arm100.scene.yaml`，`st3215_compat_bridge` 节点的 `preset` 指向 `menagerie_so_arm100.preset.yaml`
4. **改 `norma_sim.world.manifest` + `descriptor`**：接受新 schema，actuator 列表从 MJCF 推断而非 yaml 读（§8.2）
5. **跑 `python -m mujoco.viewer` 验证 Menagerie 原生可加载**（视觉基线）
6. **跑 `./target/debug/station -c station-sim-menagerie.yaml --web 0.0.0.0:8889`**
7. **浏览器打开 `http://localhost:8889`**，期望看到 Menagerie 的 N 个 motor slider 展示（非 ElRobot 8 个）
8. **拖每个 slider，视觉验证响应**
9. **MuJoCo viewer 和浏览器 side-by-side 对比**，视觉质感应该一致
10. **写 `test_menagerie_walking_skeleton.py`**（§9.2）：Phase 1 的自动化 gate，包含 no-collision / no-NaN / 所有 actuator 可驱动

**Phase 1 Gate**：步骤 5-10 全通过。如果浏览器里 Menagerie 看起来和 MuJoCo 原生 viewer 一致，**infra 假设 A 通过**。Phase 2 才可以开始。

**如果 Phase 1 失败**：停下来 debug infra 里的 ElRobot 硬编码假设。常见嫌疑：
- `norma_sim.world.manifest` 的 schema 校验过严（要求 ElRobot 特定字段）
- `norma_sim.world.capabilities` 的 REVOLUTE_POSITION 推断逻辑 hardcoded
- `st3215-compat-bridge` 的 preset loader 校验过严
- `st3215-wire` 的 pack 函数假设 8 motor

这些 bug 被 Phase 1 发现**恰好是 Phase 1 的价值**——它们是 MVP-1 隐藏的 ElRobot 耦合。

### 6.3 Phase 2 — ElRobot Adaptation

**目的**：基于 Menagerie 走通的 walking skeleton，构造 ElRobot 8-joint 的 MJCF。

**步骤**：

1. **通读 vendor 的 `trs_so_arm100.xml`**，产出 joint 映射表（§7）
2. **手写 `hardware/elrobot/simulation/elrobot_follower.xml`**：
   - 从 Menagerie 批发 `<option>` / `<default>` class / actuator 默认值
   - 用 ElRobot URDF 的 kinematics（pos / axis / mesh reference）构造 `<body>` 树
   - 6 个共享关节：直接继承 Menagerie 的 armature / damping / frictionloss
   - 2-3 个 ElRobot 独有关节（预计是 shoulder roll / shoulder yaw / wrist yaw 里的某几个）：nearest-neighbor 估值
   - collision geometry 简化：visual 用 STL，collision 用 primitive（box / sphere / capsule）
   - 保留 MVP-1 的 tendon-based gripper mimic 实现（或 adopt Menagerie 若兼容）
3. **写 `elrobot_follower.scene.yaml`**（§8.1 新 schema），actuator_annotations 声明 gripper
4. **更新 `station-sim.yaml`** 指向新 scene yaml（从 Phase 1 的 Menagerie 切回 ElRobot）
5. **恢复 `st3215-compat-bridge` 的 `elrobot.preset.yaml`** 路径（Phase 1 期间用的是 Menagerie preset）
6. **跑 Floor §3.1 的 6 条自动化测试**，迭代调参直到通过
7. **跑 Ceiling §3.2 的浏览器 smoke test**
8. **写 Menagerie 对照表**作为 MVP-2 成果文档（`docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md`）

**Phase 2 Gate**：Floor + Ceiling 全过 = MVP-2 done。

### 6.4 Phase 间的依赖链

```
Phase 0 ──► Phase 1 ──► Phase 2
         │        │
         │        └─► 暴露 infra ElRobot 耦合 bug（若有）
         │
         └─► reconnaissance 若失败，spec 重写
```

Phase 不能跳步：Phase 1 不过不能开 Phase 2，Phase 0 gate 不过不能写 implementation plan。

---

## 7. Fork-and-Adapt 策略

### 7.1 ElRobot vs Menagerie SO-ARM100 拓扑对比

**ElRobot**（从我们自己的 URDF 读出）: 7 revolute + 1 gripper = 8 actuators

```
M1 Shoulder Pitch    (肩部俯仰)
M2 Shoulder Roll     (肩部侧摆)
M3 Shoulder Yaw      (肩部偏航)  ◄ 3-DOF spherical shoulder
M4 Elbow             (肘部)
M5 Wrist Roll        (腕部旋转)
M6 Wrist Pitch       (腕部俯仰)
M7 Wrist Yaw         (腕部偏航) ◄ 3-DOF spherical wrist
M8 Gripper           (夹爪，with 2 mimic joints via tendon)
```

**Menagerie SO-ARM100**（假设，Phase 0 验证）：典型 5-6 joints

```
shoulder_pan / shoulder_lift / elbow / wrist_flex / wrist_roll / gripper
```

### 7.2 Joint 映射表（Phase 2 第一个 task 产出）

此表初始为空，Phase 2 step 1 读完 Menagerie MJCF 后填充：

| ElRobot joint | Menagerie 对应 | armature | damping | frictionloss | 参数来源 |
|---|---|---|---|---|---|
| M1 Shoulder Pitch | ? | TBD | TBD | TBD | TBD |
| M2 Shoulder Roll | **无对应** | 估（用 M1） | 估 | 估 | nearest-neighbor |
| M3 Shoulder Yaw | ? | TBD | TBD | TBD | TBD |
| M4 Elbow | ≈ elbow | TBD | TBD | TBD | menagerie |
| M5 Wrist Roll | ≈ wrist_roll | TBD | TBD | TBD | menagerie |
| M6 Wrist Pitch | ≈ wrist_flex | TBD | TBD | TBD | menagerie |
| M7 Wrist Yaw | **无对应** | 估（用 M6） | 估 | 估 | nearest-neighbor |
| M8 Gripper | ≈ gripper | TBD | TBD | TBD | menagerie |

此表最终版本作为**独立 artifact**提交到 `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md`，这是 Phase 2 的成果文档之一。

### 7.3 批发继承的东西（file-level batch copy）

1. **`<option>` 块**：`timestep` / `iterations` / `integrator` / `solver` 全部照搬
2. **`<default>` 类继承**：Menagerie 的 class 定义（通常有 `arm_link` / `finger` / `visual` / `collision`）→ 沿用名字 + 属性
3. **Collision geometry 简化策略**：visual 用 STL mesh，collision 用 primitive（解决 MVP-1 的自碰撞 bug）
4. **Gripper tendon / equality 实现**：若 Menagerie 的 gripper 也用 tendon-based mimic → 沿用；若不同（e.g., `<weld>`）→ 保留 MVP-1 的 tendon 实现，不为对齐 Menagerie 破坏 P0
5. **Actuator `<position>` 默认 `kp` / `kv`**：继承 Menagerie 的（在有 armature 前提下，他们的值才是 ground truth）

### 7.4 必须自己构造的东西

1. **2-3 个 ElRobot 独有 joint**（M2 / M3 / M7 里的某几个）的 `armature` / `damping`：nearest-neighbor 估值，spec comment 标注 "estimated, subject to tuning"
2. **kinematic 链的几何**（`<body><geom>` 树结构、joint axis、pos/quat）：从 ElRobot URDF 读（这是确定性信息）
3. **Mesh 路径**：全部指向 `../assets/*.stl`（ElRobot 自己的 STL，不用 Menagerie 的）

### 7.5 Risk register

| Risk | Mitigation |
|---|---|
| Menagerie 的 joint limit 比 ElRobot 更保守 | 以 URDF 为准（hardware truth），armature / damping 用 Menagerie 值 |
| Menagerie 的 kp/kv 在 ElRobot 拓扑下太温和，Floor 4 过不了 | 调 kp 不调 armature（armature 是物理真实值） |
| ElRobot 独有 joint 估值不准 | 预期的 manual tuning loop；spec 定义 iteration budget（最多 5 轮） |
| Menagerie 的 gripper 用了 `<weld>` / 其他不兼容构造 | 保留 MVP-1 的 tendon 实现，不强行对齐 |

---

## 8. norma_sim Schema 变更

### 8.1 `scene.yaml` 新 schema

**MVP-1 现状** `world.yaml`（混合 gen.py manifest + actuator 元数据 + scene config）：

```yaml
world_name: elrobot_follower_empty
urdf_source: ../elrobot_follower.urdf        # 删除（无 gen.py）
mjcf_output: ./elrobot_follower.xml          # 改名
scene: {timestep, gravity, integrator, solver, iterations}
scene_extras: {lights, floor}
robots:
  - robot_id: ...
    actuators: [... 8 条 ...]
    sensors: [...]
```

**MVP-2 目标** `elrobot_follower.scene.yaml`：

```yaml
world_name: elrobot_follower               # 保留，用于 display
mjcf_path: ./elrobot_follower.xml          # load 路径而非 write 路径

# scene_overrides optional; 省略则用 MJCF 自己的 <option>
scene_overrides:
  gravity: [0, 0, -9.81]                   # demo 场景允许 runtime 覆盖

# scene_extras optional; 通常为空（Menagerie MJCF 已有 floor/lights）
scene_extras: {}

# 只为 "非默认 capability" 的 actuator 写 annotation。
# 默认 REVOLUTE_POSITION 从 MJCF 自动推断。
actuator_annotations:
  - mjcf_actuator: act_motor_08
    actuator_id: rev_motor_08
    display_name: Gripper
    capability:
      kind: GRIPPER_PARALLEL
      normalized_range: [0.0, 1.0]
    gripper:
      primary_joint_range_rad: [0.0, 2.2028]
      mimic_joints:
        - {joint: rev_motor_08_1, multiplier: -0.0115}
        - {joint: rev_motor_08_2, multiplier: 0.0115}
```

**变化要点**：
1. `actuators` 列表从 8 条变成 0 条（revolute 全部从 MJCF 自动推断）
2. `actuator_annotations` 只列 GRIPPER_PARALLEL（ElRobot/Menagerie 各 1 条）
3. `sensors` 列表删除（MJCF 的 `<sensor>` 自动枚举）
4. `scene.*` → `scene_overrides.*`（表明是覆盖 MJCF `<option>`）

### 8.2 `norma_sim` 代码 touched modules

| 模块 | 变化 | 原因 |
|---|---|---|
| `norma_sim.world.manifest` | **重写 schema parsing** | 新 yaml 结构 |
| `norma_sim.world.model` | **新增** `list_actuator_names()` / `list_sensor_names()` | MJCF 枚举 |
| `norma_sim.world.descriptor` | **重写** 构造逻辑 | 从 MJCF + annotations 而非 yaml |
| `norma_sim.world.capabilities` | **扩展** capability 推断 | REVOLUTE_POSITION 默认 + annotation override |
| `norma_sim.world.actuation` | **不动** | 只依赖 descriptor，已抽象 |
| `norma_sim.world.snapshot` | **不动** | 同上 |
| `norma_sim.world._proto` | **不动** | 不依赖 manifest schema |
| `norma_sim.ipc.*` | **不动** | 与 world 无关 |
| `norma_sim.cli` | **改 `--config` 参数类型** | 接受新 scene.yaml |

**`source_hash` check 整体退役**：`gen.py` 写入的 sha256 注释 + `manifest.py` 的验证逻辑全部删除，相关 tests 删除。

### 8.3 Rust 侧：零改动 commitment

```
software/sim-runtime/*                         (28 tests, 不动)
software/drivers/st3215-wire/*                 (15 tests, 不动)
software/sim-bridges/st3215-compat-bridge/*    (16 tests, 不动*)
software/drivers/st3215/*                      (真 driver, 不动)
software/station/shared/station-iface/*        (14 tests, 不动)
software/station/bin/station/*                 (lib + 2 integration, 不动)
```

**注 \***：`st3215-compat-bridge` 的代码不动，但**新增 preset yaml**（`menagerie_so_arm100.preset.yaml`）。preset 是数据文件不是代码。

---

## 9. Testing Strategy

### 9.1 MVP-1 遗产测试分类

**零改动保留**：
- Rust 所有 (72 + 16 = 88 tests)
- Python `norma_sim.ipc.*` (13 tests)
- Python `test_mimic_gripper.py` (2 tests, **P0 不可破**)
- Python `test_model.py` (5 tests)
- Python `test_snapshot.py` (4 tests)
- Python `test_actuation.py` (5 tests)
- Python `test_full_loop.py` (3 tests; fixture yaml 需 regenerate，逻辑不变)

**必须改（broken by schema migration）**：
- `test_manifest_load.py::test_source_hash_*` (3 tests) — **删**
- `test_manifest_load.py::test_manifest_load_happy` — **重写**
- `test_manifest_load.py::test_manifest_scene_config` — **改**
- `test_manifest_load.py::test_manifest_missing_gripper_fields_raises` — **改**
- `test_descriptor_build.py::test_build_world_descriptor_encodes` — **改**
- `test_capabilities.py::*` — 可能要调 1-2 个
- `tests/conftest.py::world_yaml_path` / `mjcf_path` fixture — **改指向**

### 9.2 新增 unit tests

- `test_model_enumerate.py` (新文件): MJCF actuator enumeration for both ElRobot and Menagerie
- `test_capabilities.py` (扩展): REVOLUTE_POSITION inferred from MJCF `<position>`; gripper annotation override; missing annotation target raises
- `test_manifest_load.py` (重写): new schema parsing, scene_overrides, annotation validation

### 9.3 Phase 1 — Menagerie walking skeleton integration test

新文件 `tests/integration/test_menagerie_walking_skeleton.py`:

```python
"""Walking skeleton: prove norma_sim infra works with Menagerie SO-ARM100
verbatim. Baseline for assumption A ("infra is robot-agnostic").

MUST remain green indefinitely — if this breaks, infra has regressed even
if ElRobot still works."""

def test_menagerie_mjcf_loads(menagerie_scene_yaml): ...
def test_menagerie_no_self_collision(menagerie_scene_yaml): ...    # Floor 1 analog
def test_menagerie_step_response(menagerie_scene_yaml): ...        # loose version
def test_menagerie_stress_10000_steps_no_nan(menagerie_scene_yaml): ...  # Floor 3 analog
```

**注意**：Menagerie 测试只跑 Floor 1 + 3 的 analog，不跑 Floor 2（有效惯量）和 Floor 4（step response 严格阈值）。Floor 2 / 4 是 ElRobot 专属目标，Menagerie 的调参过不过不影响我们。

### 9.4 Phase 2 — ElRobot acceptance integration test

新文件 `tests/integration/test_elrobot_acceptance.py`，实现 Floor §3.1 的 6 条：

```python
"""MVP-2 Phase 2 acceptance: 6 Floor criteria for ElRobot smoothness.
Definition of done. If any fail, MVP-2 is not finished."""

def test_elrobot_no_self_collision(elrobot_scene_yaml): ...       # Floor 1
def test_elrobot_effective_inertia_floor(elrobot_scene_yaml): ... # Floor 2
def test_elrobot_stress_10000_steps_no_nan(elrobot_scene_yaml): ...  # Floor 3

@pytest.mark.parametrize("motor_idx", range(8))
def test_elrobot_motor_step_response(elrobot_scene_yaml, motor_idx): ...  # Floor 4

# Floor 5 / 6 没有新代码（前者是 test_mimic_gripper.py 继续过，
# 后者是整体 143 tests 集合的约束）
```

**Floor 4 的 `parametrize`**：每个 motor 一个独立测试，失败时直接告诉你"M5 Wrist Roll 过不了"，不需要挖 log。

### 9.5 Manual ceiling procedure

扩展 `software/sim-server/README.md` 的 scenario A 清单，添加 MVP-2 验收 checklist：

```
Phase 2 acceptance (manual, MVP-2):

- [ ] Start: ./target/debug/station -c station-sim.yaml --web 0.0.0.0:8889
- [ ] Browser: http://localhost:8889, 看到 ElRobot 8 motors populate
- [ ] Drag M1 (Shoulder Pitch) slowly through full range
      - 平滑响应，无 oscillation，无 jitter
      - 释放后保持位置（不下垂）
- [ ] Repeat for M2..M7
- [ ] Drag M8 (Gripper): mimic joints open/close, no NaN
- [ ] Multi-motor: M1 + M4 + M8 同时拖，无干扰
- [ ] Kill station, restart, 确认 arm 回到 home pose

Side-by-side with MuJoCo native viewer:

- [ ] Terminal 1: python -m mujoco.viewer hardware/elrobot/simulation/elrobot_follower.xml
- [ ] Terminal 2: python -m mujoco.viewer hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml
- [ ] 手动拖动两边的 joint
- [ ] 预期：ElRobot 的响应质量 ≈ Menagerie SO-ARM100 质量
```

### 9.6 CI integration

`make sim-test` 自动 collect 新测试：

```makefile
sim-test: check-arch-invariants
	cargo test -p st3215-wire
	cargo test -p sim-runtime
	cargo test -p st3215-compat-bridge
	PYTHONPATH=$(SIM_PYTHONPATH) python3 -m pytest software/sim-server/tests/
	# 新: Menagerie walking skeleton + ElRobot acceptance 自动被 pytest collect
```

**Phase 1 walking skeleton 在 Phase 2 完成后继续运行**——作为 "infra robot-agnostic" 的永久回归保证。

---

## 10. Risks

（§7.5 已列 Phase 2 fork-specific risks，此处列总体风险）

| Risk | 影响 | Mitigation |
|---|---|---|
| **Menagerie 没有 SO-ARM100** 或结构不符预期 | Phase 0 reconnaissance gate 不过，整个 MVP-2 方案失效 | Phase 0 第一步就 clone 验证；fallback 查 `gym-lowcostrobot` 或其他低成本 arm |
| **第 8 个 motor 参数估值不准**，Floor 4 某 motor 过不了 | Phase 2 的 tuning loop 拉长 | spec 定义 tuning iteration budget（最多 5 轮），超过就 escalate；最坏情况放宽 Floor 4 tolerance（spec amendment） |
| **Phase 1 暴露的 infra bug 比预期多** | Phase 1 工作量超 1-2 天 | 这是好事不是坏事；延长 Phase 1 比硬 push 好；每个 bug 都是 MVP-1 "bridge 通用性"延迟 acceptance 的 data point |
| **Menagerie license 比预期严** | 不能 vendor，要 degrade 到 "research reference only" | Apache 2.0 是预期；非 Apache 则 spec revise |
| **Browser empty state 重现**（MVP-1 bug `84ca47a`） | Phase 1 浏览器验证卡住 | 修复已在 main；若重现说明 Menagerie preset 路径有特殊 bug，debug 根因而非 workaround |
| **`mujoco` python package 版本问题**导致 Menagerie MJCF 加载失败 | Phase 1 步骤 1 失败 | 允许升级 `mujoco`（此为 non-goals 的唯一例外） |

---

## 11. 参考资料

- **MVP-1 spec**: `docs/superpowers/specs/2026-04-10-simulation-integration-design.md`
- **MVP-1 plan**: `docs/superpowers/plans/2026-04-10-simulation-integration-mvp1.md`
- **MVP-1 memory**: `~/.claude/projects/-home-yuan-proj-norma-core/memory/sim_starting_point.md`
- **Git topology**: `~/.claude/projects/-home-yuan-proj-norma-core/memory/git_topology.md`（norma-core 是外部 repo 的本地 fork，main 不 push）
- **MuJoCo Menagerie**: https://github.com/google-deepmind/mujoco_menagerie
- **ElRobot project README**: `hardware/elrobot/README.md`
- **ElRobot demo video**: https://youtu.be/WXRG1KnzKv4
- **ElRobot web playground**: https://normacore.dev/elrobot-urdf/
- **SO-ARM100 origin**: https://github.com/TheRobotStudio/SO-ARM100
- **Codex session for architecture consult**（可选）: `019d7726-6dcf-7fe2-8887-35ee3b9c2568`

---

## Appendix A — 5 个 Design Section 的 brainstorming 流程记录

此 spec 由 `superpowers:brainstorming` skill 驱动的 5-section iterative design 过程产出。每 section 用户给 ✓ 后进入下一 section。关键 midway correction：

- **Section 2 一次重写**：原方案是单阶段 Menagerie fork，用户提出 "先跑 Menagerie 原生再 adapt ElRobot" 的 walking skeleton 思路。我采纳并重写 Section 2 为 Phase 0/1/2。此次 correction 显著降低了 MVP-2 的技术风险，并把 "infra 是 robot-agnostic" 从隐含假设变成可独立验证的 Phase 1 gate。

## Appendix B — Definition of Done 整合清单

从 §3.4 展开，单独列出便于 tracking：

- [ ] Phase 0 reconnaissance gate 通过（§6.1）
- [ ] vendor Menagerie 到 `hardware/elrobot/simulation/vendor/menagerie/` + VENDOR.md + LICENSE
- [ ] `menagerie_so_arm100.scene.yaml` 写完
- [ ] `menagerie_so_arm100.preset.yaml` 写完
- [ ] `station-sim-menagerie.yaml` 写完
- [ ] `norma_sim.world.manifest` / `descriptor` / `capabilities` 适配新 schema
- [ ] Phase 1 smoke test：Menagerie 在浏览器里跑通
- [ ] `test_menagerie_walking_skeleton.py` 绿（永久回归 fixture）
- [ ] `elrobot_follower.xml` 手写完成
- [ ] `elrobot_follower.scene.yaml` 写完
- [ ] joint 映射表（comparison table）作为独立 spec artifact 提交
- [ ] Phase 2 smoke test：ElRobot 8 个 motor 在浏览器里全部正常响应
- [ ] Floor §3.1 的 6 条自动化测试全绿
- [ ] Ceiling §3.2 的 2 条手动 check 完成
- [ ] Rust 143 tests 全过（含新 Python tests 后总数更高）
- [ ] `make check-arch-invariants` 全过
- [ ] Rust clippy 零新增 warning
- [ ] `sim-server/README.md` 的 manual checklist 更新
- [ ] `sim_starting_point.md` 记录 MVP-2 完成状态
- [ ] spec + plan + comparison table commit 到 main（不 push）
