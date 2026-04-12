# NormaCore Control Layer Architecture Decision (v2)

> sim-real 统一控制层：RCS vs ROS2 vs NormaCore 现状

**Date**: 2026-04-12
**Status**: PROPOSED (v2 — incorporates Codex adversarial review)
**Context**: NormaCore 需要一个 Gymnasium 兼容的控制接口用于 imitation learning 训练。当前有 Station + sim-bridge 体系，现在要决定在哪一层加 Gymnasium adapter、需要哪些底层改动、以及分几期做。

**v1 → v2 变更说明**: v1 声称"纯 Python adapter，不动 Rust"，经 Codex 代码级审查后确认此声明不成立。v2 诚实标注了每项工作的实际依赖和改动范围。

---

## 1. 三条路线概览

### Option A: RCS 路线 (Gymnasium Wrapper)

**核心模式**: wrapper 堆叠构建 env

```
SimEnv / HardwareEnv (base)
  → RobotWrapper (joint/cartesian control)
    → GripperWrapper (normalized [0,1])
      → CameraSetWrapper (RGB + depth)
        → RelativeActionSpace (movement limits)
          → CoverWrapper (top, handles reset ordering)
```

**关键设计**:
- `BaseEnv` 分两子类：`SimEnv(MuJoCo)` / `HardwareEnv`
- `RobotWrapper` 统一 action/observation 空间，全程弧度 (radians)
- `GripperConfig.min_actuator_width = -0.17453` 直接用 ctrlrange 弧度值
- Extension 模式：每个机器人是独立 pip 包 (`rcs_so101`)
- SO101 硬件通过 LeRobot 库通信，`[-100,100]` 步数 ↔ 弧度互转
- IK/FK 用 Pinocchio (C++ pybind11)
- 单进程架构，无 IPC
- DigitalTwin wrapper 可在同一进程里并行跑 sim 和 hw

**优势**: ML 训练原生集成；接口极简；sim-real 对称好；物理单位贯穿
**劣势**: Python 单进程 GIL 绑定；无持久化/遥测/web UI；无 IPC 层；C++ core 需替换为 Rust

### Option B: ROS2 路线 (ros2_control)

**核心模式**: HardwareInterface + Controller Manager

```
Controller Manager (single node, owns RT loop)
  → ResourceManager (tracks state/command interfaces)
    → SystemInterface / ActuatorInterface / SensorInterface (pluginlib)
  → Controllers (pluginlib, exclusive interface claiming)
    → joint_trajectory_controller, forward_command_controller, etc.
```

**关键设计**:
- `read()` → controllers `update()` → `write()` 循环
- state_interfaces 共享读，command_interfaces 独占写
- Sim-to-real 通过替换 URDF `<hardware><plugin>` 标签
- 内循环避开 DDS（共享内存），外层监控走 DDS topics
- 30+ 现成 controllers，MoveIt/Nav2 生态

**优势**: 工业标准；RT-PREEMPT 下 2μs 延迟；多进程；lifecycle 管理有价值
**劣势**: 6 文件起步；C++ only plugins；URDF 耦合（NormaCore 用 MJCF）；DDS 复杂度；对 1 人项目 overkill；Rust 非一等公民

### Option C: NormaCore 现状 + 渐进改进

**核心模式**: Station + NormFS + protobuf + bridge

```
norma_sim (Python, out-of-process)
  ↕ Unix socket / Envelope protocol (Hello/Welcome/Actuation/Snapshot)
SimulationRuntime (Rust)
  ↕ SnapshotBroker + ActuationSender (2-lane QoS)
st3215-compat-bridge (Rust)
  ↕ ActuatorMap (motor_id ↔ actuator_id)
Station Engine
  ↕ NormFS queues (commands, st3215/inference)
Web UI (React/TS)
```

**优势**: 已在工作；Rust 性能；架构边界 enforced；protobuf 跨语言契约；Web UI 内置
**劣势**: 无 Gymnasium 接口；无 step-on-demand 仿真模式；Camera 不在 snapshot 里；station_py 走 ST3215 字节而非弧度

---

## 2. 决策

### 选择: Option C — NormaCore 现状 + 渐进改进 + 借鉴 RCS 接口模式

**不选 ROS2 的理由**:
1. NormaCore 的 Station + NormFS 在当前规模下更简洁
2. ROS2 生态（MoveIt/Nav2/30+ controllers）对 imitation learning 无直接价值
3. C++ pluginlib 与 Rust 技术栈不兼容
4. URDF 耦合——NormaCore 用 MJCF
5. ROS2 的 lifecycle/fault-management 有价值，但对 1 人项目引入成本太高

**不选整体切 RCS 的理由**:
1. NormaCore 已有更好的运维基础设施（持久化、IPC、Web UI、多场景模式）
2. RCS 是单进程 Python，不适合 Rust + 多进程架构
3. NormaCore 的 bridge 模式支持 shadow mode、QoS 两通道、out-of-process sim

**从 RCS 借鉴**: Gymnasium 接口模式、物理单位全程贯穿、GripperConfig 设计

---

## 3. 现状诚实盘点 (v2 新增)

在定义改动范围之前，先承认当前代码的实际状态：

| 组件 | 当前状态 | Gymnasium env 需要 |
|------|----------|-------------------|
| **Sim 调度器** | 只有 `RealTimeScheduler`（wall-clock 驱动），无 step-on-demand | 需要 `SteppingScheduler`：`step(n)` 推进 n 个 physics tick 后返回 snapshot |
| **IPC 协议** | Envelope 有 `Actuation` 和 `Snapshot`，无 `Reset` / `StepRequest` | 需要扩展：加 `Reset`、`StepRequest`/`StepResponse` 消息 |
| **Snapshot 内容** | `velocity_value=0.0, effort_value=0.0, moving=False, torque_enabled=True` 全硬编码 | Phase 1 可接受（position-only 训练）；Phase 2 需要真实 velocity/effort |
| **Camera** | sim snapshot 无 sensor payload；hw camera 是独立 NormFS 队列，时间戳不同步 | Phase 1 不含 camera；Phase 2 需要 snapshot 扩展 + 时间戳对齐 |
| **station_py** | 走 `StationCommandsPack` protobuf，内含 ST3215 字节编码（GoalPosition 步数） | Phase 1 绕过 station_py，直连 norma_sim；Phase 2 需要 rad-native 硬件 API |
| **reset 语义** | bridge 有 `ResetActuator` intent，但 sim 只实现 `set_position` + `disable_torque` 代理 | 需要端到端 reset：`mj_resetData` + 重置 ctrl → 返回初始 snapshot |
| **action→obs 同步** | 异步：action 可能被 lossy 通道丢弃，snapshot 是周期性广播 | step-on-demand 模式天然解决：`StepRequest → StepResponse(snapshot)` |
| **错误/关闭** | bridge shutdown 是 no-op；station_py 自动重连；无确定性错误行为 | Gymnasium env 需要确定性 `close()` 和可检测的错误状态 |

---

## 4. 分阶段实施

### Phase 1: `NormaSimEnv` — 直连 norma_sim 的 sim-only Gymnasium env

**关键决定**: Phase 1 绕过 Station/bridge 整条链路，直接通过 Unix socket 和 norma_sim 通信。

**为什么不走 Station**:
- Station 路径是 `station_py → commands queue → bridge → SimulationRuntime → norma_sim`，每一步都引入异步延迟和丢包可能
- norma_sim 的 IPC 协议已经有 `Hello/Welcome/Actuation/Snapshot`，加 `Reset` 和 `StepRequest` 就够了
- 训练不需要 NormFS 持久化或 Web UI
- 这条路径产出最快、语义最干净

**需要改的代码**:

#### 4.1.1 norma_sim: 加 `SteppingScheduler` (Python)

```python
# norma_sim/scheduler/stepping.py
class SteppingScheduler:
    """Step-on-demand scheduler for Gymnasium integration.

    Unlike RealTimeScheduler, this does NOT run a continuous loop.
    Each call to step(n) advances physics by exactly n ticks,
    builds a snapshot, and returns it synchronously.
    """
    def __init__(self, world, physics_hz, publish_divider):
        ...

    def step(self, n_ticks: int) -> WorldSnapshot:
        """Advance n physics ticks, return snapshot."""
        for _ in range(n_ticks):
            with self.world.lock:
                self.world.step()
            self._tick += 1
        return self._builder.build(self._make_clock())

    def reset(self) -> WorldSnapshot:
        """mj_resetData + reset ctrl → return initial snapshot."""
        self.world.reset()  # wraps mj_resetData
        self._tick = 0
        return self._builder.build(self._make_clock())
```

改动位置: `software/sim-server/norma_sim/scheduler/stepping.py` (新文件)
影响: 纯 Python 新增，不改现有 `RealTimeScheduler`

#### 4.1.2 norma_sim IPC: 加 `Reset` 和 `StepRequest` 消息

```protobuf
// world.proto 扩展
message StepRequest {
  int32 n_ticks = 1;   // 推进多少个 physics ticks
}

message StepResponse {
  WorldSnapshot snapshot = 1;
}

message ResetRequest {}  // 重置世界到初始状态

// Envelope 新增 oneof variants
message Envelope {
  oneof payload {
    ...existing...
    StepRequest step_request = 7;
    StepResponse step_response = 8;
    ResetRequest reset_request = 9;
  }
}
```

改动位置:
- `protobufs/sim/world.proto` (加 3 个 message + Envelope 扩展)
- `software/sim-server/norma_sim/ipc/codec.py` (加 encode/decode)
- `software/sim-server/norma_sim/ipc/session.py` (加 step_request/reset_request handler)

影响: protobuf schema 变更，但只加新字段——向后兼容

#### 4.1.3 norma_sim CLI: 加 `--mode stepping` 启动选项

```python
# cli.py
if args.mode == "stepping":
    scheduler = SteppingScheduler(world, args.physics_hz, ...)
    # session._reader_loop 直接分发 StepRequest → scheduler.step() → StepResponse
elif args.mode == "realtime":
    scheduler = RealTimeScheduler(...)  # 现有行为
```

改动: `software/sim-server/norma_sim/cli.py` (加 `--mode` arg + stepping 分支)

#### 4.1.4 `NormaSimEnv` — Gymnasium 包装 (Python)

```python
# norma_sim/gym_env.py (或独立包)
class NormaSimEnv(gymnasium.Env):
    """Gymnasium env that talks directly to norma_sim over Unix socket."""

    def __init__(self, manifest_path, physics_hz=500, action_hz=30):
        # 1. 启动 norma_sim --mode stepping 子进程
        # 2. 连接 Unix socket, 完成 Hello/Welcome 握手
        # 3. 从 WorldDescriptor 推导 action_space/observation_space
        self.n_substeps = physics_hz // action_hz  # 每 step() 推进的 physics ticks

    def step(self, action):
        # 1. 发 ActuationBatch (action → SetPosition per actuator, 弧度)
        # 2. 发 StepRequest(n_ticks=self.n_substeps)
        # 3. 收 StepResponse(snapshot)
        # 4. 从 snapshot.actuators 提取 obs
        # 因为 step-on-demand，action→obs 因果关系是确定的
        return obs, 0.0, False, False, info

    def reset(self, *, seed=None, options=None):
        # 1. 发 ResetRequest
        # 2. 收 StepResponse(snapshot)  — reset 后的初始状态
        return obs, info

    def close(self):
        # 1. 发 Goodbye
        # 2. 关闭 socket
        # 3. terminate 子进程
```

改动: 新文件，纯 Python，依赖 norma_sim IPC 协议

#### 4.1.5 Phase 1 不做的事

| 不做 | 为什么 |
|------|--------|
| Camera observations | snapshot 无 sensor payload，需要先扩展 proto + MuJoCo 渲染管线 |
| Hardware mode (`mode="hw"`) | station_py 不是 rad-native，需要新的硬件 API |
| Shadow mode (`mode="shadow"`) | 需要 synchronized dual observations，现有 shadow 只是路由 |
| 通过 Station 走 | 引入 lossy queue + bridge 翻译，增加延迟和不确定性 |
| velocity/effort observation | snapshot 硬编码为 0，Phase 2 解决 |

#### 4.1.6 Phase 1 step() 语义定义

```
env.step(action) 的精确语义:

1. SEND: action dict → ActuationBatch (radians, reliable lane)
   - joints[i] → SetPosition(value=rad, actuator_ref=manifest.actuators[i])
   - gripper → SetPosition(value=normalized * (max_rad - min_rad) + min_rad)

2. ADVANCE: StepRequest(n_ticks = physics_hz / action_hz)
   - norma_sim 在 scheduler thread 同步执行 n_ticks 次 mj_step()
   - 所有 actuation 在第一个 tick 前 apply

3. OBSERVE: StepResponse(snapshot) → obs dict
   - joints[i] = snapshot.actuators[i].position_value (rad)
   - gripper = (position_value - min_rad) / (max_rad - min_rad)  (normalized [0,1])

4. DETERMINISM:
   - 同一个 (state, action) → 同一个 (next_state, obs)
   - 无丢包、无竞争、无时间依赖
   - world_tick 单调递增，可用于 reproducibility

5. ERROR:
   - socket 断开 → raise ConnectionError (env 不可恢复，需 close + 重建)
   - protobuf 解码失败 → raise RuntimeError
   - 不自动重连（训练需要确定性失败）
```

#### 4.1.7 Phase 1 reset() 语义定义

```
env.reset() 的精确语义:

1. SEND: ResetRequest

2. SERVER SIDE:
   - mujoco.mj_resetData(model, data) — 重置所有 qpos/qvel/ctrl/act/warm-start
   - tick 归零
   - 执行一次 mj_forward (计算初始位姿，不推进时间)
   - 构建 snapshot

3. OBSERVE: StepResponse(snapshot) → initial obs

4. SEED: MuJoCo 是确定性引擎，同一模型 reset 后状态相同
   - 如果将来需要随机化，在 reset 后用 StepRequest + 随机 actuation 实现
   - 不在 protocol 层加 seed 参数
```

---

### Phase 2: 硬件支持 + Camera (未来)

Phase 2 在 Phase 1 稳定后启动，触发条件：`NormaSimEnv` 能跑完一次完整的 imitation learning 训练循环。

| 工作项 | 依赖 | 改动范围 |
|--------|------|----------|
| **rad-native 硬件 API** | 新 protobuf 消息 or station_py 大改 | Rust (新 API) + Python (client) |
| **`NormaHwEnv`** | rad-native API + homing + torque lifecycle | Python (new env class) |
| **Camera in snapshot** | MuJoCo 渲染管线 + proto 扩展 + 时间戳同步 | Python (norma_sim) + proto |
| **velocity/effort obs** | MuJoCo `data.qvel` + `data.actuator_force` 读取 | Python (snapshot builder) |
| **Shadow mode env** | Phase 2 hw env + sim env + sync mechanism | Python |

**Phase 2 不在本 ADR 范围内**——需要独立的设计文档。

---

## 5. 对比矩阵 (v2 修正)

v1 矩阵过分强调基础设施指标（μs 延迟、Web UI），这些对训练接口决策无关。v2 围绕 env 语义：

| 维度 | RCS | NormaCore Phase 1 |
|------|-----|-------------------|
| **step 确定性** | 同步 `mj_step()` 调用 | 同步 `StepRequest → StepResponse` over UDS |
| **action→obs 因果** | 同一函数栈，guaranteed | StepRequest 包含 actuation，server 同步执行后返回 |
| **reset 语义** | `sim.reset()` → `mj_resetData` | `ResetRequest` → server `mj_resetData` → snapshot |
| **observation 完整性** | position + velocity + FK | position (Phase 1)；velocity/FK Phase 2 |
| **camera** | `SimCameraSet` in-process 渲染 | Phase 1: 无；Phase 2: snapshot 扩展 |
| **训练速度** | in-process，最快 | UDS + protobuf 开销，but step-on-demand 无 wall-clock 等待 |
| **硬件切换** | 换 env class | Phase 2 `NormaHwEnv` |
| **依赖** | gymnasium + mujoco + rcs (C++ build) | gymnasium + norma_sim (纯 Python) |

---

## 6. 架构图

### Phase 1: 训练时 (sim-only, 直连)

```
┌─── TRAINING ──────────────────────────────────┐
│                                                │
│  LeRobot / ACT / Custom Policy                 │
│      ↓                                         │
│  NormaSimEnv (Gymnasium API)                   │
│  ├─ action_space: {joints: Box(rad),           │
│  │                  gripper: Box(0,1)}         │
│  ├─ obs_space: {joints: Box(rad),              │
│  │               gripper: Box(0,1)}            │
│  ├─ step(action) → StepRequest → StepResponse │
│  └─ reset() → ResetRequest → StepResponse     │
│      ↓                                         │
│  Unix socket (Envelope protocol)               │
│                                                │
└──────────────┬─────────────────────────────────┘
               ↓
┌─── norma_sim (Python subprocess) ──────────────┐
│                                                 │
│  SteppingScheduler                              │
│  ├─ step(n) → n × mj_step → build snapshot     │
│  └─ reset() → mj_resetData → build snapshot    │
│                                                 │
│  IpcServer (asyncio UDS)                        │
│  ├─ handles StepRequest → scheduler.step()      │
│  ├─ handles ResetRequest → scheduler.reset()    │
│  └─ handles Actuation → applier.drain_and_apply │
│                                                 │
│  MuJoCoWorld + ActuationApplier + SnapshotBuilder│
│                                                  │
└──────────────────────────────────────────────────┘
```

### 运维时 (Web UI + 实时仿真, 现有架构不变)

```
Station (Rust) ← norma_sim --mode realtime
  ↕ SimulationRuntime
  ↕ st3215-compat-bridge
  ↕ NormFS queues
  ↕ WebSocket
Web UI (React)
```

**两条路径共存**：训练用 `--mode stepping` 直连，运维用 `--mode realtime` 走 Station。不互相干扰。

---

## 7. 验证标准

### Phase 1 完成标准

1. `NormaSimEnv` 能跑标准 Gymnasium 循环 — `env.reset()` → N 次 `env.step()` → `env.close()`
2. `env.step()` 确定性 — 同一序列 action 产出同一序列 observation（`world_tick` 可验证）
3. action/obs 全程弧度 — 训练代码里看不到 steps
4. `env.reset()` 真正重置 — `mj_resetData` + tick 归零
5. `env.close()` 干净关闭 — subprocess 终止，socket 清理
6. 错误确定性 — socket 断开 raise，不自动重连
7. 能跑一个简单的 imitation learning 训练循环（记录 episode → replay → 验证 obs 一致）
8. 现有 `--mode realtime` + Station + Web UI 不受影响

### Phase 1 不验证

- Camera observations (Phase 2)
- Hardware mode (Phase 2)
- 训练速度优化 (确认语义正确后再优化)

---

## 8. 工作量诚实估计

| 工作项 | 文件 | 改动类型 | 估计 |
|--------|------|----------|------|
| `SteppingScheduler` | `scheduler/stepping.py` (新) | 纯 Python | ~100 行 |
| `MuJoCoWorld.reset()` | `world/model.py` | Python 改动 | ~10 行 |
| Proto: `StepRequest/Response/ResetRequest` | `world.proto` | schema 扩展 | ~15 行 |
| Codec: encode/decode 新消息 | `ipc/codec.py` | Python 改动 | ~40 行 |
| Session: 处理新消息 | `ipc/session.py` | Python 改动 | ~30 行 |
| CLI: `--mode stepping` | `cli.py` | Python 改动 | ~20 行 |
| `NormaSimEnv` | `gym_env.py` (新) | 纯 Python | ~200 行 |
| 测试 | tests/ | 新测试 | ~150 行 |
| **总计** | | | **~565 行 Python + ~15 行 proto** |

**不涉及 Rust 代码改动。** Station、bridge、sim-runtime 全部不动。

---

## 9. 总结

| 维度 | v1 决策 | v2 决策 (修正后) |
|------|---------|------------------|
| **架构路线** | 保持现有 + adapter | 保持现有 + **直连 norma_sim** adapter |
| **训练接口** | `station_py.NormaEnv` | `NormaSimEnv` (直连 Unix socket，不走 Station) |
| **step 语义** | 未定义 | `StepRequest → StepResponse`，同步确定性 |
| **reset 语义** | 未定义 | `ResetRequest → mj_resetData → snapshot` |
| **Camera** | "从 snapshot 取" | Phase 2，承认当前 snapshot 无 sensor payload |
| **Hardware** | "改 mode 就行" | Phase 2，承认需要 rad-native 硬件 API |
| **改动范围** | "纯 Python，不动 Rust" | **纯 Python + proto 扩展，确实不动 Rust** |
| **RCS 借鉴** | 接口模式 + GripperConfig | 同上 |
| **ROS2** | 不引入 | 不引入 |

**一句话**: 在 norma_sim 里加一个 step-on-demand 调度器和 3 个 IPC 消息，就能得到一个语义干净的 Gymnasium env，不经过 Station/bridge 的 lossy 异步链路。训练路径和运维路径分离，互不干扰。
