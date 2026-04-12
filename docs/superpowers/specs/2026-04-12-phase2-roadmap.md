# NormaCore Phase 2 Roadmap — 从训练到部署

> 综合 Claude 架构分析 + Codex 代码级审查，形成分阶段执行计划

**Date**: 2026-04-12
**Depends on**: Phase 1 (NormaSimEnv + SteppingScheduler + mjviser) — 已完成
**Goal**: 让 imitation learning 能闭环：录制 → 训练 → sim 验证 → 硬件部署

---

## 0. 立即修复 (Codex 发现的 bug)

### 0.1 时序 bug：31.25Hz ≠ 30Hz

**问题**: `n_substeps = physics_hz // action_hz = 500 // 30 = 16`，实际控制频率 = 500/16 = 31.25Hz。
**影响**: sim 训练的控制频率和 real 部署不一致，是 sim-to-real gap 的隐蔽来源。

**修复**:
```python
# gym_env.py — 用 round() 并暴露实际频率
self.n_substeps = round(physics_hz / action_hz)
self.actual_action_hz = physics_hz / self.n_substeps  # 暴露给用户
```

文件: `norma_sim/gym_env.py:90`

### 0.2 seed 硬编码

**问题**: `reset()` 硬编码 `ResetRequest(seed=1)`，服务端忽略 seed。
**影响**: 无法做 seeded deterministic reset，reproducibility 是半成品。

**修复**:
- `gym_env.py`: `reset(seed=N)` → `ResetRequest(seed=N or 0)`
- `session.py`: 把 seed 传给 `on_reset(seed)`
- `stepping.py`: `reset(seed)` 暂时忽略（MuJoCo 本身是确定性的），但预留接口给将来的 domain randomization

文件: `gym_env.py:291`, `session.py:216`, `stepping.py:55`

### 0.3 snapshot 语义修正

**问题**: `velocity_value=0.0, effort_value=0.0, moving=False` 全硬编码。
**影响**: shadow mode 对比失真；训练 policy 如果依赖 velocity obs 会得到错误信号。

**修复** (最小):
```python
# snapshot.py — 从 MuJoCo data 读真实 velocity
velocity_value = float(data.qvel[dof_addr])  # 需要 jnt_dofadr 映射
```

文件: `norma_sim/world/snapshot.py:57`

---

## 1. Phase 2A — 完善 sim 训练路径

**目标**: NormaSimEnv 能完成一次完整的 imitation learning 训练循环（proprio-only）。

### 1.1 Canonical Episode Format

定义一个训练和录制共用的数据格式：

```python
@dataclass
class EpisodeStep:
    timestamp_ns: int          # world_tick * physics_timestep_ns (sim) 或 monotonic (real)
    joints_rad: np.ndarray     # (n_joints,) float64
    gripper_normalized: np.ndarray  # (n_grippers,) float64 [0,1]
    action_joints_rad: np.ndarray   # 上一步的 action
    action_gripper: np.ndarray
    # Phase 2C 加: camera_rgb: np.ndarray  # (H, W, 3) uint8

@dataclass
class Episode:
    robot_id: str
    steps: list[EpisodeStep]
    metadata: dict  # physics_hz, action_hz, seed, manifest_path, ...
```

**两个 adapter**:
- `NormaSimEnv → Episode`: 在 step() 循环里收集
- `normvla.Frame → Episode`: 从 Station NormFS 队列读取，rad 转换

**输出格式**: HDF5 (兼容 LeRobot) 或 简单 npz。

文件: 新建 `norma_sim/episode.py` (~150 行)

### 1.2 Episode Recorder Wrapper

```python
env = NormaSimEnv(manifest_path=..., render_port=8012)
env = EpisodeRecorder(env, save_dir="episodes/")
# 自动在 reset() 时开始新 episode，close() 时保存
```

文件: 新建 `norma_sim/episode_recorder.py` (~100 行)

### 1.3 Timing Contract 文档化

在 `NormaSimEnv.__init__` 里打印实际控制频率：
```
NormaSimEnv: physics_hz=500 action_hz=30 n_substeps=17 actual_hz=29.41Hz
```

### 1.4 测试

- Episode 格式 roundtrip (write → read → compare)
- EpisodeRecorder 保存完整 episode
- 时序精度验证

**估计**: ~400 行 Python

---

## 2. Phase 2B — 硬件部署路径

**目标**: 训练出的 policy 能在真实 SO-101 上跑。

### 2.1 NormaHwEnv — 薄的硬件 Gymnasium 接口

```python
class NormaHwEnv(gymnasium.Env):
    """Gymnasium env that controls real hardware via Station."""

    def __init__(self, station_url, robot_id, preset_path, action_hz=30):
        # 连接 Station WebSocket
        # 从 preset YAML 读取 actuator 映射和 ctrlrange
        # 构建 action_space / observation_space（和 NormaSimEnv 一致）

    def step(self, action):
        # 1. action dict → StationCommandsPack (ST3215 GoalPosition, steps)
        # 2. 发送到 Station commands queue
        # 3. 等 1/action_hz 秒
        # 4. 读 st3215/inference 队列最新状态
        # 5. steps → radians → obs dict
        return obs, 0.0, False, False, info

    def reset(self):
        # 1. 发送 homing 命令（所有关节回 home 位置）
        # 2. 等待到达（poll state 直到位置收敛）
        # 3. 返回 initial obs

    def close(self):
        # disable torque → 安全断开
```

**关键设计决定**:
- NormaHwEnv 走 Station（不绕过），因为 Station 管理硬件生命周期
- action/obs 合约和 NormaSimEnv **完全一致**（弧度 + normalized gripper）
- 内部做 rad↔steps 转换（复用 preset YAML 的 offset_steps 配置）
- `step()` 是异步的（发送命令 → 等时间 → 读状态），不像 sim 的确定性 step

文件: 新建 `station_py/hw_env.py` (~250 行)

### 2.2 安全基元

在 `NormaHwEnv` 里内置：

| 安全机制 | 实现 |
|----------|------|
| **Watchdog** | 如果 `step()` 超过 2×period 没被调用，自动 disable torque |
| **Homing gate** | `reset()` 必须完成 homing 才能 `step()`，否则 raise |
| **Command timeout** | 如果 Station 无响应 >500ms，disable torque + raise |
| **Joint limits** | `step()` 里 clip action 到 ctrlrange，不发超限命令 |
| **E-stop** | `env.emergency_stop()` → 立即 disable 所有 torque |

文件: 内置在 `hw_env.py`

### 2.3 共享 Python 接口

```python
# norma_envs/base.py
class NormaEnvBase(gymnasium.Env):
    """NormaSimEnv 和 NormaHwEnv 共享的接口约定。"""
    action_space: gym.spaces.Dict  # {joints: Box(rad), gripper: Box(0,1)}
    observation_space: gym.spaces.Dict

    @property
    def robot_id(self) -> str: ...
    @property
    def actual_action_hz(self) -> float: ...
```

不做深层抽象——只是一个 Protocol/ABC 确保两个 env 接口一致。

### 2.4 测试

- NormaHwEnv 单元测试（mock Station WebSocket）
- 接口一致性测试（NormaSimEnv 和 NormaHwEnv 的 space 定义一致）
- Safety watchdog 测试

**估计**: ~500 行 Python

---

## 3. Phase 2C — Sim-Real 桥接

**目标**: 验证 sim 训练的 policy 在 real 上能跑。

### 3.1 Timing Harness

```python
class TimedEnvWrapper(gymnasium.Wrapper):
    """测量实际控制频率和延迟。"""
    def step(self, action):
        t0 = time.monotonic_ns()
        result = self.env.step(action)
        dt = time.monotonic_ns() - t0
        self._latencies.append(dt)
        return result

    def report(self) -> dict:
        return {
            "mean_hz": ..., "p50_latency_ms": ..., "p99_latency_ms": ...,
            "command_drops": ...,
        }
```

### 3.2 Sim vs Real 对比轨迹

```python
# 1. 在 sim 里跑一条固定 action 序列，记录 obs 轨迹
sim_episode = run_fixed_actions(NormaSimEnv(...), actions)

# 2. 在 real 里跑同一条 action 序列，记录 obs 轨迹
real_episode = run_fixed_actions(NormaHwEnv(...), actions)

# 3. 对比
compare_episodes(sim_episode, real_episode)
# → joint position RMSE, tracking delay, overshoot
```

### 3.3 Domain Randomization 接口 (预留)

```python
class NormaSimEnv:
    def reset(self, seed=None, options=None):
        # options["domain_rand"] = {
        #     "friction_scale": (0.8, 1.2),
        #     "mass_scale": (0.9, 1.1),
        #     "actuator_gain_scale": (0.8, 1.2),
        # }
        # → 修改 MuJoCo model 参数后 reset
```

Phase 2C 只预留接口，不实现 randomization 逻辑。

**估计**: ~300 行 Python

---

## 4. 明确推迟

| 推迟项 | 理由 | 触发条件 |
|--------|------|----------|
| Camera observations | 先证明 proprio-only IL 能跑通 | 需要 VLA 时 |
| Camera sim/real 统一 | 先解决一个 camera 一个 task | Camera obs 做完后 |
| 深层 Robot 抽象 | 会违反当前隔离边界，1 人项目不值得 | 支持 ≥3 种机器人时 |
| 向量化 stepping server | 多进程够用 | 训练瓶颈在 env 吞吐时 |
| 完整 digital twin / shadow mode | 需要双通道同步观测 | sim-real gap 需要在线比对时 |
| 物理模型 sysid | `sysid_complete: false`，但先跑 demo | 发现 sim-real gap 大于预期时 |

---

## 5. 执行顺序

```
Week 1: Phase 0 (bug fixes)
  ├─ 0.1 时序 bug (30 min)
  ├─ 0.2 seed 接口 (30 min)
  └─ 0.3 velocity 读取 (1 hr)

Week 1-2: Phase 2A (sim 训练完善)
  ├─ 1.1 Episode format (2 hr)
  ├─ 1.2 EpisodeRecorder (1 hr)
  ├─ 1.3 Timing contract (30 min)
  └─ 1.4 Tests (1 hr)

Week 2-3: Phase 2B (硬件部署) ← 需要硬件
  ├─ 2.1 NormaHwEnv (3 hr)
  ├─ 2.2 Safety primitives (2 hr)
  ├─ 2.3 Shared interface (30 min)
  └─ 2.4 Tests (1 hr)

Week 3-4: Phase 2C (sim-real 桥接) ← 需要硬件
  ├─ 3.1 Timing harness (1 hr)
  ├─ 3.2 Sim vs real comparison (2 hr)
  └─ 3.3 DR interface stub (30 min)
```

**里程碑**:
- M1: sim 训练能录制 + 回放 episode (Phase 0 + 2A 完成)
- M2: policy 能在真实臂上跑 (Phase 2B 完成)
- M3: sim 和 real 轨迹可量化对比 (Phase 2C 完成)

---

## 6. 风险

| 风险 | 概率 | 影响 | 缓解 |
|------|------|------|------|
| 物理模型不准 (sysid_complete=false) | 高 | sim 训练的 policy 在 real 上不工作 | Phase 2C 对比轨迹先暴露问题；预留 DR 接口 |
| Station Python SDK 不是 rad-native | 中 | NormaHwEnv 需要手动做 steps↔rad 转换 | 复用 preset YAML 的 offset_steps，和 bridge 用同一个转换逻辑 |
| 硬件延迟 > sim 延迟 | 高 | 训练 30Hz 的 policy 在 real 上跟不上 | Timing harness 先量化；可能需要降低 action_hz |
| 合约漂移（Codex 核心洞察） | 中 | NormaSimEnv / normvla / preset / 硬件 之间不一致 | Episode format 是合约锚点；所有路径写同一个格式 |
