# NormaCore 仿真集成设计（v2）

| | |
|---|---|
| **日期** | 2026-04-10 |
| **状态** | Draft — 待 review |
| **版本** | v2（v1 已归档为 `2026-04-10-simulation-integration-v1-driver-shim-archive.md`） |
| **范围** | MVP-1 完整 scope + MVP-2 概要 |
| **驱动场景** | 无硬件开发 / 调试（Station 在无真机环境下完整运行）+ shadow mode 预留 |
| **设计原则** | 面向未来、优雅精致的架构 |

---

## 0. v1 → v2 变更摘要

本 spec 是 v1 的完全重写。v1 通过了 spec-document-reviewer 的 review loop，但在 codex 深度架构 review（session `019d7726-6dcf-7fe2-8887-35ee3b9c2568`）下暴露了 6 个维度的结构性缺陷。最致命的一句评语：

> **v1 把共享的仿真世界建模成了一个假的 ST3215 driver。MVP-1 能跑，MVP-2 以后会持续从这个缝里漏血。**

v2 的核心修正是把"共享世界服务"和"假 ST3215 设备"两个混为一谈的概念**分离并正确分层**。从"fake driver shim"升级为"一等 subsystem + capability-keyed schema + 兼容桥接"。

### Codex Top 5 的直接对应

| # | Codex 批评 | v2 对应节 |
|---|---|---|
| 1 | 把 sim 提升为 Station 内的一等 `SimulationRuntime`，不要定义为 fake `st3215` driver | §5 §6.1 |
| 2 | `sim.proto` 改成通用 world/actuator/sensor schema，capability-keyed；ST3215 bytes 只在兼容层 | §6.5 §7 |
| 3 | 拆开 `physics_hz / publish_hz / render_hz`，real-time 降级为 scheduler policy | §8 |
| 4 | 引入显式 supervisor/runtime handle 和 health stream，消除隐式副通道 | §6.2 §10.5 |
| 5 | 抽 `st3215-wire`，pack/unpack 在其中；机器人 preset 不准进 real driver crate | §6.2 §6.3 |

### v1 → v2 架构速览

| 维度 | v1 | v2 |
|---|---|---|
| 核心抽象 | `st3215-sim` driver（假装硬件）| `SimulationRuntime` subsystem + `CompatibilityBridge` |
| Station 集成 | 一个 driver 插进 driver loader | 一等 subsystem，和 `NormFS` / tcp / web 平级 |
| IPC Schema | `sim.proto`（`bus_serial + motor_id + SetPosition{target_rad}`）| `world.proto`（`robot_id + actuator_ref + capability-keyed intent`）|
| 时间模型 | real-time only、一帧一 step、`state_rate_hz` 硬绑 | `world_tick` canonical + `physics_hz / publish_hz / render_hz` 三独立频率 |
| 命令 QoS | 单通道 drop-oldest | `QOS_LOSSY_SETPOINT` + `QOS_RELIABLE_CONTROL` 双通道 |
| `sim_manager` | Station 内的模块，driver 的依赖 | **消失**。其职责由 `SimulationRuntime` 和 `/sim/health` broadcast 取代 |
| Real vs sim | `MutuallyExclusive` config 校验 | `bus_serial` 命名空间共存，**shadow mode 是第一类场景** |
| 协议库位置 | `st3215::protocol::pack` 在 real driver crate 里 | 独立 `st3215-wire` crate（纯协议，零 tokio/normfs 依赖）|
| 机器人 SKU preset | Rust 常量塞进 real driver crate | `st3215-compat-bridge/presets/*.yaml` 数据文件 |
| Crate 数量 | `st3215-sim` + `sim_manager` 模块 | `sim-runtime` + `st3215-wire` + `st3215-compat-bridge` 三个新 crate |
| 运行时状态文件 | sentinel + `.lock` + socket 三件套 | 单一 `TempRuntimeDir`（socket-as-readiness）|
| Projection 方向 | 单向下行 | 双向 Bridge（commands 上行 + snapshots 下行）|
| Python 包 ST3215 痕迹 | 允许（寄存器字节在 Python 生成） | **零**（CI 强制 `grep -ri st3215` 零命中） |
| 健康感知 | `sim_manager` 隐式回调 `driver.mark_crashed()` | `/sim/health` queue broadcast，bridge 订阅 |
| 诊断工具 | 一个 `inspect.py` | `inspect.py` + `probe_manifest.py` + `send_actuation.py` |
| 代码量估算 | ~2100 行新代码 | ~6430 行新代码（~1.7x）|

多出的 ~4000 行是**架构正确性的标价**，在 AI 辅助编码语境下是合理投资。

---

## 1. TL;DR

给 Station 加一个**一等的仿真 subsystem**，让它能在没有 ElRobot 真机的情况下跑完整代码路径。

**技术总览**：

- **`SimulationRuntime` Rust subsystem**（`software/sim-runtime/` 新 crate）和 `NormFS` / tcp server / web server 平级。它拥有 `WorldBackend` trait、`WorldClock`、`/sim/health` queue 发布器、`WorldSnapshot` broadcast、`ActuationBatch` 出站。
- **物理引擎 MuJoCo**（Apache 2.0、IL 事实标准、接触物理强、URDF 成熟）
- **运行在独立 Python 进程 `norma_sim`**（`software/sim-server/`），通过 Unix domain socket + length-prefixed protobuf 和 Rust 通信。Python 包**零 ST3215 痕迹**（CI 强制）。
- **Capability-keyed schema** `protobufs/sim/world.proto`：`robot_id + actuator_ref + ActuationCommand{oneof SetPosition/...}`。单位由 `ActuatorCapability.kind` 决定（`CAP_REVOLUTE_POSITION` / `CAP_GRIPPER_PARALLEL` / ...），不硬编码进字段名。
- **`st3215-wire` 独立 crate**：纯协议库，无 tokio/normfs/station 依赖。real `st3215` driver 和 sim bridge 共享这份"寄存器布局 + 打包/解包"单点真相。机器人 SKU 级别的 preset 做成 yaml 数据文件，不进 crate。
- **`st3215-compat-bridge`**（`software/sim-bridges/st3215-compat-bridge/` 新 crate）：双向 Bridge。上行**订阅全局 `commands` queue**（`station_iface::COMMANDS_QUEUE_ID = "commands"`），按 `StationCommandType::StcSt3215Command` 过滤并按 `target_bus_serial` 匹配本 bridge 的 `legacy_bus_serial`，翻译为 `ActuationBatch`；下行订阅 `WorldSnapshot` 用 `st3215-wire` 打包为 71 字节 dump 写进 `st3215/inference` queue。下游 web UI 和 station_py 零修改。
- **时间模型**：`world_tick` 是 canonical time，独立于 wall clock。`physics_hz=500 / publish_hz=100 / render_hz=30` 三个独立频率（render 仅 MVP-2+）。Real-time 是 scheduler policy，不是核心时间语义。Tick-driven deterministic mode 在 schema 和 scheduler protocol 层预留，MVP-1 不实现。
- **启动 UX**：`station -c station-sim.yaml` 一条命令。Station 通过完全不透明的 `launcher: [cmd, args...]` config 启动 Python backend，只通过环境变量 `NORMA_SIM_SOCKET_PATH` 告诉它去哪 bind。**Station 不做 Python 包管理**。
- **`world.yaml` 是 build + runtime 双用途 manifest**：gen.py 根据它从 URDF 派生 MJCF；`norma_sim` 启动时读同一份 yaml 构造 `WorldDescriptor`。一份文件两个用途，零一致性风险。
- **Shadow mode 是第一类场景**：real `st3215` 和 sim bridge 通过 `bus_serial` 命名空间共存，不走 mutual exclusion 校验。
- **`/sim/health` queue** 发布结构化健康事件（1Hz + event-driven extra push），所有消费者（bridge / web UI / 未来的 driver）订阅它感知 sim 死活——**消灭所有隐式副通道**。
- **MVP-1 scope**：ElRobot follower 8 关节 + 夹爪 `GRIPPER_PARALLEL` 归一化 + MJCF `<equality>` 约束驱动 prismatic mimic + 空世界；shadow mode + external mode 都支持。
- **MVP-2 scope**：`usbvideo-compat-bridge` 新 crate + MuJoCo 相机渲染 + `CAMERA_RGB` capability（schema slot 在 MVP-1 已预留）+ NormVLA 端到端。

---

## 2. 背景与动机

### 2.1 当前状态

NormaCore 是一个面向物理 AI 研究和机器人自动化的端到端平台：自研硬件（ElRobot 7+1 DoF 机械臂、PGripper 夹爪）+ Station 软件平台（Rust 单二进制 + 多个 Rust drivers + Web UI + Python/Go 客户端 SDK + 自研 gremlin protobuf + NormFS 持久化队列）。技术栈以 Rust 为主，Go 做 gremlin，Python 做客户端 SDK 和生成器。

**已有的仿真原料**（grep 验证）：
- `hardware/elrobot/simulation/elrobot_follower.urdf` — 完整 URDF（521 行，含 inertia/mass/visual/collision）
- `hardware/elrobot/simulation/assets/*.stl` — 19 个网格
- `software/station/clients/station-viewer/public/` — URDF 副本 + Three.js + `urdf-loader` 做**可视化**

**完全缺失**（grep 零命中）：
- 任何物理引擎集成（mujoco / pybullet / drake / isaac / genesis / gazebo）
- 任何 mock / fake / virtual / stub driver
- MJCF、leader 臂 URDF、pgripper 独立 URDF
- Python sim server 或示例

当前的 `simulation/` 目录实际用途是让 web UI 把**真机数据流**可视化成 3D 动画——是"数据的 3D 投影"，不是"世界的仿真"。

### 2.2 用户故事

**主要**：*"我是 Station dev，手上没接机械臂，现在想跑 Station、打开浏览器看到 web UI、发个命令让 ElRobot 在 UI 里动起来。"*

**次要**：
- 无硬件 CI 能跑（将来）
- IL/RL 研究者能用 MuJoCo 生态做策略训练（未来 MVP-B/C/F）
- sim-to-real 验证时能做真机 + sim 并跑的 divergence 分析（**shadow mode，v2 第一类场景**）
- Web UI 和客户端 SDK 能在 dev 机上 dogfood

### 2.3 v1 被推翻的原因

v1 spec 把仿真建模成"一个假的 ST3215 driver"（`st3215-sim` crate 实现和 real driver 一致的接口，内部转发到 Python 子进程）。这个选择让 MVP-1 容易落地，但创造了以下结构性问题（codex 深度 review 揭示）：

1. **两套设计硬拼**：v1 同时存在"共享世界服务"概念（`sim_manager`、shared MuJoCo、multi-client、future camera）和"假 ST3215 设备"概念（`st3215-sim`、raw register bytes、mutually exclusive）。两套核心抽象不是一个东西——Fred Brooks 意义上的概念完整性破损。

2. **Schema 绑死 ST3215**：v1 的 `sim.proto` 用 `bus_serial + motor_id + SetPosition{target_rad}` 作为一等字段。未来 linear motor、soft actuator、pgripper、mobile base 都会被硬塞进"旋转舵机总线"这个错误抽象。

3. **时间模型耦合错了**：v1 的 `state_rate_hz=100` 和 `timestep=0.002` 组合意味着每 10ms wall clock 只推进 2ms sim time，要么 sim 5 倍慢，要么 spec 没定义清楚步进模型。`real-time only` + `non-deterministic` + `broadcast 100Hz` 三者绑死让 replay、精确对齐、faster-than-real-time 和可复现实验都默认不可能。

4. **Mutual exclusion 锁死 shadow mode**：v1 的 `Drivers::validate()` 硬拒 `st3215 + st3215-sim` 共存，直接封死 digital twin、sim-to-real 对照分析、divergence 验证。

5. **隐式副通道**：v1 要求 `sim_manager` 监控 subprocess 退出并"通知 `st3215-sim` driver 进 Crashed 状态"，但当前 `StationEngine` 只有 `register_queue()`，`Station::start_drivers()` 不保留 driver handle——实现只能造全局单例 / watch channel 之类的脏耦合。

6. **协议知识漂移**：v1 把 `st3215::protocol::pack` 塞进 real driver crate，同时在那里定义 `MotorPreset{"elrobot-follower"}`——把机器人 SKU 目录塞进舵机协议库。既违反职责分离，又让 real driver 和 sim driver 无法纯粹共享协议层。

v2 的每一个设计决定都针对性地解决上述问题之一。

### 2.4 继续成立的 v1 假设

以下 v1 决定在 v2 **继续成立**，没有被 codex 批评推翻：

- **Level 3 物理驱动**：接触物理是夹爪抓取验证前提，Level 1/2 kinematic mock 无法胜任
- **MuJoCo 作为物理引擎**：IL/RL 研究事实标准、Apache 2.0、URDF 导入成熟、Python 绑定质量好
- **独立 Python sim 进程**（而不是 Rust in-process FFI 或 Python 完全替代 Station）
- **Unix domain socket + length-prefixed protobuf** 作为 IPC 传输（本地 dev loop 够用）
- **启动期快错、运行期软错** 的错误哲学
- **无自动重启 / 无自动重连** 的恢复策略
- **CI 不进 GitHub Actions**，仅提供 Makefile target 的立场
- **Apache 2.0 项目调性**——Isaac Sim / 闭源商业引擎继续拒绝
- **MVP-1 scope = ElRobot follower 8 关节 + 夹爪 + 空世界**

---

## 3. Goals / Non-Goals

### 3.1 MVP-1 Goals

- `station -c station-sim.yaml` 一条命令从干净仓库启动，浏览器访问 web UI 看到 ElRobot follower 3D 模型动态响应命令
- 7 个主臂 revolute 关节 + 1 个夹爪关节在 MuJoCo 物理下正确运动
- 夹爪的两个 prismatic mimic 关节通过 MJCF `<equality>` polycoef 约束正确联动（★ Level 3 价值证明点）
- `st3215-compat-bridge` 写 `st3215/inference` queue 的字节与真机二进制兼容，下游 web UI 和 Python 客户端零修改
- **Capability-keyed schema** 已全部到位：Python 侧零 ST3215 痕迹（CI 强制）
- **Shadow mode** 可配置：真机 `st3215` 和 sim bridge 通过 `bus_serial` 命名空间共存
- **External mode** 可用：terminal 1 跑 sim，terminal 2 跑 Station
- `/sim/health` queue 发布结构化健康事件，bridge 通过订阅感知 backend 死活
- 启动失败走快错 + 明确错误；运行期 subprocess 崩溃走软错，Station 继续运行
- `WorldBackend` trait + `ChildProcessBackend` + `ExternalSocketBackend` 两个 impl 就位，未来 `NativeBackend` 是增量添加
- 三个诊断工具：`inspect.py` / `probe_manifest.py` / `send_actuation.py`

### 3.2 MVP-2 Goals（紧接里程碑）

- 新 crate `software/sim-bridges/usbvideo-compat-bridge/`
- `norma_sim/world/rendering.py` + MuJoCo Renderer API
- `world.yaml` 的 `sensors:` 段加 `CAMERA_RGB` capability；gen.py 根据 manifest 生成 MJCF `<camera>` 标签
- NormVLA 推理回路端到端跑通（关节 + 假图像）
- **关键验证**：MVP-2 不应该碰 `sim-runtime` crate。如果碰了，说明 MVP-1 的抽象不够通用。这是 MVP-1 架构质量的延迟验收测试

### 3.3 显式 Non-Goals（MVP-1/2 都不做）

- ❌ Leader 臂（URDF 尚不存在，需硬件团队建模）
- ❌ PGripper（独立产品线，需新的 capability 和 bridge）
- ❌ SO-101 机型
- ❌ 桌面场景 / 可抓取物体 / pick-and-place 任务
- ❌ 遥操作 (motors-mirroring) 的 sim 版本
- ❌ CI 自动化集成（Makefile target 有但不进 GitHub Actions）
- ❌ 确定性 / seed 控制 / reproducible runs（`TickDrivenScheduler` 在 schema 和 protocol 层预留，impl MVP-5）
- ❌ 性能 benchmark（吞吐 / 延迟指标）
- ❌ 自动重启 / 自动重连 / 健康检查 / heartbeat / 熔断
- ❌ 鉴权 / 加密 / 跨机器仿真
- ❌ Graceful degradation（sim 挂了不会回落 kinematic mock）
- ❌ 在线 GUI viewer（MuJoCo headless，可视化走 web UI）
- ❌ Fuzzing / fault injection / 性能 profile 模式
- ❌ Systemd unit / docker compose / 二进制分发
- ❌ **Top-level `robots:` config section**（`robot_id` 只在 schema 和 bridge config 层一等，Station-level robots 抽象留给独立 spec）
- ❌ **`NativeBackend` 实现**（trait 就位，impl 留给 MVP-6）
- ❌ **OpenTelemetry / distributed tracing / Prometheus metrics export**
- ❌ **Web UI 展示 `/sim/health`**（web UI 是下游 client，本 spec 不修）
- ❌ **Real driver 的 robot-awareness 重构**（独立更大的 spec）
- ❌ **`auto` 逻辑在多 bus 下的行为修正**（v1 既有 bug，本 spec 在 shadow mode 中通过"必须显式配置 `st3215-bus`"绕过）

---

## 4. 核心设计决定

| 决定 | v2 选择 | 关键理由 |
|---|---|---|
| Fidelity level | **Level 3 物理驱动** | 接触物理是夹爪抓取验证前提；为场景 B/C/F 铺路 |
| 物理引擎 | **MuJoCo** | IL/RL 事实标准、Apache 2.0、URDF 成熟、接触物理稳 |
| Sim 进程位置 | **独立 Python 进程** | 复用 IL/RL 生态，sim 迭代不拖累 Rust build，天然升级到场景 B/C/F |
| **Sim 在 Station 中的地位** | **一等 subsystem `SimulationRuntime`**，和 NormFS / tcp / web 平级 | codex #1：拒绝"fake driver shim"；概念完整性 |
| **核心 schema 抽象** | **`WorldTick + ActuationIntent + CompatibilityBridge`** | codex #6：单一核心抽象统一整个设计 |
| **Schema 寻址** | **`robot_id + actuator_ref`**（字符串，非 bus+motor_id） | codex #2 §2.2：去硬件化 |
| **Schema 命令语义** | **Capability-keyed intent**（`SetPosition{value}` + `capability.kind` 决定单位） | codex #2：支持未来 linear / soft / gripper / camera |
| **时间基准** | **`world_tick` canonical + `sim_time_ns` 推导 + `wall_time_ns` 辅助** | codex #3 §3.4：解耦时间语义和实时策略 |
| **调度频率** | **`physics_hz / publish_hz / render_hz` 三独立**，整除约束 | codex #4.1：修正 v1 时间步进 bug |
| **Real-time 地位** | **Scheduler policy，不是核心时间模型**（`TickDrivenScheduler` 枚举值预留）| codex #3：为 deterministic/replay 铺路 |
| **Station 集成点** | **`Arc<SimulationRuntime>` 字段**，Station 显式持有 handle | codex #4：消除隐式副通道 |
| **Backend 抽象** | **`WorldBackend` trait（`pub(crate)`）**，`ChildProcessBackend` + `ExternalSocketBackend` + `MockBackend` | codex #5.4：为 `NativeBackend` / `RemoteBackend` 铺路；subsystem 边界编译期强制 |
| **Subprocess 运行时状态** | **`TempRuntimeDir`**（RAII 目录），socket-as-readiness | codex §4.3：状态机爆炸压缩为单一目录生命周期 |
| **IPC 传输** | **Unix domain socket**（MVP-1）；未来加 shared memory 做 data plane（MVP-2+） | 本地 dev loop 够用；trait 层面留扩展 |
| **IPC 编码** | **Length-prefixed protobuf**（新 `protobufs/sim/world.proto`） | 跨语言 + 结构化 + 复用 gremlin 管道 |
| **命令 QoS** | **`QOS_LOSSY_SETPOINT` + `QOS_RELIABLE_CONTROL` 双通道** | codex §3.2：修正 v1 的 discrete 动作被丢的 bug |
| **`st3215-sim` driver** | **不存在**，由 `St3215CompatBridge` 取代 | codex §6.1：bridge 不是 driver |
| **`st3215-compat-bridge` 方向** | **双向 Bridge (pattern)**：commands 上行 + snapshots 下行 | 反映真实数据流向 |
| **协议库位置** | **`st3215-wire` 独立 crate**，零 tokio/normfs/station 依赖 | codex #5：纯协议层 |
| **机器人 SKU preset** | **`st3215-compat-bridge/presets/*.yaml` 数据文件** | codex #5：不进 real driver crate |
| **Real vs sim 关系** | **`bus_serial` 命名空间共存**，无 mutual exclusion | codex §2.1：shadow mode 第一类 |
| **Config 根** | `sim-runtime` + `bridges` 两个新顶级段；`drivers` 零修改 | 既有 `station.yaml` 零破坏 |
| **MJCF 来源** | **`gen.py` + `world.yaml` manifest**，一份文件两用途 | 零一致性风险 |
| **URDF mimic → MJCF** | **`<equality>` polycoef** | MuJoCo 不支持 `<mimic>` |
| **Gripper 表达** | **`CAP_GRIPPER_PARALLEL` capability + 归一化 0..1**，primary joint + 两个 mimic | 向外表达和内部物理解耦 |
| **Python 包 ST3215 痕迹** | **零**（CI 强制 grep 零命中） | codex §6.1 根问题的硬防线 |
| **健康感知** | **`/sim/health` queue 广播**，bridge 订阅 | codex #4：零隐式副通道 |
| **跨语言 correlation** | **`world_tick` 作为主键** | 最小可用；不引入 OTel |
| **错误哲学** | **启动期快错、运行期软错**（延续 v1） | 启动沉默失败最糟糕 |
| **恢复策略** | **无自动重连 / 无自动重启**（延续 v1）| 自动恢复掩盖真 bug |
| **CI** | **MVP 不做 GitHub Actions**，只 Makefile target + grep 架构不变量 | MuJoCo wheel 在 runner 上依赖未知 |

---

## 5. 整体架构

### 5.1 三层与三进程

```
 ┌───────────────────────────────────────────────────────────────────┐
 │                     Station (Rust 二进制)                          │
 │                                                                    │
 │   NormFS   ◄────────┐                                             │
 │                     │                                             │
 │   SimulationRuntime │  (✨ 一等 subsystem，与 NormFS 平级)         │
 │   ├─ WorldBackend (trait, pub(crate))                             │
 │   │   ├─ ChildProcessBackend  (MVP-1)                             │
 │   │   ├─ ExternalSocketBackend (MVP-1)                            │
 │   │   └─ MockBackend  (test-only)                                 │
 │   ├─ WorldClock (tick canonical + sim/wall time 辅助)             │
 │   ├─ ActuatorRegistry (robot_id → actuator_ref → capability)      │
 │   ├─ SensorRegistry                                               │
 │   ├─ SnapshotBroker (broadcast WorldSnapshot to subscribers)      │
 │   ├─ ActuationSender (QoS lane routing)                           │
 │   └─ HealthStream (→ /sim/health NormFS queue, 1Hz + events)      │
 │                                                                    │
 │   现有 drivers (零修改)：                                           │
 │   ├─ st3215         (真机；和 sim 互不感知)                        │
 │   ├─ usbvideo, sysinfod, motors-mirroring, inferences             │
 │                                                                    │
 │   Bridges (订阅 SimulationRuntime 公开 API, 不是 driver)           │
 │   └─ St3215CompatBridge  (MVP-1)                                  │
 │       ├─ command_task (commands queue → filter → ActuationBatch)  │
 │       ├─ state_task   (WorldSnapshot → st3215/inference bytes)    │
 │       └─ health_task  (/sim/health → st3215/meta offline marker)  │
 │                                                                    │
 │   Web UI / TCP / WebSocket  (零修改)                               │
 └──────────────────────────────┬────────────────────────────────────┘
                                │ UDS + length-prefixed protobuf
                                │ schema: 通用 World/Actuation/Sensor
                                │         零硬件术语
                                ▼
 ┌───────────────────────────────────────────────────────────────────┐
 │  norma_sim (Python 子进程, ZERO ST3215 痕迹)                       │
 │                                                                    │
 │  world/                                                            │
 │  ├─ manifest.py       (加载 world.yaml)                            │
 │  ├─ model.py          (MjModel/MjData + 锁)                        │
 │  ├─ descriptor.py     (manifest → WorldDescriptor proto)           │
 │  ├─ actuation.py      (ActuationCommand → data.ctrl)               │
 │  ├─ snapshot.py       (data → WorldSnapshot proto)                 │
 │  └─ capabilities.py   (★ 唯一含能力语义的模块)                     │
 │                                                                    │
 │  scheduler/                                                        │
 │  ├─ base.py           (WorldScheduler protocol)                    │
 │  └─ realtime.py       (RealTimeScheduler, MVP-1 唯一实现)          │
 │                                                                    │
 │  ipc/                                                              │
 │  ├─ framing.py / codec.py                                          │
 │  └─ server.py / session.py                                         │
 └───────────────────────────────────────────────────────────────────┘
```

### 5.2 运行时不变量

1. **Station 主循环对现有 drivers 零触碰**——所有现有的 driver loader / queue / TCP/WS / Web UI 都不改动
2. **`SimulationRuntime` 由 Station 显式持有**（`Option<Arc<SimulationRuntime>>` 字段），启动 / 关闭顺序受 `main.rs` 显式控制
3. **`St3215CompatBridge` 和 real `st3215` driver 互不感知**，通过 `bus_serial` 命名空间共存
4. **`/sim/health` queue 是 sim 死活的唯一信号源**，消费者通过 NormFS subscribe 感知
5. **Python `norma_sim` 零 ST3215 痕迹**（CI 强制）
6. **`WorldBackend` trait 是 `pub(crate)`**（subsystem 边界编译期强制）
7. **`world_tick` 是跨语言 correlation 唯一主键**
8. **`TempRuntimeDir` 是唯一 on-disk 状态载体**，RAII 管理，整个 filesystem 副作用就一个目录

### 5.3 为什么是 subsystem 不是 driver

`SimulationRuntime` 不是 driver，因为：

1. **生命周期**：driver 是 `Station::start_drivers()` 生命期内批量启动的"硬件适配器"，其退出与 Station 生命期耦合弱。Runtime 需要 spawn 子进程 + 完成 handshake + 提供共享世界——它的生命周期需要**在 drivers 之前**启动、**在 drivers 之后**关闭
2. **服务对象**：driver 是 web UI / inference / Python client 的硬件桥梁（一对多）。Runtime 是**多个 bridge + 多个 driver + 多个未来消费者**的共享世界（N 对 N 共享）
3. **抽象层级**：driver 的 contract 是 NormFS queue + protobuf message（下游契约）。Runtime 的 contract 是 Rust 公开 API（`subscribe_snapshots()` / `send_actuation()` / `subscribe_health()`）——Rust-level API 不适合通过 driver 抽象暴露
4. **概念**：real `st3215` driver 的职责是"和一条真实 ST3215 总线说话"。如果让 sim 也戴 driver 帽子，等于承认"和一条虚构 ST3215 总线说话"是同级概念——这正是 v1 的概念断裂

---

## 6. 组件设计（Rust 侧）

### 6.1 `sim-runtime` crate — `SimulationRuntime` subsystem

**位置**：`software/sim-runtime/`

**模块布局**：

```
sim-runtime/src/
├── lib.rs                  # re-export 公共 API
├── runtime.rs              # SimulationRuntime 主结构 + start/shutdown
├── config.rs               # SimRuntimeConfig
├── clock.rs                # WorldClock + tick helpers
├── registry.rs             # ActuatorRegistry / SensorRegistry
├── snapshot_broker.rs      # broadcast<Arc<WorldSnapshot>>
├── actuation_sender.rs     # QoS lane routing
├── health.rs               # SimHealth + 发布到 /sim/health queue
├── supervisor.rs           # backend wait_task + 死亡事件广播
├── errors.rs               # SimRuntimeError
├── backend/
│   ├── mod.rs              # WorldBackend trait (pub(crate))
│   ├── child_process.rs    # ChildProcessBackend (MVP-1 prod)
│   ├── external_socket.rs  # ExternalSocketBackend (MVP-1 prod)
│   ├── mock.rs             # MockBackend (#[cfg(test)])
│   ├── transport.rs        # UnixSocketTransport (shared helper)
│   └── runtime_dir.rs      # TempRuntimeDir RAII
├── ipc/
│   ├── mod.rs
│   ├── framing.rs          # LengthDelimitedCodec 包装
│   ├── codec.rs            # Envelope encode/decode (prost)
│   └── handshake.rs        # Hello/Welcome 流程
└── proto/world.rs          # prost 生成
```

**公共 API**（`runtime.rs`）：

```rust
pub struct SimulationRuntime { /* 内部 */ }

impl SimulationRuntime {
    /// 启动：构造 backend → start() 等 handshake → 注册 /sim/health queue → 返回
    pub async fn start(
        normfs: Arc<NormFS>,
        station_engine: Arc<dyn StationEngine>,
        config: SimRuntimeConfig,
    ) -> Result<Arc<Self>, SimRuntimeError>;

    /// 优雅关闭：发 Goodbye → 关 broadcast → backend.shutdown → drop TempRuntimeDir
    pub async fn shutdown(self: Arc<Self>) -> Result<(), SimRuntimeError>;

    /// 启动时 backend 在 handshake 中发回的 world 自描述
    pub fn world_descriptor(&self) -> &WorldDescriptor;

    /// 订阅 WorldSnapshot 流（tokio broadcast Receiver）
    pub fn subscribe_snapshots(&self) -> broadcast::Receiver<Arc<WorldSnapshot>>;

    /// 发 ActuationBatch，按 lane 路由：
    /// - QOS_LOSSY_SETPOINT：fire-and-forget，溢出 drop oldest
    /// - QOS_RELIABLE_CONTROL：溢出返回 Err(Backpressure)
    pub async fn send_actuation(&self, batch: ActuationBatch) -> Result<(), SimRuntimeError>;

    /// 订阅 SimHealth 流
    pub fn subscribe_health(&self) -> broadcast::Receiver<SimHealth>;
}
```

**关键设计点**：

- **4 个公开方法 + 构造/析构** 就是 `sim-runtime` 对外的全部表面积。Bridge 代码里**不出现** backend / ipc / framing 这些词
- **`WorldBackend` trait 是 `pub(crate)`**——不对外暴露。未来添加新 backend 的方式是进 sim-runtime crate 本身，不是第三方 crate 实现 trait。这是对 subsystem 边界的编译期强制
- **`ActuationSender` 按 QoS lane 分通道**：两个独立的 mpsc，`QOS_LOSSY_SETPOINT` bounded + drop-oldest，`QOS_RELIABLE_CONTROL` bounded + block/error
- **`SnapshotBroker` 用 tokio `broadcast`**：多消费者天然支持，溢出时消费者得到 `Lagged(n)` 事件，bridge 能检测并 log
- **`HealthStream` 同时发**到：(a) 内部 `broadcast` 给 Rust 订阅者，(b) `/sim/health` NormFS queue 给跨进程 / 未来 web UI
- **`WorldDescriptor` 持久化到 `/sim/descriptor` NormFS queue**：见下方详述

**`WorldDescriptor` 的 NormFS 持久化**（离线 replay 的前置要求）：

Capability-keyed schema 让消息语义**天然依赖 descriptor** —— 一条 `SetPosition{value=0.5}` 在没有 descriptor 的情况下无法解释（0.5 是 rad？m？还是 gripper 的 0..1？）。live session 里 descriptor 在内存；但如果把全局 `commands` queue 或 `st3215/inference` queue 归档后想 offline 分析，没 descriptor 就是半盲。

**解法**：`SimulationRuntime::start()` 完成 handshake 后，**立即把 `WorldDescriptor` 作为一条 message 写入 `/sim/descriptor` NormFS queue**（runtime 负责注册这条 queue）。特性：

- **一次写入**：每次 `SimulationRuntime::start()` 写一条 `WorldDescriptor` message（带 `runtime_session_id`）
- **持久化**：NormFS 既有的存档机制会把它保留下来，离线 replay 能读到历史所有 sessions 的 descriptor
- **订阅者**：bridges / 未来 web UI / 未来 replay 工具都可以通过 NormFS subscribe 读到当前 session 的 descriptor，不必走 `sim_runtime.world_descriptor()` Rust API（那只对进程内 consumer 可用）
- **Replay 时的对齐键**：离线工具读一批全局 `commands` queue（filter by `StationCommandType::StcSt3215Command` + `target_bus_serial` 前缀为 `sim://`）里某段时间的 entries，然后读 `/sim/descriptor` 里时间对齐的 descriptor（通过 `written_at_wall_ns` 找最近的那条），就能反向解释每条命令的单位和语义

**queue schema**：`/sim/descriptor` 每条 entry 就是一个 `WorldDescriptor` protobuf message（§6.5 定义），加上 runtime 补的 wrapper 字段 `runtime_session_id` 和 `written_at_wall_ns`（用 Station 的 CLOCK_MONOTONIC）。

**与 `/sim/health` 的关系**：两条 queue 互补。`/sim/health` 是运行时**动态状态**，`/sim/descriptor` 是会话**静态契约**。Bridge 只订阅 health，离线工具只读 descriptor + 归档的 commands/snapshots。

**MockBackend 的用途**（`#[cfg(test)]` 在 `sim-runtime` 内部可见）：用于写 `sim-runtime` 的**纯 Rust 单元测试**——不 spawn 子进程也不要真 UDS，用一对 `mpsc::Channel` 手工注入 "backend 收到的 Envelope" 并观察 "runtime 发了什么"。典型场景：`test_handshake_happy_path`（手工注入 `Welcome`）、`test_actuation_sender_lossy_lane_drop_oldest`（观察 outbound mpsc 溢出行为）、`test_backend_crash_broadcasts_health`（手工触发 `BackendTermination::Crashed`）。集成测试（真起子进程的那种）应该用 `ChildProcessBackend` + 假 launcher 指向 fixture script，不用 `MockBackend`。

**估计代码量**：~1700 行（含测试）

### 6.2 `st3215-wire` crate — 纯协议库

**位置**：`software/drivers/st3215-wire/`

**设计原则**：无 tokio / 无 normfs / 无 station-iface / 无 log 宏依赖。纯 `bytes::Bytes` + `u8` 数据操作。可以被 `no_std` 项目使用（虽然 MVP 不追求）。

**模块布局**：

```
st3215-wire/src/
├── lib.rs
├── register.rs    # RamRegister / EepromRegister enum + 地址常量
├── layout.rs      # EEPROM (0x00..0x28) + RAM (0x28..0x47) 的 71 字节布局
├── units.rs       # steps ↔ rad, sign-magnitude 15bit, load scaling
├── unpack.rs      # 从 71 字节 dump 解析到语义字段（从 st3215::port 迁入）
├── pack.rs        # ✨ 新：从语义字段打包到 71 字节 dump
├── presets.rs     # MotorModelSpec (ST3215 舵机型号级常量，非机器人 SKU)
└── tests.rs       # 含 pack ↔ unpack 往返测试
```

**关键 API**：

```rust
/// 描述一颗 ST3215 舵机的静态属性（型号级，非机器人 SKU 级）
pub struct MotorModelSpec {
    pub model_number: u16,      // ST3215 = 777
    pub firmware_version: u8,   // = 10
    pub baud_rate_code: u8,
    pub steps_per_rev: u32,     // = 4096
    pub default_eeprom: [u8; 40],  // 出厂默认 EEPROM 映射
}

pub const ST3215_STANDARD: MotorModelSpec = MotorModelSpec { /* ... */ };

/// 打包：语义状态 → 71 字节 EEPROM+RAM dump
pub fn pack_state_bytes(
    motor_id: u8,
    spec: &MotorModelSpec,
    instance: &MotorInstance,  // 机器人 SKU 级参数（min/max/offset/torque_limit）
    state: &MotorSemanticState,  // 语义状态（rad、rad/s、nm、温度...）
) -> bytes::Bytes;

/// 解包：71 字节 dump → 语义状态
pub fn unpack_state_bytes(
    bytes: &[u8],
    spec: &MotorModelSpec,
    instance: &MotorInstance,
) -> Result<MotorSemanticState, UnpackError>;

pub struct MotorInstance {
    pub min_angle_steps: u16,
    pub max_angle_steps: u16,
    pub offset_steps: i16,
    pub torque_limit: u16,
    pub voltage_nominal_v: f32,
}

pub struct MotorSemanticState {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub load_nm: f32,
    pub temperature_c: f32,
    pub torque_enabled: bool,
    pub moving: bool,
    pub goal_position_rad: f32,
    pub goal_speed_rad_s: f32,
}
```

**`MotorModelSpec` vs `MotorInstance` 的分界**：

- `MotorModelSpec` 是 ST3215 **舵机型号**的常量（所有 ST3215 都一样）。出厂在 `st3215-wire` 里 hardcode `ST3215_STANDARD`
- `MotorInstance` 是**一颗具体舵机**的参数（min/max/offset/torque_limit 等），不同机器人 SKU 不同，甚至同一 SKU 的不同关节也不同
- 机器人 SKU 到 `MotorInstance` 的映射**不**在 `st3215-wire` 里。它在 `st3215-compat-bridge/presets/*.yaml` 数据文件里

**从 `st3215` real driver 迁入的内容**：

| 现有 `st3215` 路径 | 迁入 `st3215-wire` 的部分 | 说明 |
|---|---|---|
| `src/protocol/memory.rs` | `register.rs` + `layout.rs` | EEPROM/RAM enum + 地址常量 |
| `src/protocol/units.rs` | `units.rs` | steps↔rad, sign-magnitude, load scaling |
| `src/protocol/packet.rs` | `packet.rs`（可选）| 帧协议；如果 real driver 的 `port.rs` 能继续用 `st3215-wire::packet`，一并迁入；否则留在 `st3215` |
| `src/port.rs`（parsing 段）| `unpack.rs` | ★ 注：`port.rs` 是 ~1182 行的 I/O-dense 文件，parse 逻辑**嵌在方法里**（如 `scan_motors`、状态读取方法），不是顶层 `pub fn parse_*`。迁移时需要把"读 71 字节后解析"的纯函数部分提取出来——这是 code extraction，不是简单的 file move |
| `src/protocol/devices.rs` | **留在 `st3215`** | USB device matching (`is_st3215_usbdevice`)——有 `udev`/`rusb` 依赖，属于 I/O 层 |
| inline 舵机型号常量 | `presets.rs::ST3215_STANDARD` | 舵机型号级常量（`model_number=777`、`firmware_version=10`、`steps_per_rev=4096` 等）|

留在 `st3215`（不动）：**串口 I/O + 主控制循环 + NormFS 注册 + auto-calibration + USB 发现**——这些是 real driver 的 tokio/io/udev 密集部分，不是纯协议。

**`MotorInstance` 字段的当前位置**（迁移起点）：

`MotorInstance` 的字段（`min_angle_steps` / `max_angle_steps` / `offset_steps` / `torque_limit` / `voltage_nominal_v`）**当前不在 `st3215/src/presets.rs`**——`presets.rs` 只有 `DEFAULT_TORQUE_LIMIT` / `DEFAULT_MAX_TORQUE` / `PidConfig` / `ELROBOT_PID` / `SO101_PID` 这类舵机型号级常量，不含机器人 SKU 级 per-motor 数据。

- `min_angle_steps` / `max_angle_steps` 当前活在 `src/state.rs` 的 `MotorBounds: HashMap<String, HashMap<u32, (u32, u32, bool)>>` 和 `src/auto_calibrate/{elrobot,so101}.rs` 的 SKU-specific calibration 数据里
- `offset_steps` 类似，分散在 calibration 数据
- `voltage_nominal_v` 是**新概念**，当前 real driver 不使用

因此迁移路径不是"从一个文件搬到另一个文件"，而是**从多处汇总到一个结构**。操作：
1. 定义 `st3215-wire::MotorInstance` 新结构
2. 从 `auto_calibrate/elrobot.rs` 抽出 ElRobot follower 每关节的 min/max/offset 数值
3. 把这些数值写入 `st3215-compat-bridge/presets/elrobot-follower.yaml`（人类编辑的数据文件）
4. Real driver 继续通过 `state.rs::MotorBounds` 内部跑 calibration，**不动**——它的运行时路径不需要 `MotorInstance`

**`st3215` real driver 的改动**：

- `Cargo.toml` 加 `st3215-wire = { path = "../st3215-wire" }`
- 删 inline 的 register 常量和 unit 函数
- 改 `use` 语句让 parse 走 `st3215_wire::unpack`
- 既有测试**全部保留**——它们是 wire 迁移的 regression guard
- **估计差异**：~300 行代码位置移动，几十行 `use` 改动。**纯 code move，无逻辑变化**

**估计代码量**：~500 行新代码 + ~300 行移动 = ~800 行总计

### 6.3 `st3215-compat-bridge` crate — 双向 Bridge

**位置**：`software/sim-bridges/st3215-compat-bridge/`（独立顶级目录 `sim-bridges/`，不塞 `drivers/`）

**模块布局**：

```
st3215-compat-bridge/
├── Cargo.toml                             # deps: sim-runtime, st3215-wire, normfs, station-iface
├── src/
│   ├── lib.rs                             # start_st3215_compat_bridge 入口
│   ├── config.rs                          # BridgeConfig
│   ├── actuator_map.rs                    # robot_id + actuator_id ↔ (bus_serial, motor_id)
│   ├── preset_loader.rs                   # yaml → RobotPreset
│   ├── command_task.rs                    # 任务 A: 全局 commands queue → filter → ActuationBatch
│   ├── state_task.rs                      # 任务 B: WorldSnapshot → st3215/inference
│   ├── health_task.rs                     # 任务 C: SimHealth → st3215/meta offline marker
│   └── errors.rs
└── presets/
    └── elrobot-follower.yaml              # 机器人 SKU 数据文件
```

**启动入口**：

```rust
pub async fn start_st3215_compat_bridge(
    normfs: Arc<NormFS>,
    station_engine: Arc<dyn StationEngine>,
    sim_runtime: Arc<SimulationRuntime>,
    config: St3215CompatBridgeConfig,
) -> Result<Arc<BridgeHandle>, BridgeError>;
```

**Bridge 是三个并发 task 的集合**：

1. **command_task**（上行；真实 legacy 契约，非发明的 queue 名）：
   - 订阅 **`station_iface::COMMANDS_QUEUE_ID = "commands"`**——这是 Station 唯一的全局 commands queue，和真 `st3215` driver (`driver.rs:69`) 订阅的是**同一条 queue**
   - Callback 里的过滤流程**逐字 mirror** 真 driver 的逻辑（`driver.rs:74-87`）：
     ```rust
     let pack = commands::StationCommandsPack::decode(data)?;
     for cmd in &pack.commands {
         if cmd.r#type() != drivers::StationCommandType::StcSt3215Command { continue; }
         let st_command = st3215_proto::Command::decode(cmd.body.clone())?;
         if st_command.target_bus_serial != config.legacy_bus_serial { continue; }
         // ↑ 这条过滤让 bridge 只吃自己这条 sim bus 的命令；
         //   真 driver 会吃匹配自己那条真 bus 的命令。两者按 target_bus_serial 天然分流
         let batch = translate_st3215_command_to_generic(&st_command, &actuator_map, &preset)?;
         sim_runtime.send_actuation(batch).await?;
     }
     ```
   - **`target_bus_serial` 过滤是 shadow mode 正常工作的关键**：真机 bus serial（例如 `ST3215-BUS-A1B2C3`）和 bridge 的 `sim://elrobot-shadow` 自然分流——同一条全局 `commands` queue 里两套命令按 target 各走各路
   - **`translate_st3215_command_to_generic`** 的职责：解析 `st_command.{write, reg_write, sync_write, reset, torque_enable, ...}`，根据寄存器地址识别语义（`GoalPosition=0x2A` → `SetPosition`；`TorqueEnable=0x28` → `EnableTorque/DisableTorque`；`Reset`→`ResetActuator`），用 `st3215-wire::units` 反向把 step 值换算成 rad（或对 gripper 反向到 0..1 归一化）
   - QoS lane 路由：位置类命令 → `QOS_LOSSY_SETPOINT`；`EnableTorque/DisableTorque/Reset` → `QOS_RELIABLE_CONTROL`
   - **零改动 station_py / web UI 的保证**：因为 bridge 订阅的是 **真实既有** `commands` queue，现有客户端发命令的路径（`station_py/example_commands.py` 写到 `commands` queue）**一字不改**就能同时驱动真机和 sim
   - 错误处理：decode 失败 → log warn 丢这条继续；未知 motor_id → log warn 丢这条继续；lane 溢出 → log error 丢这条继续；**session 级隔离**，一条坏命令不影响后续

2. **state_task**：
   - 调用 `sim_runtime.subscribe_snapshots()` 取 broadcast Receiver
   - 循环 `recv()` WorldSnapshot
   - 对每个 actuator state：查 `actuator_map` 反向映射；调 `st3215_wire::pack_state_bytes(motor_id, spec, instance, state)`；组装 `MotorState` protobuf；写 `st3215/inference` NormFS queue
   - 使用 Station **自己的** `CLOCK_MONOTONIC`（不是 snapshot 里带的 `wall_time_ns`）作为 inference 元数据的时间戳
   - 错误处理：`Err(Closed)` = runtime 干净终止，return Ok(()) 退出；`Err(Lagged(n))` = log warn 继续；pack 失败 = log error 丢这条继续

3. **health_task**：
   - 调用 `sim_runtime.subscribe_health()` 取 broadcast Receiver
   - 接收 `SimHealth` 事件，关键是识别 `backend_alive: false` + `termination: Some(_)`
   - 触发时：写一条 "bus offline marker" 到 `st3215/meta` queue，让 web UI 显示总线离线
   - 然后 return `Err(BridgeError::BackendTerminated(reason))` 结束 task

**`actuator_map.rs`**：基于加载的 `RobotPreset` 构造的双向查表：
- `actuator_id: str → motor_id: u8` 和反向
- `actuator_id: str → MotorInstance`（min/max/offset/torque_limit）

**`preset_loader.rs`**：加载 `elrobot-follower.yaml` → `RobotPreset` 结构。字段严格类型化，不兼容旧字段立即报错。

**`elrobot-follower.yaml` 格式**：

```yaml
robot_id: elrobot_follower
legacy_bus_serial: "sim://bus0"           # default，可被 BridgeConfig 覆盖
motors:
  - actuator_id: rev_motor_01
    motor_id: 1
    min_angle_steps: 0
    max_angle_steps: 4095
    offset_steps: 2048
    torque_limit: 500
    voltage_nominal_v: 12.0
  # ... motor_02 ~ motor_07 同构 ...
  - actuator_id: rev_motor_08              # gripper primary
    motor_id: 8
    min_angle_steps: 0
    max_angle_steps: 4095
    offset_steps: 0
    torque_limit: 500
    voltage_nominal_v: 12.0
```

**启动期校验**（代码中强制）：

1. `sim_runtime.world_descriptor().robots` 必须包含 config 指定的 `robot_id`——否则 `BridgeError::RobotNotInWorld`
2. 每个 `preset.motors[i].actuator_id` 必须在 world descriptor 的 robot 里存在对应 actuator——否则 `BridgeError::ActuatorNotInWorld`
3. **`config.legacy_bus_serial` 必须以 `sim://` 为前缀** —— 这是**结构性不变量**，见下方 §6.3.a 详述
4. `preset.motors.len()` 必须和 bridge config 期望的 motor count 一致

所有启动期校验失败都是**快错 + 明确错误消息**。

### 6.3.a Shadow Mode Bus Namespace — `sim://` 前缀不变量

**问题**：Shadow 模式下 real `st3215` driver 和 `st3215-compat-bridge` 都往 `st3215/inference` / `st3215/meta` queue 写数据，必须有机制保证两者的 `bus_serial` **永远不会冲突**。

**v1 拒绝方案**：启动时扫 `st3215/meta` queue 看已注册的 bus serial——但 real driver 的 bus 是**异步**发现的（`driver.rs:132` 每 1 秒扫一次串口），bridge 启动时可能 real bus 还没被发现，check 通过后 real bus 才出现，两者在运行时产生冲突。这是 racy 的。

**v2 采用**：**`legacy_bus_serial` 必须以 `sim://` 为前缀**。

**原因**（为什么这条规则结构性排除冲突）：

1. 真 ST3215 硬件的 bus serial 来源于 `driver.rs` 里 `create_bus_info(&port_info)`——它基于 USB 设备的枚举信息（vendor/product ID + serial number 字段）。USB 设备的 serial number 字段**不可能**出现 `sim://` 字符串（URL scheme 不合 USB 设备命名规范）
2. `sim://` 是项目自留的 URL-scheme-like 前缀，**bridge crate 独占**。未来其它 sim bridge（`usbvideo-compat-bridge` 等）同样使用 `sim://` 前缀（例如 `sim:///dev/video-wrist`）

**实现**（`preset_loader.rs` + bridge 启动）：
- Config 加载时校验：`if !config.legacy_bus_serial.starts_with("sim://") { return Err(BridgeError::InvalidLegacyBusSerial) }`
- Preset yaml 加载时校验：preset 里如果覆盖 `legacy_bus_serial`，同样要求 `sim://` 前缀
- Real `st3215` driver 无需任何改动——它的 bus serial 天然不会有 `sim://` 前缀

**这不是 "防御式编程" 是结构性保证**：
- **不做** runtime 冲突检测（不查 `st3215/meta`）——没必要，规则让冲突无法发生
- **不做** 显式 registry——没必要，规则让 registry 无状态可管
- operator 在 yaml 里写 `legacy_bus_serial` 只能写 `sim://xxx` 形式，否则启动失败
- 真机 bus serial 自然落在 `sim://` 之外的所有其它字符串空间里

**启动期错误示例**：
```
ERROR: st3215-compat-bridge: invalid legacy_bus_serial "my-custom-bus"
in BridgeConfig or preset yaml. Bus serial MUST start with "sim://"
to ensure structural separation from hardware bus serials.
Example: "sim://bus0" or "sim://elrobot-shadow".
```

**与 codex 之前的 race 批评的对账**：v1-ish 的"启动时扫 meta queue 做 collision check"是 racy 的弱机制；v2 用**命名空间硬规则**把问题从"检测冲突"降级为"冲突不可能发生"——无需 detection，无需 registry，零运行时状态。

**估计代码量**：~890 行（含测试 + yaml + preset loader）

### 6.4 Station `main.rs` 增量

既有 `main.rs` 是 flat `#[tokio::main] async fn main()`（在 `software/station/bin/station/src/main.rs`）。v2 在其中**注入 ~80 行**，不重构既有结构。

**既有代码小注**：
- `env_logger` init **在 `Station::new()` 内部**（`main.rs:96`），不在 `main()` 顶层。sim-runtime 启动时日志已经可用，无需重排
- `main()` 现有路径（简化）：`parse args → Station::new → start_main_queue → Inference::start_queue → start_commands_queue → Inference::start + *station.engine.inference.lock() = Some(inference) → start_drivers → tcp/web → ctrl_c → station.shutdown()`
- `Station::shutdown()` 是**唯一的既有关闭方法**（`main.rs:297`），只负责停 `usbvideo` 实例 + 关 NormFS。**没有**单独的 `shutdown_drivers()` 方法。其它 driver（`st3215` / `sysinfod` / `motors-mirroring`）没有显式关闭路径，它们以 tokio task 形式运行，在 runtime 退出时被 abort。这是既有行为，本 spec 不重构

**启动顺序**：

```rust
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let mut station = Station::new(&args).await?;           // 既有（内含 env_logger init）

    // ✨ 新增：config validation
    station.config.validate()?;

    station.start_main_queue().await?;                       // 既有
    inference::Inference::start_queue(&station.normfs).await?;  // 既有
    station.start_commands_queue().await?;                   // 既有

    // 既有：Inference 实体装载（main.rs:348）
    let inference_instance = inference::Inference::start(
        station.normfs.clone(),
        // ... 参数照旧 ...
    ).await?;
    *station.engine.inference.lock() = Some(inference_instance);

    // ✨ 新增：SimulationRuntime 在 drivers 之前启动
    //          （start_drivers 之后 bridges 之前）
    if let Some(sim_cfg) = &station.config.sim_runtime {
        if sim_cfg.enabled {
            let sim_runtime = SimulationRuntime::start(
                station.normfs.clone(),
                station.engine.clone(),       // Arc<Engine> → Arc<dyn StationEngine> 自动协变
                sim_cfg.clone(),
            ).await?;
            station.sim_runtime = Some(sim_runtime);
        }
    }

    station.start_drivers().await?;                          // 既有，零增量

    // ✨ 新增：bridges 在 drivers 之后启动
    if let Some(bridge_cfg) = &station.config.bridges.st3215_compat {
        if bridge_cfg.enabled {
            let sim_rt = station.sim_runtime.as_ref()
                .ok_or(StationError::BridgeRequiresSimRuntime)?;
            let bridge = start_st3215_compat_bridge(
                station.normfs.clone(),
                station.engine.clone(),
                sim_rt.clone(),
                bridge_cfg.clone(),
            ).await?;
            station.bridges.push(bridge);
        }
    }

    // 既有：tcp / web / ctrl_c loop
    // ...

    // ✨ 新增：有序 shutdown
    //  顺序原则：bridges (new) → sim_runtime (new) → station.shutdown() (既有，清 usbvideo + NormFS)
    //  注：sim_runtime 必须在 NormFS 关闭（即 station.shutdown 内部）之前完成，
    //       因为 sim_runtime 在 shutdown 时要往 /sim/health queue 写最后一条 backend_alive=false
    for bridge in station.bridges.drain(..) {
        bridge.shutdown().await?;
    }
    if let Some(rt) = station.sim_runtime.take() {
        rt.shutdown().await?;
    }
    station.shutdown().await?;                               // 既有：usbvideo + normfs.close()

    Ok(())
}
```

**关于既有 drivers 的关停**：现有的 `st3215` / `sysinfod` / `motors-mirroring` / `inferences` driver 都没有显式 shutdown 路径——它们在 `start_drivers()` 里被 `tokio::spawn`，返回的 JoinHandle 不被保留。当 `#[tokio::main]` runtime 退出时 task 被 abort。**本 spec 不修正这个**（见 §17.2 P4 「v2 已有结构性债务但 MVP-1 不处理」）。未来给 real driver 加显式 shutdown 时，应该采用 v2 的 bridge/sim_runtime pattern（显式 handle + `shutdown().await`）。

**`Station` struct 新字段**：

```rust
pub struct Station {
    // ... 既有字段 ...
    pub config: Config,                             // 既有（需改 pub）
    pub normfs: Arc<NormFS>,                        // 既有
    pub engine: Arc<Engine>,                        // 既有，concrete struct（见下方说明）

    // ✨ 新增
    pub sim_runtime: Option<Arc<SimulationRuntime>>,
    pub bridges: Vec<Arc<BridgeHandle>>,
}

// 既有（main.rs 内定义）：
// struct Engine { main_queue: ..., inference: Mutex<Option<inference::Inference>> }
// impl station_iface::StationEngine for Engine { fn register_queue(&self, ...) { ... } }
```

**关于 `Arc<Engine>` → `Arc<dyn StationEngine>` 的协变**：`sim-runtime` crate 的 `SimulationRuntime::start()` 签名接受 `Arc<dyn StationEngine>`（§6.1），而 `Station` 的 `engine` 字段是具体类型 `Arc<Engine>`。Rust 的 unsized coercion 规则保证 `Arc<Engine> → Arc<dyn StationEngine>` 会在函数调用处自动发生（因为 `Engine: StationEngine` 且 `StationEngine` 是 object-safe 的，只有 `register_queue` 一个方法无 generics 无 Self return）。callsite 代码 `SimulationRuntime::start(station.engine.clone(), ...)` 无需显式 `as` 转换。

**启动/关闭顺序强不变量**：

- Startup: `NormFS` → `SimulationRuntime`（如启用）→ `drivers` → `bridges`（如启用）→ `tcp/web`
- Shutdown: `tcp/web` → `bridges` → `drivers` → `SimulationRuntime` → `NormFS`

**理由**：
- `SimulationRuntime` 必须在 drivers 之前启动，因为 bridges 要拿 `Arc<SimulationRuntime>`
- `bridges` 必须在 runtime 之前关闭，否则 bridge 的 `subscribe_snapshots()` 会在不预期时刻拿到 `Closed`，触发错误路径而不是正常退出
- Runtime 必须在 NormFS 之前关闭，因为 runtime 要写最后一条 `backend_alive=false` 到 `/sim/health`

### 6.5 `protobufs/sim/world.proto` — Capability-Keyed Schema

**位置**：`protobufs/sim/world.proto`（新目录 `sim/`，package `norma_sim.world.v1` 显式版本号）

**核心 schema**：

```protobuf
syntax = "proto3";
package norma_sim.world.v1;

// —— Refs：字符串寻址，不是 bus+motor_id ——

message RobotRef     { string robot_id = 1; }                     // "elrobot_follower"
message ActuatorRef  { string robot_id = 1; string actuator_id = 2; }  // e.g. "elrobot_follower" + "rev_motor_01"
message SensorRef    { string robot_id = 1; string sensor_id   = 2; }

// —— 时间基准：所有帧都带 ——

message WorldClock {
  uint64 world_tick   = 1;  // ★ monotonic sim tick，独立于 wall clock，canonical time
  uint64 sim_time_ns  = 2;  // 推导：world_tick * physics_timestep_ns
  uint64 wall_time_ns = 3;  // sim host 的 CLOCK_MONOTONIC；跨进程无意义，禁用作对齐
}

// —— Actuator 能力：capability 决定单位和语义 ——

message ActuatorCapability {
  enum Kind {
    CAP_UNSPECIFIED        = 0;
    CAP_REVOLUTE_POSITION  = 1;  // value = rad
    CAP_PRISMATIC_POSITION = 2;  // value = m
    CAP_GRIPPER_PARALLEL   = 3;  // value = 0..1 normalized
  }
  Kind kind              = 1;
  double limit_min       = 2;    // 单位由 kind 决定
  double limit_max       = 3;
  double effort_limit    = 4;    // N·m or N
  double velocity_limit  = 5;
}

message SensorCapability {
  enum Kind {
    SENSE_UNSPECIFIED = 0;
    SENSE_JOINT_STATE = 1;       // MVP-1
    SENSE_CAMERA_RGB  = 2;       // MVP-2
    SENSE_IMU_6DOF    = 3;       // future
  }
  Kind kind = 1;
}

// —— Actuation 命令：capability-keyed，不是硬件寄存器 ——

message ActuationCommand {
  ActuatorRef ref = 1;
  oneof intent {
    SetPosition    set_position    = 10;
    SetVelocity    set_velocity    = 11;
    SetEffort      set_effort      = 12;
    EnableTorque   enable_torque   = 13;
    DisableTorque  disable_torque  = 14;
    ResetActuator  reset_actuator  = 15;
  }
}

message SetPosition   { double value = 1; double max_velocity = 2; }  // 单位由 capability 决定
message SetVelocity   { double value = 1; }
message SetEffort     { double value = 1; }
message EnableTorque  {}
message DisableTorque {}
message ResetActuator {}

// —— Actuator state：publish 时一并回报 ——

message ActuatorState {
  ActuatorRef ref = 1;
  double position_value      = 2;  // 单位 = capability
  double velocity_value      = 3;
  double effort_value        = 4;
  bool   torque_enabled      = 5;
  bool   moving              = 6;
  double goal_position_value = 7;
}

// —— Sensor sample：多 payload type ——

message SensorSample {
  SensorRef ref = 1;
  oneof payload {
    JointStateSample joint_state  = 10;  // MVP-1
    CameraFrame      camera_frame = 11;  // MVP-2 预留
    ImuSample        imu          = 12;  // future
  }
}

message JointStateSample {
  double position_value = 1;
  double velocity_value = 2;
  double effort_value   = 3;
}

message CameraFrame {
  uint32 width      = 1;
  uint32 height     = 2;
  string encoding   = 3;  // "rgb8" | "bgr8" | ...
  bytes  data       = 4;
  uint64 capture_tick = 5;
}

message ImuSample { /* future */ }

// —— QoS 分层 ——

enum QosLane {
  QOS_UNSPECIFIED      = 0;
  QOS_LOSSY_SETPOINT   = 1;  // drop-oldest OK；连续 setpoint 流
  QOS_RELIABLE_CONTROL = 2;  // 不允许丢；discrete actions
}

message ActuationBatch {
  WorldClock as_of = 1;              // client 标记的意图时间戳（MVP-1 仅 informative）
  repeated ActuationCommand commands = 2;
  QosLane lane = 3;                  // 必填
}

// —— Handshake + 顶层 ——

message WorldDescriptor {
  string world_name = 1;
  repeated RobotDescriptor robots = 2;
  WorldClock initial_clock = 3;
  uint32 publish_hz = 4;             // sim 告诉 Station 它跑多快
  uint32 physics_hz = 5;
}

message RobotDescriptor {
  string robot_id = 1;
  repeated ActuatorDescriptor actuators = 2;
  repeated SensorDescriptor sensors = 3;
}

message ActuatorDescriptor {
  string actuator_id = 1;            // "rev_motor_01"
  string display_name = 2;           // "Shoulder Pitch"
  ActuatorCapability capability = 3;
}

message SensorDescriptor {
  string sensor_id = 1;
  string display_name = 2;
  SensorCapability capability = 3;
}

message WorldSnapshot {
  WorldClock clock = 1;
  repeated ActuatorState actuators = 2;
  repeated SensorSample  sensors = 3;
}

// —— SimHealth：published to /sim/health NormFS queue ——
//
// MVP-1 字段分级（消费者契约）:
//   - 必填（bridge 读）: runtime_session_id, backend_kind, world_tick,
//                         backend_alive, termination, handshake_complete,
//                         world_name, robot_ids, actuator_count, sensor_count
//   - 选填（operator 诊断用，可填 0 或默认值）: 其它字段
//   bridge 的 health_task 只读 backend_alive + termination + runtime_session_id。
//   其余字段属于"发了不丢、不发不错"的未来扩展，避免 MVP-1 过早实现
//   rolling avg 等复杂统计逻辑。

message SimHealth {
  string runtime_session_id = 1;     // UUID per SimulationRuntime::start
  string backend_kind = 2;           // "child_process" | "external_socket"

  // 时间
  uint64 world_tick = 3;             // ★ correlation key
  uint64 sim_time_ns = 4;
  uint64 wall_time_unix_ns = 5;      // Station 侧 wall clock

  // 调度
  string scheduler = 6;              // "realtime" | "tickdriven"
  uint32 physics_hz_target = 7;
  uint32 publish_hz_target = 8;
  float  physics_hz_achieved = 9;
  float  publish_hz_achieved = 10;

  // 状态
  bool backend_alive = 11;
  BackendTermination termination = 12;  // filled when backend_alive = false
  bool handshake_complete = 13;

  // 质量
  uint64 overrun_count_total = 14;
  uint32 overrun_count_1s = 15;
  int64  max_lag_ns_1s = 16;
  uint64 snapshot_count_total = 17;
  uint64 actuation_count_total = 18;
  uint64 decode_error_count = 19;

  // 结构
  string world_name = 20;
  repeated string robot_ids = 21;
  uint32 actuator_count = 22;
  uint32 sensor_count = 23;
}

message BackendTermination {
  enum Cause {
    UNSPECIFIED          = 0;
    CLEAN                = 1;  // Goodbye + exit 0
    CRASHED              = 2;  // non-zero exit
    KILLED_BY_SUPERVISOR = 3;  // shutdown 期间
    SIGNALED_BY_OS       = 4;  // OOM killer 等
  }
  Cause cause = 1;
  int32 exit_code = 2;
  int32 signal = 3;
  bytes stderr_tail = 4;
}

// —— Envelope：所有消息的顶层封包 ——

message Envelope {
  oneof payload {
    Hello          hello          = 1;   // client → server
    Welcome        welcome        = 2;   // server → client, 带 WorldDescriptor
    ActuationBatch actuation      = 3;   // client → server
    WorldSnapshot  snapshot       = 4;   // server → client
    Goodbye        goodbye        = 5;
    Error          error          = 6;
  }
}

message Hello {
  uint32 protocol_version = 1;          // MVP-1: 固定 = 1。客户端声明"我说的版本"
  string client_role = 2;               // "station-runtime" / "inspect-tool" / ...
  string client_id = 3;
}

message Welcome {
  uint32 protocol_version = 1;          // MVP-1: 固定 = 1。服务端回应"我同意的版本"
  WorldDescriptor world = 2;
}

// 协议版本策略：
//
// MVP-1：任何 Hello.protocol_version != Welcome.protocol_version = 硬失败
//        (Error{E_PROTOCOL_VERSION})。两侧都固定发 1。
//
// 未来（当引入 v2 schema 时）：
//   1. 客户端发 Hello{protocol_version=<自己支持的最高版本>}
//   2. 服务端发 Welcome{protocol_version=min(client, server)} —— "我们协定用哪个版本"
//   3. 如果 min(client, server) < 1，服务端回 Error{E_PROTOCOL_VERSION}
//   这个语义在 MVP-1 是 no-op（双方都是 1），但 v2 引入时不需要改 Hello/Welcome schema。

message Goodbye { string reason = 1; }

message Error {
  enum Code {
    E_UNSPECIFIED        = 0;
    E_PROTOCOL_VERSION   = 1;
    E_UNKNOWN_ROBOT      = 2;
    E_UNKNOWN_ACTUATOR   = 3;
    E_UNKNOWN_CAPABILITY = 4;
    E_INVALID_COMMAND    = 5;
    E_SIM_INTERNAL       = 6;
    E_BACKPRESSURE       = 7;
  }
  Code code = 1;
  string message = 2;
}
```

**关键设计决定**：

1. **`world.proto` 不是 `sim.proto`**——"world" 是更准确的名字，反映共享世界语义
2. **Package 显式版本号 `v1`**——未来 breaking change 走 `v2` 平级 package，消费者显式选择
3. **`WorldClock` 在所有帧上**，不是只在 snapshot。`world_tick` 是 canonical time
4. **`wall_time_ns` 禁用作跨进程对齐**——schema 注释明文写，runtime 侧强制
5. **`SetPosition.value` 单位由 capability 决定**，不在字段名里写死。`rad_to_steps` 这种硬件术语完全不出现
6. **`QosLane` 必填**——消息不能没有明确 QoS 归属
7. **`SimHealth` 也是 protobuf message**——跨语言可消费，未来 Python 客户端也能订阅
8. **`BackendTermination` 作为独立 message**，不是 enum——需要携带 exit code / signal / stderr tail

### 6.6 代码生成

复用项目既有的 `make protobuf` 管道：

- **Rust** (`sim-runtime/build.rs`)：`prost_build::Config::new().compile_protos(&["protobufs/sim/world.proto"], &["protobufs"])`。生成到 `src/proto/world.rs`。同样方式用于 `sim-runtime` 和 `st3215-compat-bridge` 两个 crate（后者只需 re-export 前者）
- **Python**（`gremlin_py`）：生成到 `target/gen_python/protobuf/sim/world.py`，`norma_sim` 包直接 import
- **Go**（`gremlin_go`）：生成到 `target/generated-sources/protobuf/sim/`。MVP-1 不用，但管道一致

**Framing**：`u32 big-endian length + payload bytes`。最大 16 MiB/帧（留给未来相机帧）。Rust 用 `tokio_util::codec::LengthDelimitedCodec`，Python 用 `asyncio.StreamReader.readexactly`。

---

## 7. 组件设计（Python 侧）

### 7.1 `norma_sim` 包结构

**位置**：`software/sim-server/`

**布局**：

```
software/sim-server/
├── pyproject.toml
├── README.md                             # 含 MVP-1 smoke test checklist
├── norma_sim/
│   ├── __init__.py
│   ├── __main__.py                       # `python -m norma_sim`
│   ├── cli.py                            # argparse + wire everything together
│   ├── logging_setup.py                  # JsonFormatter (~60 行)
│   │
│   ├── world/                            # 物理与能力
│   │   ├── __init__.py
│   │   ├── manifest.py                   # world.yaml → dataclasses
│   │   ├── model.py                      # MjModel/MjData 持有 + 锁
│   │   ├── descriptor.py                 # manifest → WorldDescriptor proto
│   │   ├── actuation.py                  # ActuationCommand → data.ctrl
│   │   ├── snapshot.py                   # data → WorldSnapshot proto
│   │   └── capabilities.py               # ★ 唯一含能力语义的模块
│   │
│   ├── scheduler/                        # 时间与调度
│   │   ├── __init__.py
│   │   ├── base.py                       # WorldScheduler protocol
│   │   └── realtime.py                   # RealTimeScheduler
│   │
│   └── ipc/                              # 传输与连接
│       ├── __init__.py
│       ├── framing.py                    # length-prefixed u32 big-endian
│       ├── codec.py                      # Envelope encode/decode
│       ├── server.py                     # asyncio UDS accept loop
│       └── session.py                    # per-client handshake + streams
│
├── scripts/                              # 诊断工具（同时作为 ipc 消费者 smoke test）
│   ├── inspect.py                        # 连 sim, 打印 WorldSnapshot
│   ├── probe_manifest.py                 # 加载 manifest dry-run 校验
│   └── send_actuation.py                 # 手工发 ActuationBatch
│
└── tests/
    ├── conftest.py
    ├── world/
    │   ├── test_manifest_load.py
    │   ├── test_descriptor_build.py
    │   ├── test_capabilities_revolute.py
    │   ├── test_capabilities_gripper.py  # ★ P0
    │   ├── test_actuation_mapping.py
    │   ├── test_snapshot_all_actuators.py
    │   ├── test_mimic_gripper.py         # ★ P0
    │   └── test_scheduler_realtime_pacing.py
    ├── ipc/
    │   ├── test_framing.py
    │   ├── test_codec_envelope.py
    │   ├── test_handshake_happy.py
    │   ├── test_handshake_wrong_version.py
    │   └── test_handshake_unknown_robot.py
    └── integration/
        ├── test_full_loop.py
        ├── test_multi_client_fan_out.py  # ★
        ├── test_subprocess_clean_shutdown.py
        └── test_goodbye_flow.py
```

**结构原则**：

- **三个顶层子包 = 三个关注点**：`world`（物理 + 能力）、`scheduler`（时间）、`ipc`（传输）
- 三者**互不导入对方内部**：world 不知道 ipc 的存在；ipc 不知道 MjModel 是什么；scheduler 不知道 protobuf
- 唯一的 "glue" 在 `cli.py` 和 `__main__.py`——它们导入三个子包并把回调函数接起来
- **`capabilities.py` 是唯一含能力语义的地方**——所有 "GRIPPER_PARALLEL 0..1 映射到 joint rad" 这类知识集中一个 ~140 行的文件

**`world/` 三个文件的边界测试**（供 reviewer 和实现者）：

| 断言 | 含义 |
|---|---|
| "我可以不装 mujoco 就能 load 一份 `WorldManifest`" | → `manifest.py` 只依赖 `yaml + dataclasses`，**零** mujoco import |
| "我可以不加载 MJCF 就能 build 一份 `WorldDescriptor` proto" | → `descriptor.py` 只吃 `WorldManifest`，**不**吃 `MjModel` |
| "我可以不知道 protobuf 就能 load manifest" | → `manifest.py` **不** import `norma_sim.proto` |
| "我可以不碰 world.yaml 就能跑 mj_step" | → `model.py` 只吃 MJCF 路径和 timestep 数值，**不**吃 `WorldManifest` |

这四条断言是未来 code review 的边界守卫：违反了就意味着子模块边界在漂移。

### 7.2 依赖

```toml
[project]
name = "norma-sim"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = [
    "mujoco>=3.2,<4",
    "numpy>=1.26",
    "protobuf>=4",
]

[project.optional-dependencies]
dev = ["pytest>=8", "pytest-asyncio>=0.23", "ruff", "mypy"]
```

**故意缺席**（同 v1）：mujoco-python-viewer、stable-baselines3、gymnasium、dm-control、torch、opencv、imageio。这些是后续里程碑或 scene authoring 工具，不属于 MVP-1 sim server 的运行时。

**distribution 名 vs import 名**：distribution 名 `norma-sim`（连字符，PEP 508 推荐），Python import 名 `norma_sim`（下划线）。README 明确说明两者关系避免混淆。

### 7.3 关键模块签名

**`world/manifest.py`**：

```python
@dataclass(frozen=True)
class WorldManifest:
    world_name: str
    scene: SceneConfig
    robots: list[RobotManifest]
    mjcf_path: Path

def load_manifest(path: Path) -> WorldManifest:
    """加载 world.yaml，严格 schema 校验，失败即报错。"""
    ...
```

**`world/capabilities.py`**（★ 唯一含能力语义的模块）：

```python
def command_value_to_ctrl(
    cmd: SetPosition,                    # proto message
    capability: ActuatorCapability,
    actuator_meta: ActuatorManifest,
) -> float:
    """把 capability-keyed 命令值翻译成 MJCF actuator 的 ctrl 值。"""
    kind = capability.kind
    if kind == CAP_REVOLUTE_POSITION:
        return cmd.value                                    # 恒等
    elif kind == CAP_GRIPPER_PARALLEL:
        g = actuator_meta.gripper
        norm_lo, norm_hi = g.normalized_range
        joint_lo, joint_hi = g.primary_joint_range_rad
        t = (cmd.value - norm_lo) / (norm_hi - norm_lo)
        return joint_lo + t * (joint_hi - joint_lo)
    elif kind == CAP_PRISMATIC_POSITION:
        return cmd.value                                    # m, 恒等
    else:
        raise InvalidCapabilityError(f"unsupported: {kind}")

def ctrl_state_to_position_value(
    joint_qpos: float,
    capability: ActuatorCapability,
    actuator_meta: ActuatorManifest,
) -> float:
    """反向：从 joint qpos 计算暴露给外部的 position_value。"""
    # 对称实现 ...
```

**`world/actuation.py`** / **`world/snapshot.py`**：按 manifest 遍历 actuators，调 `capabilities` 做 per-actuator 转换，构造 protobuf message。

**`scheduler/realtime.py`**：见 §8 时间模型。

**`ipc/server.py`** + **`ipc/session.py`**：asyncio UDS accept loop + per-connection handshake + reader/writer 协程。多 client 支持通过 `asyncio.Queue` 广播给所有 session。

### 7.4 零 ST3215 不变量

**CI 强制**：

```bash
#!/bin/bash
# scripts/check-arch-invariants.sh
if grep -r -i "st3215" software/sim-server/norma_sim/ software/sim-server/scripts/; then
  echo "ERROR: norma_sim must not reference ST3215"
  exit 1
fi
if grep -r -i "st3215" software/sim-runtime/src/; then
  echo "ERROR: sim-runtime must not reference ST3215"
  exit 1
fi
```

这个脚本加进 `Makefile` 的 `sim-test` target，在本地和 future CI 都跑。

**为什么要硬**：v1 的根问题（codex §6.1 "两套设计硬拼"）的预防必须机械化执行，不能靠 code review。

---

## 8. 时间模型 + Scheduler

### 8.1 三频率独立（修正 v1 时间步进 bug）

| 频率 | MVP-1 默认 | 约束 | 用途 |
|---|---|---|---|
| `physics_hz` | **500** (= 1 / 0.002) | = `1 / timestep_sec`，和 MJCF `<option timestep>` 严格一致 | `mj_step` 节拍 |
| `publish_hz` | **100** | 必须整除 `physics_hz`（500 / 100 = 5） | 发 `WorldSnapshot` 节拍 |
| `render_hz` | (MVP-1 不用) | MVP-2: 30 或 25 | 相机渲染节拍 |

**整除约束**：`publish_every = physics_hz / publish_hz` 必须整数，否则 publish 相对 physics 会漂移。Config 加载时强制校验。

### 8.2 `WorldClock` 语义

```
WorldClock {
  world_tick:   u64   // 单调递增，从 0 开始，每个 mj_step +1，canonical time
  sim_time_ns:  u64   // 纯推导：world_tick * timestep_ns
  wall_time_ns: u64   // sim host 的 CLOCK_MONOTONIC，跨进程无意义，禁用作对齐
}
```

**优先级和禁用规则**：

- **`world_tick` 是唯一真相**——所有跨进程 / 跨语言时间对齐 = 用 `world_tick`
- **`sim_time_ns` 是冗余推导**，仅为让 Python/Go/JS 消费者免乘一次
- **`wall_time_ns` 仅在 sim host 进程内有意义**，跨进程必须视作不可比较。`St3215CompatBridge` 写 `st3215/inference` 的时间戳时用 **Station 自己的** `CLOCK_MONOTONIC`，不是 snapshot 里带的

### 8.3 `WorldScheduler` 抽象（Python 侧）

```python
# norma_sim/scheduler/base.py
class WorldScheduler(Protocol):
    def run_until_stopped(
        self,
        world: World,
        publish: Callable[[WorldSnapshot], None],
        render: Optional[Callable[[], None]],
    ) -> None: ...

# norma_sim/scheduler/realtime.py
class RealTimeScheduler:
    def __init__(self, physics_hz: int, publish_hz: int, render_hz: Optional[int] = None):
        assert physics_hz % publish_hz == 0, \
            f"physics_hz must divide publish_hz (got {physics_hz} / {publish_hz})"
        # ...

# norma_sim/scheduler/tickdriven.py （未实现，占位）
class TickDrivenScheduler:
    """按 Station 的 advance-to-tick 命令推进；用于 replay / deterministic / RL rollout。
       MVP-5 实现。"""
    def run_until_stopped(self, *args, **kwargs):
        raise NotImplementedError("TickDrivenScheduler is MVP-5 scope")
```

Config 里 `scheduler: realtime | tickdriven`，默认 `realtime`。`tickdriven` 在 MVP-1 抛 `NotImplementedError` + 明确错误——**但 enum 现在就到位**，避免未来加 case 需要改 schema。

### 8.4 `RealTimeScheduler` 主循环

```python
def run_until_stopped(self, world, publish, render):
    physics_ns = int(1e9 / self.physics_hz)              # 2_000_000 for 500Hz
    publish_every = self.physics_hz // self.publish_hz   # 5
    render_every = (self.physics_hz // self.render_hz) if self.render_hz else None

    tick = 0
    t0_wall_ns = monotonic_ns()                          # ★ 只记一次，全局 deadline
    overruns = 0
    last_health_report_tick = 0

    while not self.stopping:
        # 1. Physics step
        world.drain_pending_commands()
        world.mj_step()
        tick += 1

        # 2. Publish (每 N 步一次)
        if tick % publish_every == 0:
            snapshot = world.read_snapshot(
                clock=WorldClock(
                    world_tick=tick,
                    sim_time_ns=tick * physics_ns,
                    wall_time_ns=monotonic_ns(),
                ),
            )
            publish(snapshot)

        # 3. Render (MVP-2+)
        if render_every and tick % render_every == 0:
            render()

        # 4. Pace to wall clock (real-time policy)
        deadline_ns = t0_wall_ns + tick * physics_ns
        slack_ns = deadline_ns - monotonic_ns()
        if slack_ns > 0:
            sleep_ns(slack_ns)
        else:
            overruns += 1
            # 不 catch-up；接受 drift，health report 告警

        # 5. Health report at 1Hz
        if tick - last_health_report_tick >= self.physics_hz:
            self._report_health(tick, overruns, slack_ns)
            last_health_report_tick = tick
```

**关键设计点**：

- **`t0_wall_ns` 只记一次**，不是每步重置。deadline 是"第 N 个 tick 应该在 `t0 + N*dt` 完成"的**全局**目标。这样 sim overrun 后追回节拍时不会永久漂移
- **Overrun 不 catch-up**：不尝试连续跑几步物理追赶。那会让 web UI 看到"卡顿 → 猛冲"的视觉瑕疵。只是不 sleep，继续下一步。**接受 drift，上报指标**
- **drain commands 在 step 之前**：命令和它要影响的物理步发生在同一个 tick 里，语义干净
- **Physics 节拍永远不跳步**：每个 tick 都真实 `mj_step` 过，不会出现"sim 落后时跳过中间步"

### 8.5 Rust 侧 `WorldClock` 消费

`SimulationRuntime` 在 `/sim/health` queue 上 1Hz 发布 `SimHealth`（schema 见 §6.5），消费者包括：

- **`St3215CompatBridge` `state_task`**：处理每个 `WorldSnapshot` 时用 `clock.world_tick` 做 dedupe（tick 回退的帧丢弃 + log error）
- **`St3215CompatBridge` `health_task`**：监听 `backend_alive: false` 触发 offline marker
- **Station web UI**（未来）：订阅 `/sim/health` 显示 sim 健康状态
- **启动检查**：`Welcome` 握手后，Rust 拿 `WorldDescriptor.initial_clock.world_tick`，不为 0 → log warn（表示 sim 非 fresh start）

### 8.6 Clock skew 处理

**MVP-1 立场**：**不尝试跨进程时间对齐**。

- sim host `wall_time_ns` 是它进程的 `CLOCK_MONOTONIC`，Station `wall_time_ns` 是 Station 进程的 `CLOCK_MONOTONIC`——两者起点不同，不可跨进程比较
- `St3215CompatBridge` 写 `st3215/inference` 的 meta 时使用 **Station 自己的** `CLOCK_MONOTONIC`
- `snapshot.clock.wall_time_ns` 只在 Station log 里 echo 让 operator 肉眼看 drift
- **未来升级路径**（MVP-5+）：Handshake 交换一次 `(station_monotonic, sim_monotonic)` 样本算 offset。MVP-1 不做

### 8.7 Determinism hook（schema 就位，impl 留空）

`ActuationBatch.as_of.world_tick` 字段**现在就有**，但 MVP-1 的 `RealTimeScheduler` **忽略它**：

- MVP-1：`as_of` 仅作为 log / debug 信息
- 未来 `TickDrivenScheduler`：Station 显式发 `advance_to(tick=T)` 指令；sim 把 pending actuations 按 `as_of.world_tick` 排序，推进物理直到 `tick == T`，然后 publish

未来切换到 deterministic 模式时：

- Schema **零改动**
- Python `RealTimeScheduler` **零改动**（新加一个 `TickDrivenScheduler` 类）
- Rust `SimulationRuntime` 加 scheduler-mode-aware 的 actuation 发送策略
- 所有消费者（`St3215CompatBridge` 等）**零改动**

---

## 9. URDF → MJCF 派生 + Manifest

### 9.1 派生拓扑

```
hardware/elrobot/simulation/
├── elrobot_follower.urdf                      # 既有，不动
├── assets/*.stl                               # 既有
└── worlds/                                    # ✨ 新目录
    ├── elrobot_follower.world.yaml            # ★ 唯一人工编辑源
    ├── elrobot_follower.xml                   # gen.py 生成 + checkin
    ├── gen.py                                 # 派生脚本
    └── README.md
```

**三个产物关系**：

```
urdf (机械)  ──┐
               │
               ├──►  gen.py  ──►  MJCF xml (物理)
               │
world.yaml  ───┘
            (能力)

runtime  ──►  同时加载 world.yaml + MJCF xml
```

**设计原则**：

- **`world.yaml` 是唯一人工编辑入口**——机器人 SKU 的 world-level 元数据全在这
- **MJCF 是 gen.py deterministic 输出**——不手工编辑
- **URDF 保持不动**——硬件团队维护 URDF，仿真团队维护 world.yaml
- **`world.yaml` 既是 build 输入又是 runtime 输入**——`norma_sim` 启动直接读，不走"派生 registry yaml"中间步骤。**一份文件两个用途，零一致性风险**

### 9.2 `world.yaml` 字段设计

```yaml
# hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml
world_name: elrobot_follower_empty
urdf_source: ../elrobot_follower.urdf
mjcf_output: ./elrobot_follower.xml               # 相对本文件

scene:
  timestep: 0.002
  gravity: [0, 0, -9.81]
  integrator: RK4
  solver: Newton
  iterations: 50

scene_extras:
  lights:
    - name: default_top
      pos: [0, 0, 3]
      dir: [0, 0, -1]
  floor:
    size: [2, 2, 0.1]
    material: grid

robots:
  - robot_id: elrobot_follower

    actuators:
      # 7 个 revolute 主关节 （motor_01 ~ motor_07）
      - actuator_id: rev_motor_01
        display_name: "Shoulder Pitch"
        urdf_joint: rev_motor_01
        mjcf_actuator: act_motor_01
        capability:
          kind: REVOLUTE_POSITION
          # 由 gen.py 从 URDF <limit> 自动抽取回填
          limit_min: -1.5509
          limit_max: 1.5509
          effort_limit: 2.94
          velocity_limit: 4.71
        actuator_gains:
          kp: 15.0
          kv: 0.5

      # ... motor_02 ~ motor_07 同构 ...

      # 第 8 关节：gripper，特殊 capability
      - actuator_id: rev_motor_08
        display_name: "Gripper"
        urdf_joint: rev_motor_08
        mjcf_actuator: act_motor_08
        capability:
          kind: GRIPPER_PARALLEL                 # ★ 不是 REVOLUTE_POSITION
          limit_min: 0.0                         # 归一化 0..1
          limit_max: 1.0
          effort_limit: 2.94
          velocity_limit: 4.71
        actuator_gains:
          kp: 10.0
          kv: 0.3
        gripper:                                 # GRIPPER_PARALLEL 专属元数据
          primary_joint_range_rad: [0.0, 2.2028]
          normalized_range: [0.0, 1.0]
          mimic_joints:
            - joint: rev_motor_08_1
              multiplier: -0.0115
            - joint: rev_motor_08_2
              multiplier: 0.0115

    sensors:
      - sensor_id: joint_state_all
        display_name: "All joint states"
        capability:
          kind: JOINT_STATE
        source: all_actuators                    # 特殊关键字
```

**三个架构决定**：

1. **`capability.kind` 是 yaml 字段**，和 schema `ActuatorCapability.Kind` enum 逐字对应。错配 → 加载失败
2. **`GRIPPER_PARALLEL` 的归一化参数是 capability 的一部分**——不同 gripper actuator 可以有不同归一化策略，Python 代码零改动
3. **`sensors.joint_state_all.source = all_actuators`** 是语义关键字——避免为 "所有 actuator state 汇总" 写显式映射。未来真实 IMU/FT sensor 走显式 `mjcf_sensor: xxx` 路径

### 9.3 `gen.py` 职责

```python
# hardware/elrobot/simulation/worlds/gen.py
def main():
    manifest = load_yaml("elrobot_follower.world.yaml")
    urdf_tree = load_urdf(manifest["urdf_source"])

    # 1. 从 URDF 读每个 actuator 的物理约束回填 manifest
    validate_and_fill_limits(manifest, urdf_tree)

    # 2. 构造 MJCF 扩展段
    mjcf = build_mjcf(
        base_urdf=manifest["urdf_source"],
        option=manifest["scene"],
        compiler={"angle": "radian", "meshdir": "../assets", "autolimits": "true"},
        equality=derive_equality_constraints(manifest["robots"]),
        actuator=derive_actuator_elements(manifest["robots"]),
        default_classes=build_default_contype(),
        worldbody_extras=manifest["scene_extras"],
    )

    # 3. 写 MJCF
    write_mjcf(manifest["mjcf_output"], mjcf)

    # 4. 回写 manifest（limits 填充后）
    write_yaml("elrobot_follower.world.yaml", manifest)

    # 5. 一致性自检
    run_self_check(manifest, mjcf)
```

**原则**：idempotent + 带 self-check + **产出 hash 校验**。反复跑结果一致；自检失败 → exit 1。

**关于 "一份文件两用途" 的诚实声明**：严格来说，v2 的 build/runtime 真相**不是一份文件**，而是 `urdf + world.yaml + mjcf` 三份文件，runtime 读其中两份（`world.yaml + mjcf`）。§9.1 说的"一份文件两用途"特指 **`world.yaml`**——build 阶段和 runtime 阶段都读同一份 `world.yaml`，不做派生拆分。但 `mjcf` 是 `gen.py` 的派生输出，人工或自动可能与 `world.yaml` 失同步。**为此引入 MJCF hash 保护**（见下）。

**MJCF hash 保护**（gen.py 产出 + runtime 验证）：

- `gen.py` 在生成 MJCF 时，把 `sha256(urdf_source + world.yaml)` 的 hex 字符串作为 XML comment **嵌入 MJCF 文件头**：
  ```xml
  <mujoco model="elrobot_follower_empty">
    <!-- norma-sim: generated by gen.py
         source_hash=sha256:7c9e6679f4b6d3... (urdf + world.yaml)
         generator_version=1.0 -->
    <compiler angle="radian" .../>
    ...
  </mujoco>
  ```
- `norma_sim` 启动时：加载 `world.yaml` → 计算 `sha256(urdf + world.yaml)` → 读 MJCF 的 `source_hash` comment → 比对
- 不匹配 → **exit 1**，错误消息：`MJCF source_hash mismatch: world.yaml or URDF changed but MJCF was not regenerated. Run 'make regen-mjcf'.`
- 这把"world.yaml 和 MJCF 漂移"的可能性从"沉默 bug"降级为"启动期快错"

**权衡**：
- 代价：gen.py 多 10 行读文件+算 hash+写注释；runtime 多 15 行读注释+算 hash+比对
- 收益：消除了"我改了 world.yaml 但忘了 regen-mjcf"这个最常见的一致性漏洞
- 不做的事：不试图从 hash mismatch 自动触发 gen.py（那会引入静默修改源文件的风险）

**Self-check**：
- 每个 manifest `actuator_id` 在 MJCF 有对应 joint 元素
- 每个 manifest `mjcf_actuator` 名字在 MJCF `<actuator>` 段存在（防止 manifest 引用不存在的 actuator 名）
- 每个 `gripper.mimic_joints[*].joint` 在 MJCF 存在
- `REVOLUTE_POSITION` 的 `limit_min/max` 和 MJCF joint `<limit>` 一致
- `manifest.scene.timestep == mjcf.option.timestep`
- `mjcf.compiler.angle == "radian"`（因为 `norma_sim.world.capabilities` 全部假定 rad）
- 每个 `<equality>` polycoef 的 `c1` 系数严格等于 manifest 对应 `gripper.mimic_joints[i].multiplier`（防止 gen.py 和 manifest 多项式系数漂移）

### 9.4 MJCF 扩展段的关键内容

**`<option>`**：
```xml
<option timestep="0.002" iterations="50" solver="Newton"
        gravity="0 0 -9.81" integrator="RK4"/>
```

**`<compiler>`**：
```xml
<compiler angle="radian" meshdir="../assets"
          autolimits="true" coordinate="local" discardvisual="false"/>
```

**`<equality>`**（核心：替代 URDF mimic）：
```xml
<equality>
  <joint joint1="rev_motor_08_1" joint2="rev_motor_08"
         polycoef="0 -0.0115 0 0 0"/>
  <joint joint1="rev_motor_08_2" joint2="rev_motor_08"
         polycoef="0 0.0115 0 0 0"/>
</equality>
```

`polycoef="c0 c1 c2 c3 c4"` 语义是 `joint1 = c0 + c1*joint2 + c2*joint2² + ...`。`gen.py` 从 `world.yaml` 的 `gripper.mimic_joints[*].multiplier` 自动生成。

**`<actuator>`**：
```xml
<actuator>
  <position name="act_motor_01" joint="rev_motor_01"
            kp="15" kv="0.5" ctrlrange="-1.5509 1.5509"
            forcerange="-2.94 2.94"/>
  <!-- act_motor_02 ~ act_motor_07 每个 ctrlrange 由 gen.py 从 URDF 自动抽取 -->
  <position name="act_motor_08" joint="rev_motor_08"
            kp="10" kv="0.3" ctrlrange="0 2.2028"
            forcerange="-2.94 2.94"/>
</actuator>
```

**`<default>`**：
```xml
<default>
  <default class="arm_link">
    <geom contype="1" conaffinity="1" friction="0.9 0.005 0.0001"/>
  </default>
  <default class="gripper_finger">
    <geom contype="2" conaffinity="3" friction="1.5 0.05 0.001"/>
  </default>
</default>
```

**`<worldbody>`**：一个灯 + 一个地板（MVP-2 相机渲染需要光照）。

**备选方案**：如果 `<equality>` polycoef 在数值上不稳定（罕见），可以改用 `<tendon><fixed>`：
```xml
<tendon>
  <fixed name="grip_link_1">
    <joint joint="rev_motor_08" coef="1"/>
    <joint joint="rev_motor_08_1" coef="-0.0115"/>
  </fixed>
</tendon>
```
这个备选作为 README troubleshooting 提及，不是 `gen.py` 的策略开关（等运行现象驱动切换）。

---

## 10. 启动 UX + 配置文件

### 10.1 顶层 config 分段

v2 在现有 `Config` 上加**两个可选新段**，**不动**既有段：

```yaml
# station-sim.yaml
sim-runtime:          # ✨ 新
  ...
bridges:              # ✨ 新
  st3215_compat:
    ...
drivers:              # 既有，零改动
  ...
inference:            # 既有，零改动
  ...
```

**总原则**：

- **`sim-runtime` 和 `drivers` 正交**，不互斥。`drivers.st3215.enabled: false` 只是"不启动真机"，不排斥 sim
- **`bridges` 依赖 `sim-runtime`**：bridge 启用要求 runtime 启用；反向不强制
- **既有 `station.yaml` 零改动即可继续工作**——两个新段都是 optional
- **MVP-1 只支持单 yaml**：`station -c <path>` 只接受一个路径，无环境变量覆盖，无多文件 merge。如果未来需要多环境 config，走独立 spec

### 10.2 干掉 Mutual Exclusion：`bus_serial` 命名空间

**不新增** v1 spec §6.2 提议的 `Drivers::validate()` mutual exclusion 规则（既有代码里这条 validation 根本不存在——v1 提议从未实现）。v2 明确决定**不引入**这种硬性校验。

**替代**：real 和 sim 通过 `bus_serial` 命名空间共存。两者都可以同时往 `st3215/inference` queue 写，通过不同 `bus_serial` 字段区分：

- Real `st3215`：bus_serial = 真机 EEPROM 里烧的序列号（例如 `ST3215-BUS-A1B2C3`）
- `St3215CompatBridge`：bus_serial = config 里的 `legacy_bus_serial`（例如 `sim://bus0`）

下游 `inference` 段已经有 `st3215-bus` 字段选择消费哪条 bus。v2 要求：**shadow 模式下必须显式配置 `st3215-bus`**，不走 `auto`（`auto` 在多 bus 下行为未定义是既有 bug，不在本 spec 修）。

### 10.3 `sim-runtime` 段

```yaml
sim-runtime:
  enabled: true
  mode: internal                  # internal | external

  # Internal 模式：Station spawn backend
  launcher:                       # internal 必填；对 Station 完全不透明
    - python3
    - -m
    - norma_sim
    - --manifest
    - hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml
    - --physics-hz
    - "500"
    - --publish-hz
    - "100"

  # External 模式：Station 只 connect
  socket-path: null               # null → auto (internal 时自动用 runtime-dir/sim.sock)；external 必填
  runtime-dir: null               # null → auto /tmp/norma-sim-<station_pid>/

  startup-timeout-ms: 5000
  shutdown-timeout-ms: 2000
  log-capture: file               # file | inherit | null
  log-file: ./station_data/sim-runtime.log
```

**设计决定**：

- **`launcher` 对 Station 完全不透明**：Station 不知道是 Python 还是 native。Station 只做三件事：(1) spawn 这个命令，(2) 通过环境变量 `NORMA_SIM_SOCKET_PATH` 告诉它 bind 路径，(3) 等 socket 出现作为 readiness 信号
- **`physics-hz` / `publish-hz` 不在 Station config**——它们是 backend CLI 参数。Station 通过 `WorldDescriptor` handshake **读取** backend 实际配置，不强加
- **`runtime-dir` 是新概念**（取代 v1 的 sentinel + lock + stale 三件套）：一个 Station 拥有的临时目录，里面放 socket 和可选 log tail。整个目录 shutdown 时 `rm -rf`

### 10.4 `bridges` 段

```yaml
bridges:
  st3215_compat:
    enabled: true
    robot-id: elrobot_follower
    preset-path: software/sim-bridges/st3215-compat-bridge/presets/elrobot-follower.yaml
    legacy-bus-serial: "sim://bus0"
```

**四个字段的不变量**：

- `enabled`：开关
- `robot-id`：bridge 启动时 `sim_runtime.world_descriptor()` 必须包含，否则启动失败
- `preset-path`：yaml 数据文件路径，加载失败则启动失败
- `legacy-bus-serial`：运行时校验不和真机 bus 冲突

**未来扩展**（MVP-2+）：

```yaml
bridges:
  st3215_compat: { enabled: true, robot-id: ..., ... }
  usbvideo_compat:
    enabled: true
    robot-id: elrobot_follower
    cameras:
      - sensor_id: wrist_cam
        legacy_device_path: "sim:///dev/video-wrist"
```

Bridges 可组合，多个 bridge 共享一个 sim_runtime。

### 10.5 Rust struct 形状

加在 `software/station/shared/station-iface/src/config.rs`：

```rust
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Config {
    // 既有字段 — 类型和 Option 包装保持不变
    pub drivers: Drivers,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inference: Option<Vec<Inference>>,           // 既有类型名 `Inference`（不是 `InferenceConfig`）
    #[serde(rename = "cloud-offload", skip_serializing_if = "Option::is_none")]
    pub cloud_offload: Option<CloudOffloadConfig>,

    // 新增
    #[serde(rename = "sim-runtime", default, skip_serializing_if = "Option::is_none")]
    pub sim_runtime: Option<SimRuntimeConfig>,

    #[serde(default, skip_serializing_if = "Bridges::is_empty")]
    pub bridges: Bridges,
}

#[derive(Debug, Serialize, Deserialize, Clone, Default)]
pub struct Bridges {
    #[serde(rename = "st3215_compat", default, skip_serializing_if = "Option::is_none")]
    pub st3215_compat: Option<St3215CompatBridgeConfig>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SimRuntimeConfig {
    pub enabled: bool,
    pub mode: SimMode,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub launcher: Option<Vec<String>>,

    #[serde(rename = "socket-path", default, skip_serializing_if = "Option::is_none")]
    pub socket_path: Option<PathBuf>,

    #[serde(rename = "runtime-dir", default, skip_serializing_if = "Option::is_none")]
    pub runtime_dir: Option<PathBuf>,

    #[serde(rename = "startup-timeout-ms", default = "default_startup_timeout")]
    pub startup_timeout_ms: u64,

    #[serde(rename = "shutdown-timeout-ms", default = "default_shutdown_timeout")]
    pub shutdown_timeout_ms: u64,

    #[serde(rename = "log-capture", default)]
    pub log_capture: LogCapture,

    #[serde(rename = "log-file", default, skip_serializing_if = "Option::is_none")]
    pub log_file: Option<PathBuf>,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum SimMode { Internal, External }

#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Default)]
#[serde(rename_all = "lowercase")]
pub enum LogCapture { #[default] File, Inherit, Null }

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct St3215CompatBridgeConfig {
    pub enabled: bool,
    #[serde(rename = "robot-id")]
    pub robot_id: String,
    #[serde(rename = "preset-path")]
    pub preset_path: PathBuf,
    #[serde(rename = "legacy-bus-serial")]
    pub legacy_bus_serial: String,
}

impl Config {
    pub fn validate(&self) -> Result<(), ConfigError> {
        // (1) sim-runtime internal 模式必须有 launcher
        if let Some(sim) = &self.sim_runtime {
            if sim.enabled && matches!(sim.mode, SimMode::Internal) && sim.launcher.is_none() {
                return Err(ConfigError::MissingLauncher);
            }
            if sim.enabled && matches!(sim.mode, SimMode::External) && sim.socket_path.is_none() {
                return Err(ConfigError::MissingExternalSocketPath);
            }
        }

        // (2) bridge 启用要求 sim-runtime 启用
        if let Some(bridge) = &self.bridges.st3215_compat {
            if bridge.enabled {
                let sim_enabled = self.sim_runtime.as_ref().map_or(false, |s| s.enabled);
                if !sim_enabled {
                    return Err(ConfigError::BridgeRequiresSimRuntime);
                }
            }
        }

        // NOTE: 零 mutual exclusion 校验。real + sim coexistence 合法。
        Ok(())
    }
}
```

**不新增**：v1 spec §6.2 的 `Drivers::validate()` + `MutuallyExclusive` error variant。**注**：这两项在既有代码里**从未实现**——它们是 v1 spec 的提议，不是代码里存在需要删除的东西。v2 明确选择"不新增这套 validation"。`Config::validate()` 本身是全新方法，只做启动完备性检查（缺 launcher / 缺 socket_path / bridge 无 runtime），零互斥规则。

### 10.6 三个场景的完整 yaml

**场景 A — 纯 sim dev loop** (`station-sim.yaml`)：

```yaml
sim-runtime:
  enabled: true
  mode: internal
  launcher:
    - python3
    - -m
    - norma_sim
    - --manifest
    - hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml
    - --physics-hz
    - "500"
    - --publish-hz
    - "100"

bridges:
  st3215_compat:
    enabled: true
    robot-id: elrobot_follower
    preset-path: software/sim-bridges/st3215-compat-bridge/presets/elrobot-follower.yaml
    legacy-bus-serial: "sim://bus0"

drivers:
  st3215: { enabled: false }
  system-info: true
  usb-video: { enabled: false }

inference:
  - queue-id: inference/normvla
    shm: /dev/shm/normvla                         # 既有必填字段（Linux 默认 /dev/shm；macOS 用 /tmp）
    shm-size-mb: 12
    format: normvla
    st3215-bus: "sim://bus0"                      # 显式，不走 auto
    update-interval: 100ms
```

**场景 B — External 模式** (`station-sim-external.yaml`)：

```yaml
sim-runtime:
  enabled: true
  mode: external
  socket-path: /tmp/norma-sim-dev.sock
  startup-timeout-ms: 5000

bridges:
  st3215_compat:
    enabled: true
    robot-id: elrobot_follower
    preset-path: software/sim-bridges/st3215-compat-bridge/presets/elrobot-follower.yaml
    legacy-bus-serial: "sim://bus0"

drivers:
  st3215: { enabled: false }
  system-info: true

inference:
  - queue-id: inference/normvla
    shm: /dev/shm/normvla
    shm-size-mb: 12
    format: normvla
    st3215-bus: "sim://bus0"
    update-interval: 100ms
```

启动顺序：terminal 1 跑 `python -m norma_sim --manifest ... --socket /tmp/norma-sim-dev.sock`；terminal 2 跑 `station -c station-sim-external.yaml`。

**场景 C — Shadow 模式** (`station-shadow.yaml`)：

```yaml
sim-runtime:
  enabled: true
  mode: internal
  launcher: [python3, -m, norma_sim, --manifest,
             "hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml",
             --physics-hz, "500", --publish-hz, "100"]

bridges:
  st3215_compat:
    enabled: true
    robot-id: elrobot_follower
    preset-path: software/sim-bridges/st3215-compat-bridge/presets/elrobot-follower.yaml
    legacy-bus-serial: "sim://elrobot-shadow"    # 和真机不冲突

drivers:
  # 真机同时启用。ST3215 driver 通过 USB 枚举自动发现真实串口，
  # 不需要 yaml 指定 port。既有 St3215Config 字段：enabled / current-threshold /
  # motor-current-thresholds / deadband，不包含 `buses`。
  st3215:
    enabled: true
    current-threshold: 100
    deadband: 20
  usb-video: { enabled: true }
  system-info: true

inference:
  # 真机回路
  - queue-id: inference/normvla-real
    shm: /dev/shm/normvla-real                    # ★ shadow 下必须两条独立 shm
    shm-size-mb: 12
    format: normvla
    st3215-bus: "ST3215-BUS-A1B2C3"              # 真机 EEPROM 序列号（示例；真值由 discovery 提供）
    update-interval: 100ms
  # sim 回路
  - queue-id: inference/normvla-sim
    shm: /dev/shm/normvla-sim                     # ★ 独立 shm 路径
    shm-size-mb: 12
    format: normvla
    st3215-bus: "sim://elrobot-shadow"
    update-interval: 100ms
```

**这在 v1 spec 的提议下是被 `MutuallyExclusive`（从未实现的）硬拒绝的**。v2 明确作为第一类用例。

**Shadow 模式的运行时假设**：
- 真机 `ST3215-BUS-A1B2C3` 是 USB 枚举时由 ST3215 driver 写入 `st3215/meta` queue 的序列号（操作员通过 `station-queue-dump st3215/meta` 查询后填到 yaml）
- 两条 `inference` 必须有**独立的 shm 路径**（`normvla-real` vs `normvla-sim`），否则共享内存会写花
- Web UI 同时显示两条 bus，operator 可对照 divergence

### 10.7 `robot_id` 的配置位置

Codex Top 5 #2 的 "按 `robot_id/backend_id` 建模" 在 v2 有**两个**体现层：

1. **Schema 层**：`world.proto` 的 `RobotDescriptor`、`ActuatorRef.robot_id`
2. **Bridge 层**：`bridges.st3215_compat.robot-id` 字段

**没做**的是"Station-level `robots:` 段"：

```yaml
# 故意不做（future work）
robots:
  - id: elrobot_follower
    hardware: { st3215_bus: /dev/ttyUSB0 }
    sim_backend: default
```

**不做的理由**：

- Real drivers 当前不知道"robot"概念，引入 top-level 段会让 config 出现两份真相
- MVP-1 场景只有一个 robot，结构开销无收益
- `robot_id` 作为 schema + bridge 概念**足够**支撑 shadow mode 和 multi-robot 未来
- Real driver 的 robot-awareness 重构是独立 spec

**做的是**：在 bridge 配置层**承认 `robot_id` 是一等概念**，让未来 Station 重构有明确迁移方向。

---

## 11. 错误处理 + 故障模式矩阵

### 11.1 错误哲学

**延续 v1**：启动期快错 + 运行期软错。

- **启动期**：任何错误 → Station exit 1 + 明确错误消息 + 修复建议
- **运行期**：subprocess 崩溃 / socket 断 → bridge 和 sim_runtime 降级，**其它 drivers 不受影响**，Station 继续运行

### 11.2 启动期故障矩阵

| # | 故障 | 响应 | Rust error |
|---|---|---|---|
| S1 | Config validation fail（bridge 无 sim-runtime / internal 无 launcher / external 无 socket_path） | Station exit 1 before any spawn | `ConfigError::*` |
| S2 | `mkdir runtime_dir` 失败（权限 / 磁盘满） | Station exit 1 | `SimRuntimeError::RuntimeDirCreate` |
| S3 | `launcher` spawn 失败（command not found / exec permission） | Station exit 1，错误含命令字符串 | `BackendError::SpawnFailed` |
| S4 | Socket 文件在 `startup-timeout-ms` 内未出现 | SIGKILL subprocess，Station exit 1，错误含 log 路径 | `BackendError::SocketTimeout` |
| S5 | Socket 出现但 UDS connect 失败 | SIGKILL，Station exit 1 | `BackendError::ConnectFailed` |
| S6 | Hello/Welcome 在剩余 timeout budget 内未完成 | SIGKILL，Station exit 1，建议检查 backend log | `BackendError::HandshakeTimeout` |
| S7 | Welcome 收到但 `protocol_version` 不匹配 | Station exit 1，错误含两侧版本号 | `BackendError::ProtocolMismatch` |
| S8 | Welcome 收到但 `WorldDescriptor.robots` 不含 bridge 的 `robot_id` | Station exit 1 **在 bridge 启动阶段** | `BridgeError::RobotNotInWorld` |
| S9 | Bridge 的 `legacy-bus-serial` 和 real driver 探测出的 bus serial 冲突 | Station exit 1 **在 bridge 启动阶段**，运行时检测 | `BridgeError::BusSerialCollision` |
| S10 | `preset-path` 加载失败（file missing / yaml 错 / 字段缺失） | Station exit 1 | `BridgeError::PresetLoad` |
| S11 | External 模式下 `socket-path` 不存在 / connection refused | Station exit 1，明确 "sim backend 是否在运行" | `BackendError::ExternalNotRunning` |

### 11.3 运行期故障矩阵

| # | 故障 | SimulationRuntime 响应 | Bridge 响应 | Station 响应 |
|---|---|---|---|---|
| R1 | Subprocess 崩溃（non-zero exit） | `wait_terminated()` 返回 `Crashed`；发 `/sim/health` `backend_alive: false` | 收到 health 事件 → 写 `st3215/meta` offline marker → task 退出 | 继续运行 |
| R2 | Socket EOF（subprocess clean exit 无 Goodbye） | 同 R1 | 同 R1 | 同 |
| R3 | Subprocess SIGKILL（OOM / 外部 kill） | `wait_terminated()` 返回 `SignaledByOs`；health 带 signal | 同 R1 | 同 |
| R4 | Protobuf decode 失败（Envelope 损坏） | 关连接 → 进入 Crashed 路径 | 同 R1 | 同 |
| R5 | `QOS_LOSSY_SETPOINT` queue 溢出 | Drop oldest，log warn | Bridge fire-and-forget 看不到 | — |
| R6 | `QOS_RELIABLE_CONTROL` queue 溢出 | `send_actuation()` 返回 `Err(Backpressure)` | Bridge log error，单条命令失败 | — |
| R7 | Backend 回 `E_UNKNOWN_ACTUATOR` | runtime 原样 broadcast error event | Bridge log error，单条命令失败继续 | — |
| R8 | Physics NaN 爆炸（backend 自杀） | Backend stderr `CRITICAL` + exit 1 → 走 R1 路径 | 同 R1 | 同 |
| R9 | Backend 长时间不发 snapshot（>5s） | `/sim/health` 的 `publish_hz_achieved` 降到 0，log warn，**不认定死亡** | Bridge 看到 snapshot stream 超时 → log warn，不动作 | — |
| R10 | Bridge task panic（Rust 代码 bug） | — | tokio 捕获 panic，bridge 进 Failed，log error | 其它 bridge / driver 不受影响 |

### 11.4 关闭期故障

| # | 故障 | 响应 |
|---|---|---|
| T1 | Subprocess 忽略 SIGTERM | grace 到 → SIGKILL，log warn |
| T2 | Subprocess 已死（shutdown 前 crash） | no-op，继续清 runtime_dir |
| T3 | runtime_dir 删除失败 | log warn 继续（`TempRuntimeDir::Drop` 是 best-effort） |
| T4 | Goodbye 发送失败（socket 已断） | 吞错，进入 SIGTERM 路径 |

### 11.5 MVP-1 明确不做

- ❌ 自动重启 sim subprocess
- ❌ Driver / bridge 自动重连
- ❌ 健康检查 / heartbeat / circuit breaker
- ❌ Graceful degradation（回落 kinematic mock）

---

## 12. 可观测性 + 日志 + 诊断工具

### 12.1 三根支柱 + 一个主键

```
 ┌──────────────────────────────────────────────────────────────────┐
 │                        观测体系                                   │
 │                                                                   │
 │  (1) Logs          (2) Health Stream     (3) Diagnostic Tools    │
 │  ─────────         ───────────────       ──────────────────      │
 │  Rust: log+env     /sim/health queue     inspect.py              │
 │  Python: JSON       SimHealth 1Hz         probe_manifest.py      │
 │                     broadcast             send_actuation.py      │
 │                                                                   │
 │           ┌──────────────────────────────────┐                   │
 │           │   主键：world_tick               │                    │
 │           │   所有事件带 tick → 跨语言对齐   │                    │
 │           └──────────────────────────────────┘                   │
 └──────────────────────────────────────────────────────────────────┘
```

**核心原则**：`world_tick` 是跨语言 correlation 的唯一主键。Station log、Python log、health queue、bridge 的 inference 写入、诊断工具输出——**只要事件发生在某个物理步上，都带 `world_tick` 字段**。

不引入 request_id / trace_id / span_id——那是未来 OpenTelemetry 的事。

### 12.2 Rust 侧日志约定

沿用现有 `log + env_logger`（不迁移到 `tracing` — scope discipline）。

**约定**：

1. **log target = crate::module 路径**：`log::info!(target: "sim_runtime::backend", ...)`
2. **结构化字段内嵌为 `key=value` 字符串**（和现有 Station 代码一致）：
   ```rust
   log::info!(
       target: "sim_runtime::handshake",
       "handshake complete session_id={} protocol_version={} world_name={} actuators={}",
       session_id, protocol_version, world_name, actuator_count
   );
   ```
3. **`world_tick=NNN` 字段强制**，未知时用 `world_tick=-1` 占位方便 grep
4. **Error log 带 root cause 链**（`thiserror` 生成的 `source()` 链）

**crate log target 约定**：

```
sim_runtime::runtime
sim_runtime::backend
sim_runtime::handshake
sim_runtime::dispatch
sim_runtime::health
st3215_compat_bridge::start
st3215_compat_bridge::command_task
st3215_compat_bridge::state_task
st3215_wire::pack
```

### 12.3 Python 侧日志约定

**格式**：JSON per line，stdlib logging + 自定义 `JsonFormatter`（~60 行）。

**component 约定**（mirror Rust 的 crate target）：

```
norma_sim.cli
norma_sim.world.manifest
norma_sim.world.model
norma_sim.world.actuation
norma_sim.world.snapshot
norma_sim.world.capabilities
norma_sim.scheduler.realtime
norma_sim.ipc.server
norma_sim.ipc.session
norma_sim.ipc.framing
norma_sim.ipc.codec
```

**使用**：

```python
logger.info(
    "applied actuation batch",
    extra={"extra_fields": {
        "world_tick": current_tick,
        "session_id": client_session_id,
        "batch_size": len(batch.commands),
        "robot_id": robot_id,
        "lane": lane_name,
    }}
)
```

### 12.4 跨语言 correlation 使用

```bash
# Step 1: 看 health queue 找事件窗口
station-queue-dump /sim/health | jq 'select(.overrun_count_1s > 0)'

# Step 2: 拿 world_tick 在 Station log grep
grep 'world_tick=42350' station.log

# Step 3: 同一个 tick 在 Python log grep
jq 'select(.world_tick == 42350)' sim-backend.log
```

### 12.5 诊断工具

**`inspect.py`**：外部连 sim，订阅并打印。
```bash
python software/sim-server/scripts/inspect.py \
  --socket /tmp/norma-sim-12345/sim.sock \
  --robot elrobot_follower \
  --filter actuators
```

**`probe_manifest.py`**：不启 sim，dry-run 校验 manifest。
```bash
python software/sim-server/scripts/probe_manifest.py \
  --manifest hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml
```

**`send_actuation.py`**：手工发 ActuationBatch 调试。
```bash
python software/sim-server/scripts/send_actuation.py \
  --socket /tmp/norma-sim-12345/sim.sock \
  --robot elrobot_follower --actuator rev_motor_01 \
  --set-position 0.5 --lane reliable
```

**三个工具都复用 `norma_sim.ipc` 子包**——既是运维工具又是 ipc 消费者 smoke test。

### 12.6 Debug 环境变量

| 变量 | 作用域 | 值 |
|---|---|---|
| `RUST_LOG` | Rust | `info,sim_runtime=debug,st3215_compat_bridge=debug` |
| `NORMA_SIM_LOG_LEVEL` | Python | `DEBUG` / `INFO` / `WARN` / `ERROR` |
| `NORMA_SIM_LOG_FORMAT` | Python | `json` (默认) / `text` |
| `NORMA_SIM_TRACE_IPC` | Python | `1` = 逐 Envelope DEBUG 日志 |
| `NORMA_SIM_SOCKET_PATH` | Python | **单一 contract**：由 Station `ChildProcessBackend` 注入 |
| `MUJOCO_GL` | Python | `egl` / `osmesa` / `glfw`（MVP-2 相机用）|

---

## 13. 测试策略

### 13.1 哲学

**不追求覆盖率，追求 P0 风险消减**。P0 风险有专门测试，P1 基本测试，P2 靠手工 smoke。

**P0 风险**：

1. ST3215 寄存器字节格式错（下游解码失败）
2. MJCF `<equality>` polycoef 写错（夹爪不动）
3. `GRIPPER_PARALLEL` capability 归一化写错（夹爪 0..1 映射错）
4. 启动时 sim backend 起不来（用户首次体验失败）
5. Multi-client fan-out 不工作（MVP-2 加 usbvideo-compat-bridge 时爆炸）

### 13.2 测试分层

| 层 | 工具 | 范围 |
|---|---|---|
| 1. 静态 | `cargo clippy`、`ruff`、`mypy --strict`、`grep -ri st3215` 架构不变量 | 代码风格 + 类型 + 架构守护 |
| 2. 单元 | `cargo test`、`pytest` | framing / codec / pack / capabilities / manifest / mimic / physics 稳定性 / handshake |
| 3. 集成 | `pytest` + 子进程 | 真启 sim backend，handshake → actuate → snapshot 回环 |
| 4. 手工 smoke | README checklist | End-to-end Station + web UI，含三个场景（internal / external / shadow）+ 三故障场景 |

### 13.3 Rust 单元测试

**`st3215-wire` crate**（≥ 10 测试，P0）：

- `test_pack_length_71_bytes` — 输出长度恒定
- `test_pack_present_position_at_0x38` — 已知 rad → bytes[0x38..0x3A] 是期望 little-endian u16
- `test_pack_sign_magnitude_negative_speed` — 负速度符号位
- `test_pack_torque_enable_boolean` — bytes[0x28] 是 0 或 1
- `test_pack_eeprom_preset_fields` — model_number 落在 0x00..0x02
- **`test_pack_roundtrip_via_unpack`** ★ — pack 71 字节 → unpack → assert position_rad 误差 < 2 mrad
- `test_unpack_preserved_regression` — 用 `st3215` real driver `port.rs` 既有的 parsed fixture 喂 `unpack`，assert 结果相同（real driver 迁移后的 regression guard）

**`sim-runtime` crate**（≥ 15 测试）：

- `test_framing_roundtrip` — length-prefixed u32 + payload
- `test_envelope_codec_roundtrip` — 所有 Envelope variant
- `test_handshake_happy_path` — 用 `MockBackend`
- `test_handshake_protocol_mismatch` — 错版本号
- `test_handshake_timeout` — backend 不回 Welcome
- `test_actuation_sender_lossy_lane_drop_oldest`
- `test_actuation_sender_reliable_lane_backpressure`
- `test_snapshot_broker_multi_subscriber` — 多 subscriber 同时收到同一条
- `test_snapshot_broker_lagged` — 慢消费者得 `Lagged(n)`
- `test_health_publishing_1hz`
- `test_backend_crash_broadcasts_health`
- `test_shutdown_clean_removes_runtime_dir`
- `test_shutdown_after_backend_death`
- `test_world_descriptor_roundtrip` — handshake 拿到的 descriptor 和 `subscribe()` 得到的一致
- `test_clock_monotonic` — world_tick 永不回退
- `test_world_descriptor_persisted_to_queue` — `SimulationRuntime::start` 后 `/sim/descriptor` queue 有且仅有 1 条 entry，含正确 `runtime_session_id`
- `test_mjcf_source_hash_mismatch` — 构造一份 MJCF 带错误的 `source_hash` comment，backend 启动时应 fail fast 并报 hash mismatch

**`st3215-compat-bridge` crate**（≥ 10 测试 — bridge 三 task 各有覆盖 + sim:// 前缀强制）：

- `test_preset_loader_happy`
- `test_preset_loader_invalid_yaml`
- **`test_legacy_bus_serial_must_have_sim_prefix`** — config 里 `legacy_bus_serial = "bus0"`（无 `sim://` 前缀）→ bridge 启动失败 `BridgeError::InvalidLegacyBusSerial`
- `test_actuator_map_bidirectional`
- `test_command_task_setpos_translates` — command_task #1
- `test_command_task_torque_enable_routes_reliable` — command_task #2
- `test_state_task_packs_bytes_correctly` — state_task：mock SimulationRuntime 发 WorldSnapshot，验证 inference queue 写入内容
- **`test_health_task_writes_offline_marker_on_backend_termination`** — health_task：给 health broadcast 投递 `backend_alive=false` + `termination=Crashed`，验证 `st3215/meta` queue 收到 offline marker + task 以 `BridgeError::BackendTerminated` 退出
- `test_bridge_startup_fails_on_unknown_robot`
- `test_bridge_startup_fails_on_bus_serial_collision`

**`station-iface` crate**（≥ 3 新测试）：

- `test_config_validate_missing_launcher`
- `test_config_validate_bridge_without_runtime`
- `test_config_accepts_real_and_sim_coexistence` — ★ 明确断言 v2 没有 mutual exclusion

### 13.4 Python 测试（≥ 25 测试）

**world/**（P0）：

- `test_manifest_load_happy` — manifest load + dataclass 构造
- `test_manifest_load_missing_gripper_fields` — GRIPPER_PARALLEL 缺 `gripper:` 段 → 加载失败
- `test_descriptor_build_revolute`
- `test_descriptor_build_gripper_with_normalization`
- **`test_capabilities_revolute_identity`** — REVOLUTE_POSITION value → ctrl 恒等
- **`test_capabilities_gripper_roundtrip`** ★ — value=0.0/0.5/1.0 → joint_rad → 反向回到 value，误差 <1e-6
- `test_actuation_apply_batch`
- `test_snapshot_build_all_actuators`
- `test_mjcf_loads_without_error`
- **`test_mimic_gripper_equality_works`** ★ — 驱动 rev_motor_08 到 1.0，跑 500 步，assert mimic joints ≈ ±0.0115 (容差 2mm)
- `test_1000_steps_no_nan`
- `test_position_tracking_no_overshoot` — overshoot < 10%
- `test_scheduler_realtime_pacing` — 100Hz publish 持续 5s，实际速率误差 < 5%

**ipc/**：

- `test_framing_roundtrip_small`
- `test_framing_roundtrip_1mb_boundary`
- `test_codec_envelope_all_variants`
- `test_handshake_happy_path`
- `test_handshake_wrong_protocol_version`
- `test_handshake_unknown_robot`

**integration/**：

- `test_full_loop` ★ — subprocess 真启 → handshake → 发 SetPosition → 观察 position 收敛
- **`test_multi_client_fan_out`** ★★ — 两个 client 并发订阅同一 world → 发一条命令 → 两者都收到 snapshot broadcast
- `test_subprocess_clean_shutdown`
- `test_goodbye_flow`

**Rust 跨 crate 集成测试**（`sim-runtime` + `st3215-compat-bridge` + real `commands` queue）：

- **`test_legacy_station_py_commands_flow_through_bridge`** ★★★ — 起 Station + `MockBackend`（sim-runtime 内）+ bridge；用 `station_py` 的命令发送 pattern（或纯 Rust 模拟相同的 `StationCommandsPack` 写入 `commands` queue）发一条 ST3215 `GoalPosition` 写命令，`target_bus_serial = "sim://bus0"`；验证 bridge 把它翻译成 `ActuationCommand::SetPosition` 并通过 `MockBackend.outbound` 发出。**这是 codex 指出的 compatibility story 的硬核 E2E 测试**——没有这条测试，`commands` vs `st3215/commands` 这种换皮 bug 会重新出现
- `test_shadow_mode_target_bus_serial_routing` — 同上但注入**两条命令**：一条 `target_bus_serial = "sim://elrobot-shadow"`，一条 `target_bus_serial = "ST3215-BUS-A1B2C3"`；验证 bridge 只处理前者，后者被过滤掉

**multi_client_fan_out** 是从 v1 的 "为 MVP-2 铺路" 升级为 **MVP-1 强制测试**——v2 架构下 bridge 和未来组件一定会并发订阅同一 runtime。

### 13.5 架构不变量测试（CI 脚本）

加到 `Makefile sim-test` target：

```bash
#!/bin/bash
set -e

# 架构不变量 #1: norma_sim 零 ST3215 痕迹
if grep -r -i "st3215" software/sim-server/norma_sim/ software/sim-server/scripts/; then
  echo "ERROR: norma_sim must not reference ST3215"
  exit 1
fi

# 架构不变量 #2: sim-runtime 零 ST3215 痕迹
if grep -r -i "st3215" software/sim-runtime/src/; then
  echo "ERROR: sim-runtime must not reference ST3215"
  exit 1
fi

# 架构不变量 #3: st3215-wire 零 tokio/normfs/station 依赖
if grep -r -i "tokio\|normfs\|station_iface\|StationEngine" software/drivers/st3215-wire/src/; then
  echo "ERROR: st3215-wire must be pure protocol"
  exit 1
fi

# 架构不变量 #4: WorldBackend trait 必须是 pub(crate)
if grep -q "^pub trait WorldBackend" software/sim-runtime/src/backend/mod.rs; then
  echo "ERROR: WorldBackend must be pub(crate), not pub"
  exit 1
fi

echo "All architecture invariants hold."
```

### 13.6 MVP-1 Definition of Done

**功能性 (F)**：

| # | 验收 |
|---|---|
| F1 | `make sim-install && make protobuf && make regen-mjcf && cargo build && ./target/debug/station -c station-sim.yaml` 从零仓库**一条命令序列**启动成功 |
| F2 | 浏览器 web UI 看到 ElRobot follower 3D 模型，8 个关节可操作 |
| F3 | 7 个主臂 revolute 关节在 MuJoCo 物理下正确跟踪 SetPosition |
| F4 | ★ 第 8 关节（gripper）通过 `GRIPPER_PARALLEL` 0..1 归一化工作；MJCF `<equality>` 驱动两个 prismatic mimic 联动 |
| F5 | `st3215-compat-bridge` 写 `st3215/inference` 的字节与真 ST3215 二进制兼容，web UI / station_py 零修改可解码 |
| F6 | ★ Shadow 模式 config 可启动（real + sim 共存），两条 bus_serial 不冲突 |
| F7 | External 模式：terminal 1 sim，terminal 2 station，连通 |
| F8 | `probe_manifest.py` 对 shipped `elrobot_follower.world.yaml` 所有 consistency 检查通过 |

**鲁棒性 (R)**：

| # | 验收 |
|---|---|
| R1 | 启动失败走快错 + 明确错误（覆盖 §11.2 的 S1~S11 全部故障） |
| R2 | 运行期 subprocess 崩溃：Station 继续运行，`/sim/health` 发出 `backend_alive=false`，bridge 在 `st3215/meta` 写 offline marker |
| R3 | Ctrl+C：干净关闭（runtime_dir 被清、subprocess 被 wait 或 kill、socket 不残留） |
| R4 | `Config::validate()` 静态捕获 missing launcher / missing socket_path / bridge without runtime |
| R5 | **不做** mutual exclusion 校验——real + sim 可同时启用 |

**质量 (Q)**：

| # | 验收 |
|---|---|
| Q1 | `cargo test -p sim-runtime --all-features` 全绿（≥ 15 测试）|
| Q2 | `cargo test -p st3215-compat-bridge` 全绿（≥ 10 测试，三个 bridge task 各有覆盖 + `sim://` 前缀强制 + command queue 过滤）|
| Q3 | `cargo test -p st3215-wire` 全绿（≥ 10 测试，含 pack↔unpack roundtrip P0）|
| Q4 | `cargo test -p st3215` 全绿（real driver 迁移后零 regression）|
| Q5 | `cd software/sim-server && pytest` 全绿（≥ 25 测试，含 ★ capabilities_gripper / ★ mimic_gripper / ★ multi_client_fan_out）|
| Q6 | `cargo clippy --workspace -- -D warnings` 全清 |
| Q7 | 手工 smoke test checklist 全过（§附录 A）|

**架构不变量 (A)**：

| # | 验收 |
|---|---|
| A1 | `grep -ri "st3215" software/sim-runtime/src/` **零命中** |
| A2 | `grep -ri "st3215" software/sim-server/norma_sim/` **零命中**（CI 强制） |
| A3 | `grep -ri "normfs\|StationEngine\|tokio" software/drivers/st3215-wire/src/` **零命中** |
| A4 | `WorldBackend` trait 是 `pub(crate)` |
| A5 | `Config::validate()` 零行 mutual exclusion 逻辑 |
| A6 | `cargo tree -p sim-runtime` 不含 `st3215-wire` |

**判定权**：maintainer 手工跑 smoke test + 上述命令序列全通 → 签字。

---

## 14. Repo Layout 完整清单

### 14.1 新增文件

```
protobufs/sim/
└── world.proto                                    [✨ ~180 行]

hardware/elrobot/simulation/worlds/                [✨ 新目录]
├── elrobot_follower.world.yaml                    [✨ ~200 行]
├── elrobot_follower.xml                           [✨ ~250 行 MJCF (gen.py 输出)]
├── gen.py                                         [✨ ~300 行]
└── README.md                                      [✨ ~60 行]

software/drivers/st3215-wire/                      [✨ 新 crate]
├── Cargo.toml                                     [无 tokio/normfs 依赖]
└── src/
    ├── lib.rs                                     [~40 行]
    ├── register.rs                                [移出 ~120 行]
    ├── layout.rs                                  [移出 ~80 行]
    ├── units.rs                                   [移出 ~100 行]
    ├── unpack.rs                                  [移出 ~150 行]
    ├── pack.rs                                    [✨ ~120 行]
    ├── presets.rs                                 [移出 ~40 行]
    └── tests.rs                                   [~200 行]

software/sim-runtime/                              [✨ 新 crate]
├── Cargo.toml
├── build.rs                                       [prost_build]
└── src/
    ├── lib.rs                                     [~40 行]
    ├── runtime.rs                                 [~260 行]
    ├── config.rs                                  [~80 行]
    ├── clock.rs                                   [~60 行]
    ├── registry.rs                                [~120 行]
    ├── snapshot_broker.rs                         [~80 行]
    ├── actuation_sender.rs                        [~100 行]
    ├── health.rs                                  [~150 行]
    ├── supervisor.rs                              [~120 行]
    ├── errors.rs                                  [~60 行]
    ├── backend/
    │   ├── mod.rs                                 [~60 行 trait]
    │   ├── child_process.rs                       [~240 行]
    │   ├── external_socket.rs                     [~100 行]
    │   ├── mock.rs                                [test-only]
    │   ├── transport.rs                           [~160 行]
    │   └── runtime_dir.rs                         [~80 行]
    ├── ipc/
    │   ├── mod.rs
    │   ├── framing.rs                             [~50 行]
    │   ├── codec.rs                               [~60 行]
    │   └── handshake.rs                           [~120 行]
    ├── proto/world.rs                             [🤖 prost 生成]
    └── tests.rs                                   [~300 行]

software/sim-bridges/                              [✨ 新顶级目录]
└── st3215-compat-bridge/                          [✨ 新 crate]
    ├── Cargo.toml                                 [deps: sim-runtime, st3215-wire]
    ├── src/
    │   ├── lib.rs                                 [~80 行]
    │   ├── config.rs                              [~60 行]
    │   ├── actuator_map.rs                        [~100 行]
    │   ├── preset_loader.rs                       [~80 行]
    │   ├── command_task.rs                        [~140 行]
    │   ├── state_task.rs                          [~120 行]
    │   ├── health_task.rs                         [~60 行]
    │   ├── errors.rs                              [~50 行]
    │   └── tests.rs                               [~200 行]
    └── presets/
        └── elrobot-follower.yaml                  [✨ ~80 行]

software/sim-server/                               [✨ 新顶级目录]
├── pyproject.toml
├── README.md                                      [含 smoke checklist]
├── norma_sim/
│   ├── __init__.py
│   ├── __main__.py                                [~10 行]
│   ├── cli.py                                     [~100 行]
│   ├── logging_setup.py                           [~60 行]
│   ├── world/
│   │   ├── __init__.py
│   │   ├── manifest.py                            [~150 行]
│   │   ├── model.py                               [~120 行]
│   │   ├── descriptor.py                          [~100 行]
│   │   ├── actuation.py                           [~100 行]
│   │   ├── snapshot.py                            [~80 行]
│   │   └── capabilities.py                        [★ ~140 行]
│   ├── scheduler/
│   │   ├── __init__.py
│   │   ├── base.py                                [~40 行 protocol]
│   │   └── realtime.py                            [~160 行]
│   └── ipc/
│       ├── __init__.py
│       ├── framing.py                             [~60 行]
│       ├── codec.py                               [~60 行]
│       ├── server.py                              [~140 行]
│       └── session.py                             [~160 行]
├── scripts/
│   ├── inspect.py                                 [~100 行]
│   ├── probe_manifest.py                          [~150 行]
│   └── send_actuation.py                          [~120 行]
└── tests/
    ├── conftest.py
    ├── world/                                     [~600 行]
    ├── ipc/                                       [~300 行]
    └── integration/                               [~400 行]

software/station/bin/station/
├── station-sim.yaml                               [✨ ~50 行]
├── station-sim-external.yaml                      [✨ ~30 行]
└── station-shadow.yaml                            [✨ ~50 行]

docs/superpowers/specs/
├── 2026-04-10-simulation-integration-design.md                       [本文档 v2]
└── 2026-04-10-simulation-integration-v1-driver-shim-archive.md       [v1 归档参考]
```

### 14.2 改动文件

| 文件 | 改动 | 风险 |
|---|---|---|
| `Cargo.toml` (workspace root) | 加 `st3215-wire` / `sim-runtime` / `st3215-compat-bridge` 为 members | 零 |
| `software/drivers/st3215/Cargo.toml` | 加 `st3215-wire = { path = "..." }` 依赖 | 零 |
| `software/drivers/st3215/src/` | `use st3215_wire::{...}` 替换 inline 类型；`port.rs::scan_motors` parse 走 `st3215_wire::unpack` | **纯 code move，无逻辑变化**。既有测试是 regression guard |
| `software/station/bin/station/Cargo.toml` | 加 `sim-runtime` + `st3215-compat-bridge` 依赖 | 零 |
| `software/station/bin/station/src/main.rs` | `~80 行` 增量：`SimulationRuntime::start` + `start_st3215_compat_bridge` + 顺序化 shutdown | 触碰 main 路径，需仔细测 |
| `software/station/shared/station-iface/src/config.rs` | 新 `SimRuntimeConfig` / `Bridges` / `SimMode` / `LogCapture` / `St3215CompatBridgeConfig` + `Config::validate()` | 零逻辑风险 |
| `Makefile` | 追加 `sim-install` / `sim-run` / `sim-standalone` / `sim-shadow` / `sim-test` / `regen-mjcf` / `probe-manifest` + 架构不变量检查 | 零 |
| `.gitignore` | `/tmp/norma-sim*` / `sim-backend.log` / `sim-runtime.log` | 零 |

**零改动**：web UI / 所有非-st3215 drivers / station_py / gremlin_go / gremlin_py / URDF / STL / CAD / NormFS 内核。

### 14.3 代码量估算

| 类别 | v2 新代码 | v2 移动代码 |
|---|---|---|
| Rust `sim-runtime` | ~1700 行 | 0 |
| Rust `st3215-wire` | ~500 行 | ~400 行（从 st3215 移入）|
| Rust `st3215-compat-bridge` | ~890 行 | 0 |
| Rust `st3215` 真机改动 | 0 | 纯 `use` 语句 |
| Rust Station main 增量 | ~80 行 | 0 |
| Rust station-iface | ~200 行 | 0 |
| Python `norma_sim` | ~2000 行（含测试） | 0 |
| Proto (`world.proto`) | ~180 行 | 0 |
| MJCF + manifest + gen.py | ~750 行 | 0 |
| Config YAML 示例 | ~130 行 | 0 |
| **合计新代码** | **~6430** | **~400** |

v2 比 v1（估 ~2440 行）多约 **~4000 行新代码**（~1.7x）。这是架构正确性的标价。在 AI 辅助编码下是合理投资。

### 14.4 构建依赖顺序

```
L0: hardware/.../worlds/elrobot_follower.world.yaml   (gen.py 派生)
L0: protobufs/sim/world.proto
            ↓ make protobuf
L1: Rust world proto                L1: Python world proto
            ↓                              ↓
L2: sim-runtime crate               L2: norma_sim package
            ↓                              ↓
L3: st3215-wire crate (与 L2 并行)
            ↓
L4: st3215-compat-bridge crate (deps sim-runtime + st3215-wire)
L4: st3215 real driver (deps st3215-wire)
            ↓
L5: station binary (deps sim-runtime + st3215-compat-bridge)
            ↓
L6: 运行时: station → spawn norma_sim → bridge 订阅 runtime
```

### 14.5 首次启动命令序列

```bash
# 一次性准备
pip install -e software/sim-server/
make protobuf
make regen-mjcf                # 仅 URDF 或 world.yaml 变动时必要
cargo build -p station

# 运行（场景 A：纯 sim）
./target/debug/station -c software/station/bin/station/station-sim.yaml

# 或场景 B：External 模式
# Terminal 1:
python -m norma_sim \
  --manifest hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml \
  --socket /tmp/norma-sim-dev.sock
# Terminal 2:
./target/debug/station -c software/station/bin/station/station-sim-external.yaml

# 或场景 C：Shadow 模式
./target/debug/station -c software/station/bin/station/station-shadow.yaml
```

---

## 15. 里程碑拆分

### 15.1 MVP-1 = 本 spec 的完整 scope

参见 §3.1 Goals 和 §13.6 DoD。

### 15.2 MVP-2（紧接里程碑）

**增量范围**：

- 新 crate `software/sim-bridges/usbvideo-compat-bridge/`（~400 行）
- `norma_sim/world/rendering.py` + `snapshot.py` 扩展（~200 行 MuJoCo Renderer）
- `world.yaml` 的 `sensors:` 段加 `CAMERA_RGB` capability
- `gen.py` 根据 manifest sensors 生成 MJCF `<camera>` 标签
- `world.proto` **零 schema 改动**（CameraFrame slot 在 MVP-1 已预留）
- Station `main.rs` `~20 行` 增量启动 usbvideo-compat-bridge

**新问题要解决**：MuJoCo OpenGL context 初始化、渲染频率（30 FPS）与物理频率（500 Hz）解耦、帧编码与既有 usbvideo 输出格式对齐。

**DoD**：NormVLA 推理回路端到端跑通（关节 + 假图像）。

**★ 关键架构验证**：**MVP-2 不应该需要碰 `sim-runtime` crate**。如果碰了，说明 MVP-1 的抽象不够通用。这是对 MVP-1 架构质量的延迟 acceptance test。

### 15.3 Future Work（不在本 spec）

每条都是独立的 brainstorming → spec → implementation cycle：

- **MVP-3**：桌面 + 可抓物体 + pick-and-place 场景
- **MVP-4**：Leader 臂 URDF（需硬件团队建模）+ 遥操作 sim 版、PGripper 独立 capability、SO-101 机型
- **MVP-5**：`TickDrivenScheduler` impl + 确定性 + replay + GitHub Actions CI 集成
- **MVP-6**：`NativeBackend`（`mujoco-rs` in-process）作为 `ChildProcessBackend` 的双实现
- **升级到场景 B**：数据生成管道、domain randomization、轨迹录制
- **升级到场景 C**：sim-to-real 策略评估（**shadow mode MVP-1 已支持**）
- **升级到场景 F**：并行 sim、GPU 加速（可能换 Isaac Lab 或 Genesis 作第三 backend）

---

## 16. Rejected Alternatives

### 16.1 继承 v1 的拒绝（理由不变）

- **Level 1 kinematic snap mock / Level 2 kinematic + velocity limits**：无接触物理，夹爪抓取验证无法做
- **架构 B (Python 完全替代 Station)**：两套实现同步成本极高，不 exercise Station Rust 代码路径
- **PyBullet**：接触物理不如 MuJoCo 稳
- **Drake**：过度工程
- **Isaac Sim**：闭源 + GPU 依赖违背项目"可负担"调性
- **Genesis**：生态不成熟
- **Gazebo**：属于 ROS 生态，Station 不是 ROS-based

### 16.2 v1 拒绝但 v2 重评

| 选项 | v1 理由 | v2 立场 |
|---|---|---|
| **架构 A** (Rust in-process FFI `mujoco-rs`) | "Rust 绑定成熟度不够" | **部分接受为 future MVP-6**。`WorldBackend` trait 的存在让未来添加 `NativeBackend` 是零 architecture 改动的增量，不是替换 |
| **复用 NormFS 做主 IPC** | v1 没讨论 | **明确拒绝**。NormFS 是持久化队列不是闭环控制数据面，延迟和 durability 不匹配。可以把 world tick 镜像到 NormFS 做 replay/debug，但不做主通道 |
| **WASM 承载 MuJoCo** | — | **明确拒绝**。MuJoCo 无 WASM 移植，生态不在 |

### 16.3 v2 新增的拒绝

| 选项 | 拒绝理由 |
|---|---|
| **Top-level `robots:` config section** | 需要 real driver 也 robot-aware，是独立更大工作。MVP-1 在 schema + bridge config 层承认 `robot_id` 是一等概念，足够支撑 shadow mode 和 multi-robot 未来 |
| **Shared memory ring buffer 做数据面** | MVP-1 无相机，UDS + protobuf 吞吐够 100Hz snapshot + actuation。MVP-2 加相机后再评估 |
| **ZeroMQ / Zenoh / DDS 作 IPC** | codex §5.3 明确指出这只换 plumbing，不修"把世界建模成 fake ST3215"的根问题。v2 的核心修复在 schema 而非 transport |
| **`st3215` vs `st3215-compat-bridge` mutual exclusion** | 被 `bus_serial` 命名空间取代。shadow mode 是第一类场景 |
| **`PresetLoader` + `preset_by_name("elrobot-follower")` 在 `st3215` crate 中** | 机器人 SKU 目录不进协议库。改为 bridge crate 下 yaml 数据文件 |
| **`St3215CompatProjection` 作为单向只读组件** | 改为 `St3215CompatBridge` 双向——命令流从**全局 `commands` queue**（按 `StationCommandType::StcSt3215Command` + `target_bus_serial` 过滤）进 bridge 也要翻译 |
| **Deterministic tick-driven mode impl** | MVP-5。本 spec 仅在 schema 预留 `as_of.world_tick` 和 `TickDrivenScheduler` 枚举占位 |
| **自动重启 / 自动重连 / heartbeat / circuit breaker** | 保留 v1 立场——自动恢复会掩盖真 bug |
| **Python sim pip install 由 Station 自动完成** | Station 不做 Python 包管理。`launcher` 对 Station 完全不透明 |
| **`WorldBackend` trait 公开为 `pub`** | 第三方 crate 不应能 impl 自定义 backend——subsystem 边界必须编译期强制。新 backend 必须进 sim-runtime crate 本身 |
| **`ActuationCommand` 用多个显式 message (`SetPositionRad/SetPositionM/SetGripperNormalized`)** | Capability-keyed 单一 `SetPosition` + `capability.kind` 决定单位更优雅。类型安全损失一点，概念统一换回来 |
| **`SimHealth` 作为独立 queue 类型而非 NormFS queue** | 复用 NormFS 模式，不发明新机制 |

---

## 17. 风险 & 未解问题

### 17.1 技术风险

| # | 风险 | 影响 | 缓解 |
|---|---|---|---|
| X1 | `<equality>` polycoef 数值不稳定，夹爪联动不收敛 | ★ MVP-1 核心 demo 失败 | `test_mimic_gripper_equality_works` P0 早期验证；备选 `<tendon><fixed>`（README troubleshooting） |
| X2 | `st3215-wire` 迁移引入 real driver regression | ★ 真机不能用，用户信任崩 | 迁移只做 code move 不改逻辑；real driver 既有测试是 regression guard；`test_pack_roundtrip_via_unpack` 双向验证；`test_unpack_preserved_regression` 用既有 fixture |
| X3 | `WorldBackend` trait 形状实现时发现不对 | 实现阶段返工 | trait 只搬 Envelope（transport-level），domain 全在 runtime 侧——trait 表面积小，返工成本低 |
| X4 | `GRIPPER_PARALLEL` capability 对未来 pgripper 不适用 | MVP-4 加 pgripper 时需要新 capability | `CAP_*` enum 保留扩展空间；pgripper 可以加 `CAP_GRIPPER_PGRIPPER` 或通用化 |
| X5 | `ChildProcessBackend` socket readiness 轮询（50ms）在慢机器命中 5s timeout 假阳性 | 启动失败误报 | `startup-timeout-ms` 可配；错误消息含 log 文件路径；未来可用 inotify |
| X6 | `sim-runtime` crate 编译时间增加 > 60s | 开发体验变差 | prost + tokio 已在 workspace，增量编译开销可控 |
| X7 | Rust `prost` 和 Python `gremlin_py` 生成的 proto wire format 不兼容 | 跨语言 IPC decode 错 | `test_handshake_happy_path` 跨语言集成测试；基于同一 .proto 源 |
| X8 | `world.yaml` schema 作为"一份文件两用途"后续演进成本 | 加字段要同时考虑 build 和 runtime | manifest loader 明确版本号字段；未来 schema 变更走独立 spec |
| X9 | `bus_serial` 命名空间 + 既有 `auto` logic bug（多 bus 下行为未定义） | shadow 模式 `inference.st3215-bus` 必须显式配置否则不对 | **spec 明文要求** shadow mode 显式配置；smoke test 含对照；修 `auto` bug 是独立 spec |
| X10 | 大 MJCF 上 MuJoCo 初始化 > 2s，handshake timeout 假阳性 | 启动失败 | `startup-timeout-ms` 可配；handshake timeout 建议 ≥ 3s buffer |

### 17.2 进程与组织风险

| # | 风险 | 缓解 |
|---|---|---|
| P1 | v2 代码量 ~6400 行，MVP-1 实现周期显著增加 | 按 `sim-runtime → st3215-wire → norma_sim world → st3215-compat-bridge → norma_sim ipc → Station integration` 依赖顺序分步 merge，避免大 bang PR |
| P2 | `WorldBackend` 抽象可能是 "premature abstraction" | MVP-1 有 2 个 prod impl（ChildProcess + External）+ 1 test impl（Mock），不是"只有一个实现的抽象"——trait 面对现实需求 |
| P3 | Capability-keyed schema 在 MVP-2 相机时发现不够 | `SensorSample.oneof payload` 已预留 CameraFrame slot；`CAP_CAMERA_RGB` enum 已存在 |
| P4 | v2 架构对实现者认知负担比 v1 高 | 本 spec 详细给出 API 签名、模块布局、启动顺序；writing-plans 阶段进一步拆成可独立实现的 tasks |

### 17.3 明确不是风险

- ✅ `sim-runtime` 对 Station 既有 drivers 的影响：零（零触碰非-st3215 driver）
- ✅ Python 依赖冲突：`norma_sim` 只依赖 `mujoco + numpy + protobuf`，与 `gremlin_py / station_py` 无冲突
- ✅ Station 二进制大小：新增 crate < 1 MB
- ✅ Rust prost codegen 方式（v1 review 已确认 `prost_build`）

---

## 18. 与 v1 的结构性差异 — 直接对账 codex Top 5

| # | Codex 批评（v1） | v2 对应 |
|---|---|---|
| **1** | "把 sim 提升为 Station 内的一等 `SimulationRuntime`，不要定义为 fake `st3215` driver" | §5 整体架构 + §6.1 `sim-runtime` crate：一等 subsystem，和 NormFS/tcp/web 平级。`st3215-sim` driver 概念彻底消失，被 `St3215CompatBridge` 取代 |
| **2** | "`sim.proto` 改成通用 world/actuator/sensor schema，capability-keyed；ST3215 bytes 只在兼容层" | §6.5 `world.proto`：`RobotRef + ActuatorRef + capability-keyed ActuationCommand`。Python 包零 ST3215 痕迹（CI 强制）。ST3215 bytes 只在 `st3215-compat-bridge` |
| **3** | "拆开 `physics_hz / publish_hz / render_hz`，real-time 降级为 scheduler policy" | §8 时间模型：三独立频率、整除约束、`world_tick` canonical、`RealTimeScheduler` vs 预留 `TickDrivenScheduler` |
| **4** | "引入显式 supervisor/runtime handle 和 health stream，消除隐式副通道" | §6.4 Station 显式持有 `Arc<SimulationRuntime>`；§6.5 `/sim/health` NormFS queue + broadcast；§11.5 bridge 通过订阅感知 backend 死活。`sim_manager` 消失 |
| **5** | "抽 `st3215-wire`，pack/unpack 在其中；机器人 preset 不准进 real driver crate" | §6.2 `st3215-wire` 独立 crate（纯协议）；§6.3 preset 是 `st3215-compat-bridge/presets/*.yaml` 数据文件，不在 `st3215` real driver crate |

### 18.1 v2 第一轮 double-review 的收口

本 spec v2 的第一稿完成后，立即跑了两路 review：spec-document-reviewer subagent（事实性校核）+ codex 同 session 续问（架构深度验证）。两路各自发现了应该收口的问题，都在本节之前的正文中已逐条修正。本节记录对账，避免未来 review 再提相同问题。

**Codex second-opinion 两个 blocker（都已修）**：

| # | Codex 发现 | v2 修复位置 |
|---|---|---|
| B1 | Bridge 上行订阅 `st3215/commands` 但真实系统入口是全局 `commands` queue，是凭空发明的 queue 名，破坏 `station_py / web UI 零修改`承诺 | §6.3 `command_task` 描述彻底改写：订阅 `station_iface::COMMANDS_QUEUE_ID = "commands"`，按 `StationCommandType::StcSt3215Command` 和 `target_bus_serial` 双重过滤，逐字 mirror 真 driver 的 `driver.rs:74-87` 逻辑 |
| B2 | Shadow mode 的 `legacy_bus_serial` 冲突检测靠"启动时扫 `st3215/meta`"，但 real driver 的 bus 是异步 1s 扫串口发现的，check 是 racy 弱机制 | §6.3.a 新增：`legacy_bus_serial` 必须以 `sim://` 为前缀的**结构性不变量**。USB 设备的 serial number 字段永远不会出现 `sim://`，所以冲突不可能发生，不需要 detection，不需要 registry |

**Codex second-opinion 其它硬核发现（都已修）**：

| # | Codex 发现 | v2 修复 |
|---|---|---|
| F1 | `world.yaml` "一份文件两用途" 说法过头——实际是 `urdf + world.yaml + mjcf` 三份，runtime 可能 out of sync | §9.3 新增 MJCF `source_hash` XML comment 机制：gen.py 写入 `sha256(urdf + world.yaml)`，runtime 启动时比对 |
| F2 | capability-keyed schema 依赖 descriptor 解释语义，但 descriptor 仅在内存 API 和 handshake 中，没持久化，离线 replay 无法解释历史命令 | §6.1 新增 `/sim/descriptor` NormFS queue：`SimulationRuntime::start()` 完成 handshake 后立即写入一条 `WorldDescriptor` message（带 `runtime_session_id`），供归档 / replay / 未来 web UI 订阅 |
| F3 | 配置层仍不是 world-first (`inference.st3215-bus` 仍 bus-centric) | **明确不修**——§10.7 说明 top-level `robots:` 段是独立 spec 工作，MVP-1 在 schema + bridge config 层承认 `robot_id` 一等即可；这是已知债务，接受 |

**spec-document-reviewer 第一轮的 5 个 major（都已修）**：

| # | 发现 | v2 修复 |
|---|---|---|
| S1 | `Station::engine` 实际类型是 `Arc<Engine>` 具体结构，不是 `Arc<dyn StationEngine>` | §6.4 struct 更新为 `pub engine: Arc<Engine>`，加 `Engine impl StationEngine` 说明 + `Arc<Engine> → Arc<dyn StationEngine>` 自动 unsized coercion 注 |
| S2 | `Station::shutdown_drivers()` 不存在，唯一既有方法是 `Station::shutdown()` | §6.4 shutdown 顺序改用 `station.shutdown()`（既有），并说明既有 drivers 无显式 shutdown 路径、本 spec 不修 |
| S3 | `Config::inference` 实际是 `Option<Vec<Inference>>`，spec 写成 `Vec<InferenceConfig>` | §10.5 改正类型名和 Option 包装 |
| S4 | `St3215Config` 没有 `buses: [{ port: ... }]` 字段，real driver 通过 USB 枚举自动发现端口 | §10.6 Scenario C 删除 `buses:` 行，改为 `enabled / current-threshold / deadband` 等真实字段 |
| S5 | 所有场景 yaml 的 `inference` 段缺必填字段 `shm` 和 `shm-size-mb` | §10.6 三个场景 yaml 都补上；shadow 模式用两条独立 shm 路径 |

**spec-document-reviewer 第一轮的 7 个 minor（都已处理）**：见 §6.2 迁移表扩展（packet.rs/devices.rs/port.rs 边界）、§6.5 Hello/Welcome 协议版本注释、§13.3 `health_task` 专项测试、§13.3 Bridge crate 测试从 8 升到 10、§6.1 `SimHealth` 字段 MVP-1 必填/选填分层说明、§7.1 `world/` 三文件边界测试断言、§10.1 MVP-1 只支持单 yaml 的显式声明、§9.3 gen.py self-check 补充 `mjcf_actuator` 名字、`compiler.angle="radian"`、`<equality>` polycoef 系数三条。

### 18.2 codex 其它硬核发现的对应

| Codex 发现 | v2 解法 |
|---|---|
| **v1 §4.1 物理时间步进 bug**（`state_rate_hz=100` + `timestep=0.002` 导致 sim 5 倍慢）| §8.4 `RealTimeScheduler` 主循环；`physics_hz` 必须整除 `publish_hz`；每个 tick 真实 mj_step |
| **Mutual exclusion 锁死 shadow mode** | §10.2 删除 `Drivers::validate()` 互斥规则；§10.6 shadow mode 作为场景 C 完整示例 |
| **命令通道 QoS 混淆**（连续 setpoint + 离散动作共用 drop-oldest） | §6.5 `QosLane` + §6.1 `ActuationSender` 双通道；`LOSSY_SETPOINT` drop-oldest，`RELIABLE_CONTROL` backpressure |
| **Station 实际没保留 driver handle** | §6.4 `Station::sim_runtime: Option<Arc<SimulationRuntime>>` 显式字段；§11.5 bridge 通过 broadcast API，不走隐式副通道 |
| **`sentinel + lockfile + socket` 状态机爆炸** | §6.3 `TempRuntimeDir` RAII 单点；socket-as-readiness |
| **`ChildProcessBackend` vs `ExternalSocketBackend` 状态机差异** | §6.1 trait 就位，两个 impl 共享 `UnixSocketTransport`，backend 分工干净 |

---

## 附录 A：MVP-1 手工 Smoke Test Checklist

放入 `software/sim-server/README.md`。

### Prereqs
- [ ] Python 3.11+
- [ ] `make sim-install` 成功
- [ ] `make protobuf` 成功
- [ ] `cargo build -p station` 成功
- [ ] `hardware/elrobot/simulation/worlds/elrobot_follower.xml` 存在
- [ ] `probe_manifest.py` 对 `elrobot_follower.world.yaml` 全绿

### 场景 A：Internal 模式（纯 sim）

- [ ] `./target/debug/station -c software/station/bin/station/station-sim.yaml`
- [ ] stderr 看到 `sim-runtime started session_id=... runtime_dir=/tmp/norma-sim-... startup_duration=<3s`
- [ ] stderr 看到 `handshake complete protocol_version=1 world_name=elrobot_follower_empty actuators=8`
- [ ] stderr 看到 `bridge started robot_id=elrobot_follower legacy_bus_serial=sim://bus0`
- [ ] 浏览器打开 `http://localhost:8889`
- [ ] ElRobot follower 3D 模型加载
- [ ] 左侧显示 `sim://bus0` 和 8 个 motor 全 online，position ≈ 0
- [ ] 拖 motor 1 滑条 → 3D 视图 `Joint_01` 转动
- [ ] **拖 motor 8 → 夹爪 `Gripper_Jaw_01` 和 `Gripper_Jaw_02` 同步开合** ★ (equality 验证)
- [ ] `station-queue-dump /sim/health | head -5` 显示合理的 1Hz 输出
- [ ] Ctrl+C Station → 干净退出
- [ ] `ls /tmp/norma-sim-*` 为空
- [ ] `ps -ef | grep norma_sim` 无残留

### 场景 B：External 模式

- [ ] 终端 1：`python -m norma_sim --manifest hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml --socket /tmp/norma-sim-dev.sock --physics-hz 500 --publish-hz 100`
- [ ] 终端 1 stderr JSON 输出看到 `{"component":"norma_sim.ipc.server","msg":"listening","world_tick":0,...}`
- [ ] 终端 2：`./target/debug/station -c software/station/bin/station/station-sim-external.yaml`
- [ ] 终端 2 看到 `handshake complete` + `bridge started`
- [ ] 浏览器操作同场景 A 的 motor 拖拽
- [ ] 终端 1 Ctrl+C → 终端 2 看到 `sim-runtime: backend terminated cause=CLEAN`
- [ ] 终端 2 bridge 写出 offline marker 到 st3215/meta
- [ ] Web UI 显示 "总线离线"
- [ ] Station 不崩，其它 driver 继续
- [ ] 终端 2 Ctrl+C → 干净退出

### 场景 C：Shadow 模式（需真机或 real driver mock）

- [ ] `./target/debug/station -c software/station/bin/station/station-shadow.yaml`
- [ ] stderr 同时看到 `st3215 driver: bus discovered serial=ST3215-BUS-...` **和** `bridge started legacy_bus_serial=sim://elrobot-shadow`
- [ ] 浏览器看到两条 bus 并排显示
- [ ] `station-queue-dump st3215/inference | head -20` 看到两个不同 bus_serial 的条目交替
- [ ] `station-queue-dump inference/normvla-sim` 有数据
- [ ] `station-queue-dump inference/normvla-real` 有数据（如有真机）
- [ ] Ctrl+C → 真机和 sim 都干净退出

### 故障场景

- [ ] `mjcf-path` 指向不存在文件 → Station exit 1 + `ERROR: ... failed to load manifest`
- [ ] `launcher[0] = /nonexistent-python` → Station exit 1 + `ERROR: ... launcher command not found`
- [ ] `bridges.st3215_compat.enabled=true` 但 `sim-runtime.enabled=false` → Station exit 1 + `ConfigError::BridgeRequiresSimRuntime`
- [ ] Internal 模式无 `launcher` → `ConfigError::MissingLauncher`
- [ ] External 模式无 `socket-path` → `ConfigError::MissingExternalSocketPath`
- [ ] External 模式 socket-path 不存在 → `BackendError::ExternalNotRunning`

### 架构不变量（命令检查）

- [ ] `grep -ri "st3215" software/sim-runtime/src/` 零命中
- [ ] `grep -ri "st3215" software/sim-server/norma_sim/` 零命中
- [ ] `grep -ri "tokio\|normfs\|station_iface" software/drivers/st3215-wire/src/` 零命中
- [ ] `cargo tree -p sim-runtime | grep st3215-wire` 零命中
- [ ] `cargo test --workspace` 全绿
- [ ] `pytest software/sim-server/tests/` 全绿

---

## 附录 B：Glossary

| 术语 | 含义 |
|---|---|
| **Station** | NormaCore 的中心控制服务（Rust 二进制，单一进程）|
| **NormFS** | 项目自研持久化队列存储系统 |
| **Gremlin** | 项目自研高性能 protobuf 实现（Go + Python 两套）|
| **NormVLA** | ML 推理帧格式（关节 + 图像），用于 imitation learning |
| **ST3215** | Feetech 串行总线舵机，ElRobot 的动力元件 |
| **ElRobot** | 项目自研的 7+1 DoF 3D 打印机械臂 |
| **Follower / Leader** | Follower 是执行臂，Leader 是遥操作臂（低减速比利于反向驱动）|
| **URDF** | Unified Robot Description Format — 机械结构描述 |
| **MJCF** | MuJoCo 的原生场景描述格式（XML），功能集优于 URDF |
| **Equality constraint** | MJCF `<equality>` 标签定义的约束，替代 URDF `<mimic>` |
| **Polycoef** | MuJoCo `<equality>` 的多项式系数，`joint1 = c0 + c1*joint2 + ...` |
| **SimulationRuntime** | ★ v2 新概念：Station 内的一等 subsystem，拥有共享世界状态 |
| **WorldBackend** | ★ v2 新概念：`SimulationRuntime` 的内部 trait，抽象 sim 实现（child process / external / native / mock）|
| **ChildProcessBackend** | `WorldBackend` 的一个 impl：Station spawn 一个子进程并管其生命周期 |
| **ExternalSocketBackend** | `WorldBackend` 的一个 impl：只 connect 已存在的 sim 进程，不管 lifecycle |
| **WorldClock** | ★ v2 新概念：`{world_tick, sim_time_ns, wall_time_ns}`，`world_tick` 是 canonical time |
| **WorldDescriptor** | Handshake 时 sim → Station 发的 world 自描述，含所有 robots/actuators/sensors 的 capability |
| **WorldSnapshot** | 运行时 sim → Station 的状态帧，含 `WorldClock` + 所有 actuator state + sensor samples |
| **ActuationCommand** | Station → sim 的单个命令，按 capability-keyed `oneof intent` |
| **ActuationBatch** | 多个 ActuationCommand 的批次，带 QoS lane |
| **ActuatorRef / SensorRef** | 字符串寻址：`(robot_id, actuator_id)`，不是 bus+motor_id |
| **ActuatorCapability** | ★ v2 新概念：`CAP_REVOLUTE_POSITION / CAP_GRIPPER_PARALLEL / ...`，决定单位和语义 |
| **CAP_GRIPPER_PARALLEL** | Gripper 能力，外部值 0..1 归一化，内部是主 joint rad + 两个 mimic 的 equality |
| **QosLane** | `QOS_LOSSY_SETPOINT`（连续 setpoint，drop-oldest）vs `QOS_RELIABLE_CONTROL`（离散动作，backpressure）|
| **Sim-runtime / sim-runtime crate** | Rust crate 实现 `SimulationRuntime` 的新 subsystem |
| **St3215CompatBridge** | ★ v2 新概念：双向 Bridge (GoF pattern)，commands 上行 + snapshots 下行，翻译 legacy ST3215 queue ↔ generic world schema |
| **st3215-wire** | ★ v2 新概念：纯协议 crate，无 tokio/normfs 依赖，real driver 和 sim bridge 共享 |
| **TempRuntimeDir** | ★ v2 新概念：Station-owned 临时目录，RAII 管理，取代 v1 的 sentinel+lock+socket 三件套 |
| **norma_sim** | Python 包名（下划线 import），distribution 名 `norma-sim`（连字符）|
| **world.yaml / manifest** | ★ v2 新概念：机器人 world-level 元数据，是 gen.py 的输入和 norma_sim runtime 的输入（一份文件两用途）|
| **gen.py** | 从 `world.yaml` + URDF 派生 MJCF 的脚本 |
| **runtime_dir** | `TempRuntimeDir` 的文件系统表现，存放 socket 和可选日志 |
| **Shadow mode** | ★ v2 新概念：real driver 和 sim bridge 通过 `bus_serial` 命名空间共存，支持 digital twin 和 sim-to-real 对照 |
| **SimHealth** | ★ v2 新概念：`/sim/health` queue 的 schema，含 `world_tick`、调度指标、终止原因 |
| **Session ID (runtime_session_id)** | 每次 `SimulationRuntime::start` 生成的 UUID，区分 session 作日志关联 |

---

**本文档结束**

下一步：spec-document-reviewer subagent review → codex second-opinion review（session `019d7726-6dcf-7fe2-8887-35ee3b9c2568` 续问）→ user final review → writing-plans skill 拆实现 plan
