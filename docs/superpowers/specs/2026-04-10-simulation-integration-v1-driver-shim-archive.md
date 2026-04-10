# NormaCore 仿真集成设计

| | |
|---|---|
| **日期** | 2026-04-10 |
| **状态** | Draft — 待 review |
| **范围** | MVP-1（首交付里程碑）+ MVP-2（紧接里程碑）概要 |
| **驱动场景** | 无硬件开发 / 调试（Station 在无真机环境下完整运行）|

---

## 1. TL;DR

给 Station 加一个**物理驱动的仿真后端**，让它能在没有 ElRobot 真机的情况下跑完整代码路径。

**技术总览**：
- 物理引擎 **MuJoCo**（Apache 2.0；IL 研究事实标准；接触物理强；URDF 导入成熟）
- **混合架构**：Station 里新建 `st3215-sim` Rust driver 作为 shim（薄连接客户端），外部独立 Python 进程 `norma_sim` 跑 MuJoCo 物理并通过 unix socket + protobuf 跟 driver 通信
- **Station `sim_manager` 模块**统一管理 sim-server 子进程的生命周期（可选 external 模式，dev 独立迭代）
- **Python 发 SI 单位的 JointState，Rust 本地把状态打包成 ST3215 寄存器字节**写进 inference queue —— 下游 web UI 和客户端零修改
- **URDF → MJCF 由 `gen.py` 派生脚本生成并 check in**，关键点是补写 `<equality>` 约束修复 URDF `<mimic>` 关节（MuJoCo 不支持 mimic）
- **MVP-1 范围**：ElRobot follower 8 关节 + 夹爪手指开合 + 空世界；MVP-2 加摄像头

---

## 2. 背景与动机

### 2.1 当前状态

项目是一个面向物理 AI 研究和机器人自动化的端到端平台：自研硬件（ElRobot 7+1 DoF 机械臂、PGripper 夹爪）+ Station 软件平台（Rust 单二进制 + Rust drivers + Web UI + Python/Go 客户端 SDK）。技术栈以 Rust 为主，辅以 Go（gremlin 高性能 protobuf）和 Python（客户端 SDK + gremlin 生成器）。

仓库里**已有的"仿真原料"**：
- `hardware/elrobot/simulation/elrobot_follower.urdf` —— follower 机械臂的完整 URDF（521 行，带 inertia/mass/visual/collision）
- `hardware/elrobot/simulation/assets/*.stl` —— 19 个 STL 网格资产
- `software/station/clients/station-viewer/public/{elrobot,so101}/` —— URDF 副本 + Three.js/urdf-loader 做 3D **可视化**（不是物理仿真）

**完全没有的**（全仓 grep 零命中）：
- 任何物理引擎集成（mujoco / pybullet / drake / isaac / genesis / gazebo）
- 任何 mock / fake / virtual / stub driver
- MJCF、leader 臂 URDF、pgripper URDF
- Python 侧 sim server 或示例

当前所谓 `simulation/` 目录的唯一用途是让 web UI 把真机数据流可视化成动画 —— 是"数据的 3D 投影"，不是"世界的仿真"。

### 2.2 驱动需求

**主要用户故事**：*"我是 dev，手上没接机械臂，现在想跑 Station、打开浏览器看到 web UI、发个命令让 ElRobot 在 UI 里动起来。"*

具体价值：
- Dev loop 不依赖硬件在手
- CI 能跑（将来）
- Web UI 和客户端 SDK 能在没真机时 dogfood
- 为将来的数据生成（场景 B）、sim-to-real（C）、RL 训练（F）搭基础

### 2.3 拒绝的替代方案

讨论过的 fidelity level 和被否决的架构：

| 被否决 | 原因 |
|---|---|
| **Level 1 kinematic snap mock** | 没有接触物理 = 夹爪抓取无法验证 = 白选"Level 3"标签 |
| **Level 2 kinematic + velocity limits** | 同上，且升级到 Level 3 代价不比重来小 |
| **架构 A：纯 Rust in-process FFI (mujoco-rs)** | Rust 绑定成熟度不够；写 sim 场景生产力差；升级到 B/C/F 时整个 Python IL 生态用不上；MVP 工作量差别不大但上限低得多 |
| **架构 B：Python 完全替代 Station** | 需要在 Python 重实现 Station 的 TCP/WS/queue/inference/driver 协议栈；两套实现同步成本极高；不 exercise Station Rust 代码路径 = 不满足"无硬件开发 Station"的目标 |
| **PyBullet / Drake / Isaac Sim / Genesis / Gazebo** | PyBullet 接触物理不如 MuJoCo 稳；Drake 过度工程；Isaac 闭源且 GPU 依赖违背项目"可负担"调性；Genesis 生态不成熟；Gazebo 属于 ROS 生态而 Station 不是 ROS-based |

---

## 3. Goals / Non-Goals

### 3.1 MVP-1 Goals

- `station -c station-sim.yaml` 一条命令启动，浏览器访问 web UI 可见 ElRobot follower 3D 模型动态响应命令
- 7 个主臂关节 + 1 个夹爪关节在 MuJoCo 物理里正确运动
- 夹爪的两个 prismatic mimic 关节通过 MJCF `<equality>` 约束正确联动（这是 Level 3 的核心价值证明点）
- `st3215-sim` driver 写出的 `st3215/inference` queue 数据与真机二进制兼容，下游 web UI / Python 客户端零修改
- 启动失败快错，运行失败软错（`st3215` 真机 driver 的错误语义对齐）
- Internal 模式（Station 自动 spawn sim-server）和 external 模式（dev 手动启动）都支持

### 3.2 MVP-2 Goals（紧接里程碑）

- 新增 `usbvideo-sim` Rust driver 和 sim-server 的 camera 渲染能力
- 至少 1~2 个 MuJoCo 渲染摄像头喂帧到 `usbvideo/frames` queue
- NormVLA 推理回路端到端跑通（关节 + 假图像）

### 3.3 显式 Non-Goals（MVP-1/2 都不做）

- ❌ Leader 臂（URDF 尚不存在，需硬件团队建模）
- ❌ PGripper（独立产品线，不混入 ElRobot MVP）
- ❌ SO-101 机型（先把 ElRobot 一条链路打通）
- ❌ 桌面场景 / 可抓取物体 / pick-and-place 任务
- ❌ 遥操作（motors-mirroring）的 sim 版本
- ❌ CI 自动化集成（Makefile target 有但不进 GitHub Actions）
- ❌ 确定性 / seed 控制 / 可复现性（tick-driven mode 留接口但 MVP 不实现）
- ❌ 性能 benchmark（吞吐、延迟指标）
- ❌ 自动重启 / 自动重连 / 健康检查 / 熔断
- ❌ 鉴权 / 加密 / 跨机器仿真
- ❌ Graceful degradation（sim 挂了不会回落 kinematic mock）
- ❌ 在线 GUI viewer（MuJoCo headless，可视化只走 web UI）
- ❌ Fuzzing / fault injection / 性能 profile 模式
- ❌ Systemd unit / docker compose / 二进制分发

---

## 4. 核心设计决定

| 决定 | 选择 | 关键理由 |
|---|---|---|
| Fidelity level | **Level 3（物理驱动）** | 接触物理是夹爪抓取验证的前提；为 B/C/F 铺路 |
| 物理引擎 | **MuJoCo** | IL/RL 研究事实标准；Apache 2.0；URDF 成熟；接触物理最稳；Python 绑定质量好 |
| Sim 进程位置 | **独立 Python 进程** | 复用成熟 Python MuJoCo 生态；sim 迭代不拖累 Rust build；天然升级到 B/C/F |
| Station 集成点 | **新建 `st3215-sim` Rust driver（shim）** | 保持 Station 的 driver 抽象 seam；上层零修改；互斥启用与 `st3215` |
| Subprocess 归属 | **Station `sim_manager` 模块**（不是 driver 管） | 多个 sim driver（MVP-2+ usbvideo-sim）需共享同一 MuJoCo 世界 |
| IPC 传输 | **Unix domain socket** | 本机通信；简单；权限 0600 即安全；tokio/asyncio 都有成熟支持 |
| IPC 编码 | **Length-prefixed protobuf**（新建 `protobufs/sim/sim.proto`）| 复用项目的 gremlin 生成管道；结构化；跨语言 |
| IPC 语义层次 | **Python 发 SI 单位 JointState，Rust 本地打包 ST3215 寄存器字节** | ST3215 协议知识 100% 留在 `st3215` crate；Python sim 通用化 |
| Sync 模式 | **Server push（state）+ Client push（commands）**，无 request/response | 简单；符合 state-based motor control 语义 |
| 时间模式 | **Real-time only**（MVP-1）| Web UI 观感自然；faster-than-real-time 留给 MVP-5 |
| MJCF 来源 | **`gen.py` 脚本派生 + check in git** | 可审查；URDF/MJCF 两份都在 repo；可 diff |
| URDF mimic 关节 | **改写为 MJCF `<equality>` polycoef 约束** | MuJoCo 不支持 URDF `<mimic>`；equality 是 idiomatic 做法 |
| 配置文件 | **新建 `station-sim.yaml`**（和 `station.yaml` 平级独立）| 现有真机 config 零修改；用户通过 `-c` 切换 |
| 错误哲学 | **启动期快错、运行期软错** | 启动沉默失败最糟糕；运行期 sim 断等同真机拔线语义 |
| 恢复策略 | **无自动重连 / 无自动重启**（MVP 手动）| 自动恢复会掩盖真 bug |
| CI | **MVP 不做**，只提供 Makefile target | MuJoCo wheel 在 CI runner 上依赖未知；先跑通人工 workflow |
| 测试策略 | **P0 风险有专门测试，靠手工 smoke 覆盖 E2E** | E2E 自动化成本高维护脆；手工 smoke 10 分钟可完成 |

---

## 5. 整体架构

```
┌───────────────────────────────────────────────────────────────────┐
│                         Developer Machine                         │
│                                                                    │
│   ┌──────────────────────────────┐                                │
│   │   Station (Rust 二进制)      │                                │
│   │                               │                                │
│   │   ┌─────────────────────┐    │    现有 drivers 照常:          │
│   │   │  sim_manager (新)    │───┼───┐ usbvideo, sysinfod,         │
│   │   │  spawn + 管子进程   │    │   │ motors-mirroring, inferences│
│   │   └──────────┬──────────┘    │   │                            │
│   │              │                │    ┌──────────────────────┐    │
│   │   ┌──────────▼──────────┐    │    │                      │    │
│   │   │  driver loader      │    │    │                      │    │
│   │   │                     │    │    │                      │    │
│   │   │  st3215-sim (新)     │◄──┼────┤                      │    │
│   │   │    ↕ unix socket    │    │    │                      │    │
│   │   └─────────────────────┘    │    │                      │    │
│   │              │                │    │                      │    │
│   │   ┌──────────▼──────────┐    │    │                      │    │
│   │   │ NormFS queues       │    │    │                      │    │
│   │   │ TCP API             │    │    │                      │    │
│   │   │ WebSocket           │    │    │                      │    │
│   │   │ station-viewer UI   │    │    │                      │    │
│   │   └─────────────────────┘    │    │                      │    │
│   └──────────────────────────────┘    │                      │    │
│                                        │                      │    │
│   ┌──────────────────────────────┐    │                      │    │
│   │  sim-server (Python, 独立进程) │◄───┘                      │    │
│   │                               │                                │
│   │  ・mujoco.MjModel/MjData      │                                │
│   │  ・mj_step 循环 (real-time)   │                                │
│   │  ・MJCF (从 URDF 派生)        │                                │
│   │  ・unix socket server         │                                │
│   │    • Hello/Welcome handshake  │                                │
│   │    • 收 MotorCommandBatch     │                                │
│   │    • 发 BusStateUpdate        │                                │
│   └──────────────────────────────┘                                │
└───────────────────────────────────────────────────────────────────┘
```

### 5.1 运行时不变量

1. **Station 主循环一行不改** —— 所有现有的 driver loader / queue / TCP/WS / Web UI 零修改
2. **`st3215-sim` 和 `st3215` 对外接口 100% 等价** —— 同样的 queue id（`st3215/inference`、`st3215/meta`、`st3215/rx`、`st3215/tx`），同样的 protobuf schema（`st3215::InferenceState::BusState`），同样订阅 `commands` queue，web UI 分不出差别
3. **`st3215-sim` 和 `st3215` 互斥启用** —— config loader 强制检查
4. **sim-server 进程由 Station `sim_manager` 统一管理**，driver 只是连接客户端
5. **Python sim-server 不依赖任何 Station Rust 代码**，只依赖 `gremlin_py` 生成的 protobuf + mujoco

### 5.2 为什么切在 driver 层

Station 的 driver 抽象是硬件适配层的天然 seam。所有下游（queue、TCP/WS、web UI、inference）对 driver 的契约是 protobuf 消息 schema，而非 driver 的内部实现。沿着这条缝插入一个"虚拟硬件"driver 是最不侵入的做法。

---

## 6. 组件设计

### 6.1 新 Rust crate: `st3215-sim`

**位置**：`software/drivers/st3215-sim/`，加入根 `Cargo.toml` workspace members。

**对外入口**：
```rust
pub async fn start_st3215_sim_driver<T: StationEngine>(
    normfs: Arc<NormFS>,
    station_engine: Arc<T>,
    config: St3215SimConfig,
    sim_socket_path: PathBuf,
) -> Result<Arc<St3215SimDriver>, Box<dyn std::error::Error>>;
```

签名与 `st3215::start_st3215_driver` 保持一致（`software/drivers/st3215/src/driver.rs:293`）—— 泛型 `T: StationEngine`、参数名 `station_engine`、返回 `Arc<Driver>` 包在 `Result<_, Box<dyn Error>>` 里 —— 只是多一个 `sim_socket_path`（由 `sim_manager` 传入）。

**模块布局**：
```
src/
├── lib.rs           — 入口
├── config.rs        — St3215SimConfig
├── ipc.rs           — unix socket 连接 + length-prefixed framing（LengthDelimitedCodec）
├── codec.rs         — protobuf encode/decode
├── command_loop.rs  — 订阅 commands queue → 翻译 → 发 socket
├── state_loop.rs    — 读 socket → 调 st3215::pack → 写 inference queue
├── errors.rs        — SimDriverError
└── proto/sim.rs     — prost 生成
```

**注意**：寄存器字节打包（`JointState → EEPROM+RAM bytes`）的实现 **不在** `st3215-sim` 里，而是在 `st3215` crate 新增 `protocol::pack` 模块对外公开（详见 §6.6）。这样 encoder 和 decoder（`port.rs` 里的既有解析）能共处一处，避免协议知识分散。`st3215-sim/state_loop.rs` 只是调 `st3215::protocol::pack::pack_state_bytes(joint_state, &preset)`。

**内部状态机**：`Starting → Ready → Running → {Stopping, Crashed} → Stopped`

**关键约束**：
- 写入与真机一致的 queue ID：`st3215/rx`（可留空）、`st3215/tx`（可留空）、`st3215/meta`（写合成元数据，bus_serial 用 `sim://bus0`）、`st3215/inference`（主数据流）
- `InferenceState::MotorState.state` 字段必须是与真 ST3215 二进制兼容的 **EEPROM + RAM 字节 dump**（71 字节；详细契约见 §6.6）

**与真 `st3215` driver 的差异**：
| 方面 | `st3215` | `st3215-sim` |
|---|---|---|
| 硬件发现 | 枚举串口 | 读 config 声明的 virtual bus 列表 |
| rx/tx queue | 真实串口字节流 | MVP 留空 |
| meta queue | 真实元数据 | 合成元数据，`bus_serial=sim://bus0` |
| inference queue | 真实舵机读取 | 从 sim 收语义 state + 本地打包寄存器字节 |
| 扭矩/温度/电流 | 读硬件寄存器 | sim 求解器的 qfrc_actuator + 常量 (温度 25°C / 电压 12V) |
| 校准 | 每次启动校准 | 从 config 读 preset，跳过校准 |

**估计代码量**：~700 行（含测试）。

### 6.2 Station `sim_manager` 模块

**位置**：`software/station/bin/station/src/sim_manager.rs`，新模块 ~150 行。

**职责**：
1. 解析 `sim-server` config section
2. Internal 模式：`tokio::process::Command::spawn` 拉起 `python3 -m norma_sim`，等 sentinel 文件 `/tmp/norma-sim-<pid>.ready`（max 5s，timeout 即 fail-fast）
3. External 模式：仅等待 socket 文件出现
4. 运行期：`tokio::spawn` 一个 wait task 监控子进程退出码，异常退出 → 通知 `st3215-sim` driver 进入 Crashed 状态
5. Station shutdown：SIGTERM → 等 2s → SIGKILL；清理 socket + sentinel 文件

**Main.rs 增量**（~40 行）：

实际的 `main.rs` 是一个扁平的 `#[tokio::main] async fn main()`（`software/station/bin/station/src/main.rs:329`），没有 `run()` 方法封装。在里面注入 sim_manager 的位置大致是：

```rust
#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    // ... log init ...

    let mut station = Station::new(&args).await?;
    station.start_main_queue().await?;
    inference::Inference::start_queue(&station.normfs).await?;
    station.start_commands_queue().await?;
    // ... start inference ...

    // ★ 新增: 在 start_drivers 之前拉起 sim-server（如果 config 有）
    let sim_manager = sim_manager::start_if_configured(&station.config).await?;

    station.start_drivers().await?;  // 现有, 内部新增 st3215-sim 分支

    // ... 现有的 tcp/web server 启动 + ctrl_c 等待不变 ...

    // ★ 新增: shutdown 路径, 先 st3215-sim 断连, 再 sim_manager 清进程
    station.shutdown().await?;
    if let Some(sm) = sim_manager {
        sm.shutdown().await;
    }
    Ok(())
}
```

**关键说明**：
- `main.rs` 的 tcp server / web server / 信号等待逻辑是 inline 写在 `main()` 里的（`main.rs:356-420`），不是封装在 `Station::run()` 里。`sim_manager` 的启动和关闭也以 inline 形式注入这两个位置
- `Station::start_drivers()`（`main.rs:197`）内部需要加 `st3215-sim` 分支，签名与现有 `st3215` 分支对称
- `Station::shutdown()`（现有方法）可能需要扩展一个返回 "driver shutdown 完成" 的信号，给 `sim_manager.shutdown()` 做顺序依赖
- **`Station::config` 字段当前是私有的**（`main.rs:65` 没有 `pub`）。要让 `sim_manager::start_if_configured(&station.config)` 编译通过，有两种改法：(a) 将 `config` 字段改为 `pub`；(b) 加一个 `impl Station { pub fn config(&self) -> &Config }` 访问器。本 spec 选 **(a) 最小改动**，仅把 `config: Config` 前加 `pub`（不暴露任何新方法）。或等价地，在 `main()` 里 inline 访问 `station.config`——这个字段本来就在同 crate 同 module 可见，只是要显式在 `Station` struct 声明前加 `pub`

**`tokio::process::Command` 配置**：新建子进程时必须 `.kill_on_drop(true)`，避免 Station panic 时留下孤儿 sim-server 进程（Linux/macOS 都需要）。

**互斥检查**：**`Config` struct 在 `software/station/shared/station-iface/src/config.rs`**（不是 `bin/station/src/config.rs`）。当前 `Config` 没有 `validate()` 方法，只做 serde 反序列化。方案：
1. 在 `station-iface/src/config.rs` 的 `Drivers` struct 上加一个 `pub fn validate(&self) -> Result<(), ConfigError>` 方法
2. `main.rs` 在 `Station::new()` 加载配置后立即调用 `config.drivers.validate()?`
3. `validate()` 的逻辑：如果 `self.st3215.is_some() && self.st3215.as_ref().unwrap().enabled && self.st3215_sim.as_ref().map_or(false, |s| s.enabled)` 则返回 `ConfigError::MutuallyExclusive`

这比改 `Station::new` 侵入性大，但是对的地方 —— config 校验属于 config 本身的责任。

### 6.3 Python 包 `norma_sim`（sim-server）

**位置**：`software/sim-server/`（新顶级目录，与 `drivers/` 和 `station/` 平级）。

**打包**：独立 `pyproject.toml`，**distribution 名 `norma-sim`（连字符，PEP 508 标准），Python import 名 `norma_sim`（下划线）**。两者差异：`pip install -e software/sim-server/` 用的是 distribution 名；Python import / `python -m` 用的是 module 名。README 必须明确说明，避免混淆。不发布 PyPI，本地 editable 安装。

**依赖**：
```toml
[project]
requires-python = ">=3.11"
dependencies = [
    "mujoco>=3.2,<4",
    "numpy>=1.26",
]
[project.optional-dependencies]
dev = ["pytest>=8", "pytest-asyncio>=0.23", "ruff", "mypy"]
```

**故意缺席**：mujoco-python-viewer、stable-baselines3、gymnasium、dm-control、torch、opencv、imageio（分别属于后续里程碑）。

**进程结构**：单进程，混合 async + 一个物理线程：

```
┌────────────────────────────────────────┐
│ asyncio main loop                       │
│  ├─ unix socket accept loop             │
│  ├─ ClientSession × N (handshake +      │
│  │   reader loop + writer loop)         │
│  └─ 广播队列 (asyncio.Queue)            │
│                                          │
│ physics_thread (threading.Thread)        │
│  while not stopping:                     │
│    t0 = monotonic()                      │
│    drain_pending_commands()              │
│    mujoco.mj_step(model, data)           │
│    snapshot = read_bus_state()           │
│    broadcast_to_clients(snapshot)        │
│    sleep_until(t0 + 1/state_rate_hz)     │
└────────────────────────────────────────┘
```

**关键类**：
- `World`（`world.py`）—— 加载 MJCF、joint/actuator 名字→id 映射、保护 `data` 的 threading.Lock、`step()` / `set_servo_target()` / `read_bus_state()` 接口
- `BusRegistry`（`buses.py`）—— 把 MJCF joint 映射成虚拟 ST3215 bus 的 motor id
- `ClientSession`（`ipc/clients.py`）—— 每连接一个 task，handshake → reader + writer 两个子协程
- `framing` 模块（`ipc/framing.py`）—— length-prefixed（u32 big-endian + payload）
- CLI（`cli.py`）—— argparse 入口，参数：`--mjcf`、`--socket`、`--state-rate-hz`、`--timestep`、`--log-level`

**启动序列**：
1. 加载日志配置（JSON 结构化）
2. 解析 CLI
3. 加载 MJCF（失败 → exit 1 + 明确错误）
4. 初始化 BusRegistry
5. 启动 physics_thread（daemon=True）
6. Bind unix socket（`.lock` 文件加 flock 检测 stale）
7. 写 sentinel 文件 `/tmp/norma-sim-<pid>.ready`
8. `asyncio.run(server.main())`

**停机**：SIGTERM → 停接新连接 → 现有 session 发 Goodbye → physics_thread 收 stop event 退出 → 关 socket + 删 sentinel → exit 0。

**估计代码量**：~1000 行（含测试）。

### 6.4 Sim IPC 协议

**Proto 文件**：`protobufs/sim/sim.proto`（新建目录 + 文件，与 `protobufs/drivers/` 和 `protobufs/station/` 平级）。

**Schema 要点**：
```proto
syntax = "proto3";
package norma_sim;

message Envelope {
  oneof payload {
    Hello hello = 1;
    Welcome welcome = 2;
    MotorCommandBatch motor_commands = 3;
    BusStateUpdate bus_state = 4;
    Goodbye goodbye = 5;
    Error error = 6;
  }
  uint64 monotonic_stamp_ns = 100;
}

message Hello {
  uint32 protocol_version = 1;
  string client_role = 2;       // "st3215-sim" / "usbvideo-sim" / ...
  string client_id = 3;
  repeated string subscribe_buses = 10;
}

message Welcome {
  uint32 protocol_version = 1;
  SimInfo sim_info = 2;
  repeated BusInfo confirmed_buses = 10;
}

message SimInfo {
  string model_name = 1;
  double timestep_sec = 2;
  double state_rate_hz = 3;
  string sim_version = 4;
  uint64 sim_start_time_ns = 5;
}

message BusInfo {
  string bus_serial = 1;
  uint32 motor_count = 2;
  repeated JointBinding joints = 3;
}

message JointBinding {
  uint32 motor_id = 1;
  string joint_name = 2;
  double range_min_rad = 3;
  double range_max_rad = 4;
}

message MotorCommandBatch {
  string bus_serial = 1;
  repeated MotorCommand commands = 2;
}

message MotorCommand {
  uint32 motor_id = 1;
  oneof action {
    SetPosition set_position = 10;
    SetVelocity set_velocity = 11;   // MVP 不实现，留 slot
    TorqueEnable torque_enable = 12;
    Reset reset = 13;
  }
}

message SetPosition {
  double target_rad = 1;
  double max_speed_rad_s = 2;
}

message BusStateUpdate {
  string bus_serial = 1;
  uint64 sim_time_ns = 2;
  repeated JointState joints = 10;
}

message JointState {
  uint32 motor_id = 1;
  double position_rad = 2;
  double velocity_rad_s = 3;
  double torque_nm = 4;
  double temperature_c = 5;       // MVP 固定 25.0
  double voltage_v = 6;           // MVP 固定 12.0
  bool torque_enabled = 7;
  bool moving = 8;
}

message Goodbye { string reason = 1; }

message Error {
  enum Code {
    E_UNSPECIFIED = 0;
    E_PROTOCOL_VERSION = 1;
    E_UNKNOWN_BUS = 2;
    E_UNKNOWN_JOINT = 3;
    E_INVALID_COMMAND = 4;
    E_SIM_INTERNAL = 5;
    E_BACKPRESSURE = 6;
  }
  Code code = 1;
  string message = 2;
}
```

**代码生成**：复用 `make protobuf` 管道：
- Rust：`st3215-sim/build.rs` 用 prost_build 生成到 `src/proto/sim.rs`
- Python：gremlin_py 生成到 `target/gen_python/protobuf/sim/sim.py`
- Go：gremlin_go 生成到 `target/generated-sources/protobuf/sim/`（MVP 不用，但管道一致）

**Framing**：`u32 big-endian length + payload bytes`。最大 1 MiB/帧。Rust 用 `tokio::codec::LengthDelimitedCodec`，Python 用 `asyncio.StreamReader.readexactly`。

**Handshake 时序**：
```
client → server  : Envelope{Hello{version=1, role, subscribe_buses=[...]}}
server → client  : Envelope{Welcome{version=1, sim_info, confirmed_buses}}
   [开始双向 push]
client → server  : Envelope{MotorCommandBatch{...}}     (任意时刻)
server → client  : Envelope{BusStateUpdate{...}}        (固定 rate, 100Hz)
...
client → server  : Envelope{Goodbye{reason}}
server → client  : Envelope{Goodbye{"bye"}}
   [close]
```

**Sync 模式**：
- State：server 按固定 `state_rate_hz` push（MVP 默认 100 Hz），不依赖 client 请求
- Commands：client 任意时刻 push，sim-server 在下一个 physics tick drain 全部 pending commands
- **无 request/response**，无 ack，无 ping/pong

**Backpressure**：双向 bounded queue（256 条），溢出策略：
- Client → Server（commands）：丢最老的，log warn，sim 回 `Error{E_BACKPRESSURE}`
- Server → Client（state）：丢最老的，**不报错**（state 是 periodic，下一次就到了）

**Transport**：Unix domain socket，默认路径 `/tmp/norma-sim-<station_pid>.sock`，权限 0600，用同路径的 `.lock` 文件做 flock stale 检测。

### 6.5 语义层 vs 协议层的职责划分

**关键技术决定**：Python 发 SI 单位语义 state，Rust 本地打包 ST3215 寄存器字节。

**原因**：`protobufs/drivers/st3215/st3215.proto:104` 的 `MotorState.state` 字段是 `bytes`，存的是原始 ST3215 EEPROM+RAM 寄存器 dump。`station_py/example_commands.py:42` 证实下游客户端按硬件寄存器地址（`PRESENT_POSITION_ADDR = 0x38`）直接 parse。这是项目刻意的设计：Station 透传硬件寄存器给客户端最大灵活性。

**若方案 1 = Python 直接生成 ST3215 字节**：ST3215 协议知识会分散到 Rust 和 Python 两处维护。

**方案 2（选）= Python 发 `JointState{position_rad, velocity_rad_s, torque_nm, ...}` 语义字段，Rust 本地调用一个集中的 pack 函数**。具体实现位置的决定见下一节。

### 6.6 `st3215::protocol::pack` 模块（P0 契约）

**位置决定**：pack 函数放在 **`st3215` crate** 的 `protocol` 模块下（新建 `software/drivers/st3215/src/protocol/pack.rs`），而**不是**在 `st3215-sim` 内部。理由：
- 真 driver 的 `port.rs` 里已经有寄存器字节的**解析**逻辑（`scan_motors` 读 EEPROM+RAM 后 parse 成语义字段）
- encoder 和 decoder 都是同一份"寄存器布局知识"的两个方向，分开维护容易漂移
- `st3215` crate 的职责本来就是"ST3215 协议实现"，加一个 pack 函数既符合职责又可被 sim 和自身测试共用
- `st3215-sim` 作为 `st3215` 的使用者，调 `st3215::protocol::pack::pack_state_bytes(...)` 即可，不触碰协议细节

**参数签名**：
```rust
/// 把语义层的关节状态打包成与真 ST3215 二进制兼容的 EEPROM+RAM dump.
///
/// 输出长度 = EEPROM (40 bytes, 0x00..0x28) + RAM (31 bytes, 0x28..0x47) = 71 bytes.
/// 格式与 `port.rs::scan_motors` 读取到的格式一致，下游 decoder 无需修改。
pub fn pack_state_bytes(
    motor_id: u8,
    preset: &MotorPreset,           // **新增** struct, 见下
    state: &SimMotorState,           // **新增** struct, 见下
) -> bytes::Bytes;

/// **新增**: 描述一颗 ST3215 舵机的静态属性, 由 st3215-sim config 根据
/// preset 名字 (如 "elrobot-follower") 查表构造, 在 Station 启动时传给
/// pack_state_bytes. 必须在 pack.rs 里定义 (现有 presets.rs 只有散装
/// const 和 PidConfig, 不包含这些字段).
pub struct MotorPreset {
    pub model_number: u16,           // ST3215 = 777
    pub firmware_version: u8,        // 常量 10
    pub baud_rate: u8,               // ST3215 baud code
    pub min_angle_steps: u16,        // 机械范围下限 (0..4095)
    pub max_angle_steps: u16,        // 机械范围上限
    pub offset: i16,                 // 零点偏移
    pub torque_limit: u16,           // 复用 presets::DEFAULT_TORQUE_LIMIT = 500
    pub voltage_nominal_v: f32,      // follower=12.0, leader=7.4
}

/// **新增**: 从 sim-server 收到的语义关节状态.
pub struct SimMotorState {
    pub position_rad: f32,           // 当前关节位置 (弧度)
    pub velocity_rad_s: f32,         // 当前角速度
    pub load_nm: f32,                // 当前负载 / 扭矩 (牛米)
    pub temperature_c: f32,          // 温度 (MVP 填常量 25.0)
    pub torque_enabled: bool,        // 是否使能扭矩
    pub moving: bool,                // 是否朝 goal 移动中
    pub goal_position_rad: f32,      // 上一次收到的 SetPosition target
    pub goal_speed_rad_s: f32,       // 0 = 默认
}

/// **新增**: 内置的 preset 工厂, 供 st3215-sim config 按名查表.
pub fn preset_by_name(name: &str) -> Option<MotorPreset> {
    match name {
        "elrobot-follower" => Some(MotorPreset { /* ... 硬编码值 ... */ }),
        "elrobot-leader"   => Some(MotorPreset { /* ... */ }),
        _ => None,
    }
}
```

**寄存器填充契约**：

**EEPROM 段**（`0x00..0x28`, 40 字节，大部分**从 `preset` 抽**不从 sim state 动态来）：

| 地址 | 寄存器 | 大小 | Sim 填充来源 |
|---|---|---|---|
| 0x00 | ModelNumber | 2 | `preset.model_number` (ST3215 = 777) |
| 0x02 | FirmwareVersion | 1 | `preset.firmware_version` (常量 10) |
| 0x05 | ID | 1 | `motor_id` 入参 |
| 0x06 | BaudRate | 1 | `preset.baud_rate` |
| 0x07 | ReturnDelay | 1 | 常量 250 |
| 0x08 | ResponseStatus | 1 | 常量 0 |
| 0x09 | MinAngleLimit | 2 | `preset.min_angle_steps` |
| 0x0B | MaxAngleLimit | 2 | `preset.max_angle_steps` |
| 0x0D..0x27 | 其他 EEPROM 字段 | ... | 各字段的"厂商默认值"常量，定义在 `pack.rs` 内部 `DEFAULT_EEPROM` table |

**RAM 段**（`0x28..0x47`, 31 字节，**从 `state` 动态来**）：

| 地址 | 寄存器 | 大小 | Sim 填充来源 |
|---|---|---|---|
| 0x28 | TorqueEnable | 1 | `state.torque_enabled as u8` |
| 0x29 | Acc | 1 | 常量 0 |
| 0x2A | GoalPosition | 2 | `rad_to_steps(state.goal_position_rad, preset)` |
| 0x2C | GoalTime | 2 | 常量 0 |
| 0x2E | GoalSpeed | 2 | `rad_s_to_sign_mag_15bit(state.goal_speed_rad_s, preset)` |
| 0x30 | TorqueLimit | 2 | `preset.torque_limit` |
| 0x37 | Lock | 1 | 常量 0 |
| 0x38 | **PresentPosition** | 2 | `rad_to_steps(state.position_rad, preset)` |
| 0x3A | **PresentSpeed** | 2 | `rad_s_to_sign_mag_15bit(state.velocity_rad_s, preset)` |
| 0x3C | **PresentLoad** | 2 | `nm_to_load_sign_mag(state.load_nm, preset)` |
| 0x3E | PresentVoltage | 1 | `(state.voltage_v * 10.0) as u8` (ST3215 是 0.1V 单位) |
| 0x3F | PresentTemperature | 1 | `state.temperature_c as u8` |
| 0x40 | Status | 1 | 常量 0 (无错误) |
| 0x42 | Moving | 1 | `state.moving as u8` |
| 0x45 | PresentCurrent | 2 | 常量 0 (MVP 不模拟电流) |

**单位转换辅助函数**（复用已有的 `st3215::protocol::units` 模块，**反向调用**）：
- `rad_to_steps(rad: f32, preset) -> u16` —— 弧度 → ST3215 4096 步的位置值，跨越 2π 对应 4096 步，加上 `preset.offset` 偏移
- `rad_s_to_sign_mag_15bit(rad_s: f32, preset) -> u16` —— 角速度 → ST3215 sign-magnitude 15-bit（**符号位在第 15 位，幅值在 0..14 位**；不是二进制补码）
- `nm_to_load_sign_mag(nm: f32, preset) -> u16` —— 负载 → sign-magnitude（类似）

**已有 units 函数复用**：
- `protocol::units::normal_position(u16) -> u16` 已经处理了 sign-magnitude 到无符号的正向转换，pack 需要**反向**：弧度 → sign-magnitude bytes。将在 `protocol/pack.rs` 内新增 `steps_to_sign_mag()` 辅助函数（~10 行）

**字节序**：ST3215 是 **little-endian**（低字节在前），与 `port.rs` 当前解析一致。

**Preset 结构**：`MotorPreset` 是**本 spec 新增**的结构，不是复用 `st3215::presets` 的既有类型。现有 `software/drivers/st3215/src/presets.rs` 只有散装的 `pub const DEFAULT_*` 常量（`DEFAULT_TORQUE_LIMIT`、`DEFAULT_MAX_TORQUE` 等）和一个 `PidConfig` struct，**没有**整合型的 "motor preset" 结构。实现时的选择：
1. **推荐**：在 `protocol/pack.rs` 里定义 `MotorPreset`（与它的 consumer `pack_state_bytes` 同文件），硬编码值时引用 `presets::DEFAULT_TORQUE_LIMIT` 等既有常量
2. **备选**：扩展 `presets.rs` 加入 `MotorPreset` struct，如果未来真机也想用同样的 preset 抽象——但 MVP 保守选 1，避免把 sim 概念漏进 real driver 的 preset 结构

**大小**：约 **120 行**（原先给 `register_pack.rs` 估的 80 行偏低；refresh 到 120 行，其中 40 行是静态 EEPROM 默认值 table）。

**测试**（`protocol/pack_test.rs`，~6 个测试，P0）：
- `test_pack_roundtrip_via_port_parser` —— pack 出 71 字节 → 用 `port.rs` 同样的 parse 逻辑解回来 → assert position_rad 误差 < 2 mrad
- `test_pack_present_position_at_0x38` —— pack 一个已知 rad，抽 `bytes[0x38..0x3A]` assert 等于期望的 little-endian u16
- `test_pack_sign_magnitude_negative_speed` —— 负速度的符号位
- `test_pack_torque_enable_boolean` —— `0x28` byte 是 0 或 1
- `test_pack_eeprom_preset_fields` —— `preset.model_number` 正确落在 `0x00..0x02`
- `test_pack_length_71_bytes` —— 输出长度恒定

---

## 7. URDF → MJCF 派生

### 7.1 源拓扑（`hardware/elrobot/simulation/elrobot_follower.urdf`）

```
base_link
  → (fixed) ST3215_1 → rev_motor_01 (revolute, ±1.5509 rad) → Joint_01
    → (fixed) ST3215_2 → rev_motor_02 → Joint_02
      → ... 串联到 ...
        → rev_motor_07 → Gripper_Base
          → (fixed) ST3215_8 → rev_motor_08 (revolute, 0~2.2028 rad) → Gripper_Gear
          → rev_motor_08_1 (prismatic, mimic rev_motor_08 × -0.0115) → Gripper_Jaw_02
          → rev_motor_08_2 (prismatic, mimic rev_motor_08 × +0.0115) → Gripper_Jaw_01
```

- 8 个 revolute 主关节（对应 8 个舵机 ID）
- 2 个 prismatic mimic 关节（靠机械齿轮联动，URDF 用 `<mimic multiplier="±0.0115">`）
- `effort=2.94 Nm, velocity=4.71 rad/s`（ST3215 规格）

### 7.2 MuJoCo URDF 导入器的已知坑

| URDF 特性 | MuJoCo 支持 | 应对 |
|---|---|---|
| `<mimic>` | **❌ 完全忽略** | **手工 `<equality>` 约束（关键）** |
| `<transmission>` | ❌ 忽略 | 本项目未用 |
| `<actuator>` | N/A（URDF 无此概念）| **手工 `<actuator>` 段** |
| `<collision>` mesh | ✅ 但默认 contype 可能太粗 | 调 `contype`/`conaffinity` |
| `<inertial>` | ✅ | 直接用 |
| `<visual>` mesh | ✅ | 直接用 |
| 尺度 `scale="0.001"` | ✅ | 直接用 |

### 7.3 派生方式：脚本生成 + check in

**拒绝**：一次性转换 + 手工维护 MJCF（URDF 和 MJCF 会分叉，两份都要维护）。

**选**：`gen.py` 脚本从 URDF 读关节限制，追加 MJCF 扩展段（option / compiler / equality / actuator / default / worldbody），输出到 `elrobot_follower.xml`，**提交到 git**（可审查、可 diff）。

**目录布局**：
```
hardware/elrobot/simulation/
├── elrobot_follower.urdf        (已存在，不动)
├── assets/*.stl                  (已存在)
└── mjcf/                         (新目录)
    ├── elrobot_follower.xml      (gen.py 生成 + 手调，check in)
    ├── gen.py                    (派生脚本 ~150 行)
    └── README.md                 (何时 re-gen)
```

**Makefile**：`make regen-mjcf` 跑 `gen.py`。URDF 改动频率低，不进 CI 自动生成。

### 7.4 MJCF 扩展段的核心内容

**Option**：
```xml
<option timestep="0.002" iterations="50" solver="Newton"
        gravity="0 0 -9.81" integrator="RK4"/>
```

**Compiler**：
```xml
<compiler angle="radian" meshdir="../assets"
          autolimits="true" coordinate="local" discardvisual="false"/>
```

**Equality 约束（关键！补回 URDF mimic）**：
```xml
<equality>
  <joint joint1="rev_motor_08_1" joint2="rev_motor_08"
         polycoef="0 -0.0115 0 0 0"/>
  <joint joint1="rev_motor_08_2" joint2="rev_motor_08"
         polycoef="0 0.0115 0 0 0"/>
</equality>
```

`polycoef="c0 c1 c2 c3 c4"` 的语义是 `joint1 = c0 + c1*joint2 + c2*joint2² + ...`，这里只用 `c1 = ±0.0115` 对应 URDF mimic multiplier。

**Actuator**（8 个 position actuator）：
```xml
<actuator>
  <!-- 以 motor_01 为模板示意 (ctrlrange 因关节而异) -->
  <position name="act_motor_01" joint="rev_motor_01"
            kp="15" kv="0.5" ctrlrange="-1.5509 1.5509"
            forcerange="-2.94 2.94"/>
  <!-- motor_02..07 每个 ctrlrange 不一样, 由 gen.py 从 URDF 的 <limit lower/upper> 自动抽取: -->
  <!--   rev_motor_02: ±1.6122    rev_motor_03: ±1.7610    rev_motor_04: ±1.7533       -->
  <!--   rev_motor_05: -3.1907~2.6998   rev_motor_06: -1.3775~1.7641                   -->
  <!--   rev_motor_07: -3.0710~2.7796                                                  -->
  <position name="act_motor_08" joint="rev_motor_08"
            kp="10" kv="0.3" ctrlrange="0 2.2028"
            forcerange="-2.94 2.94"/>
</actuator>
```

**gen.py 自动抽取每关节 range**：上例中 motor_01 的 `ctrlrange="-1.5509 1.5509"` 只是**其中一个关节的值**。实际 `gen.py` 会遍历 URDF 里所有 revolute joint 的 `<limit lower="..." upper="..."/>`，为每个 actuator 生成**独立的** `ctrlrange`。所以 MJCF 里每个 actuator 行都有自己的数值，不是同一个模板复制。`forcerange` 从 `<limit effort="..."/>` 抽（所有 ST3215 关节都是 2.94 Nm）。

**选择 `<position>` 而非 `<motor>`**：真 ST3215 内部是 position closed-loop，`<position>` 直接给 position target 最符合实际。`kp/kv` 是待调起点，`test_position_tracking_no_overshoot` 的 overshoot 约束（<10%）是验收判据。

**Default**：
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

**Worldbody**（顶层追加）：一个灯 + 一个地板，便于后续 MVP-2 的相机渲染有光照。

---

## 8. 启动 UX & 配置文件

### 8.1 两种模式

**Internal 模式**（默认，生产 dev loop 用）：
```bash
station -c station-sim.yaml
# ↓ 自动 spawn python3 -m norma_sim
# ↓ 等 sentinel 出现（max 5s）
# ↓ st3215-sim driver 连接 socket 握手
# ↓ 其他 driver 照常启动
# ↓ web UI 起来，浏览器访问 :8889
```

**External 模式**（sim 侧迭代用）：
```bash
# 终端 1
python -m norma_sim --mjcf ... --socket /tmp/norma-sim.sock

# 终端 2
station -c station-sim-external.yaml
```
sim-server 独立重启不影响 Station 编译，但 Station 的 `st3215-sim` driver 会进入 Crashed 状态，需手动重启 Station 才能重连（MVP 不做自动重连）。

### 8.2 配置文件

新建 `station-sim.yaml`（与 `station.yaml` 平级独立）：

```yaml
# station-sim.yaml
sim-server:
  enabled: true
  mode: internal                 # internal | external
  python-executable: python3
  module: norma_sim
  mjcf-path: hardware/elrobot/simulation/mjcf/elrobot_follower.xml
  startup-timeout-ms: 5000
  shutdown-timeout-ms: 2000
  log-capture: file              # file | inherit | null
  log-file: ./station_data/sim-server.log
  # Internal 模式下 socket-path 可省略, sim_manager 自动用
  # /tmp/norma-sim-<station_pid>.sock (PID 运行时注入, yaml 里无占位)
  state-rate-hz: 100
  timestep-sec: 0.002

drivers:
  st3215:
    enabled: false               # 互斥：真机关闭

  st3215-sim:
    enabled: true
    buses:
      - serial: "sim://bus0"
        motor-count: 8
        preset: elrobot-follower

  system-info: true
  usb-video:
    enabled: false               # MVP-1 关；MVP-2 换 usb-video-sim

inference:
- queue-id: inference/normvla
  shm: /tmp/normvla
  shm-size-mb: 12
  format: normvla
  st3215-bus: sim://bus0
  update-interval: 100ms
```

**Serde 映射说明**：`st3215` 和 `st3215-sim` 是 `station-iface::Config::Drivers` struct 的**兄弟字段**，都是 `Option<...>`。两者同时存在且都 `enabled=true` 时由 `Drivers::validate()` 拒绝（§6.2）。字段名 `st3215-sim` 映射到 Rust 字段 `st3215_sim` 通过 `#[serde(rename = "st3215-sim")]`，与现有 `#[serde(rename = "system-info")]` / `#[serde(rename = "usb-video")]` 的 pattern 一致。

**External 模式的 YAML**（`station-sim-external.yaml`，只列差异）：

```yaml
sim-server:
  enabled: true
  mode: external                 # ★ 改这一项
  socket-path: /tmp/norma-sim.sock  # ★ 必填, 因为没 station_pid 可用
  startup-timeout-ms: 5000        # 用于"等 socket 出现"的超时
  # internal 模式的参数 (python-executable, module, mjcf-path, ...) 都可以省略
  state-rate-hz: 100

drivers:
  # 同 station-sim.yaml
```

### 8.2.1 `SimServerConfig` Rust struct 草图

位于 `station-iface/src/config.rs`（在现有 `Drivers` struct 旁边）：

```rust
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SimServerConfig {
    pub enabled: bool,
    pub mode: SimMode,

    // Internal 模式参数 (external 模式下忽略)
    #[serde(rename = "python-executable", default = "default_python")]
    pub python_executable: String,

    #[serde(default = "default_module")]
    pub module: String,  // "norma_sim"

    #[serde(rename = "mjcf-path", skip_serializing_if = "Option::is_none")]
    pub mjcf_path: Option<PathBuf>,

    // 两模式共用
    #[serde(rename = "socket-path", skip_serializing_if = "Option::is_none")]
    pub socket_path: Option<PathBuf>,  // Internal 模式留空 → 自动用 /tmp/norma-sim-<pid>.sock

    #[serde(rename = "startup-timeout-ms", default = "default_startup_timeout")]
    pub startup_timeout_ms: u64,  // 默认 5000

    #[serde(rename = "shutdown-timeout-ms", default = "default_shutdown_timeout")]
    pub shutdown_timeout_ms: u64,  // 默认 2000

    #[serde(rename = "log-capture", default)]
    pub log_capture: LogCapture,  // file | inherit | null

    #[serde(rename = "log-file", skip_serializing_if = "Option::is_none")]
    pub log_file: Option<PathBuf>,

    #[serde(rename = "state-rate-hz", default = "default_state_rate")]
    pub state_rate_hz: f64,  // 默认 100.0

    #[serde(rename = "timestep-sec", default = "default_timestep")]
    pub timestep_sec: f64,  // 默认 0.002
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum SimMode { Internal, External }

#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Default)]
#[serde(rename_all = "lowercase")]
pub enum LogCapture {
    #[default]
    File,
    Inherit,
    Null,
}

/// St3215SimConfig 在 `Drivers` struct 下, 与现有 `St3215Config` 平级
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct St3215SimConfig {
    pub enabled: bool,
    pub buses: Vec<SimBus>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SimBus {
    pub serial: String,              // "sim://bus0"
    #[serde(rename = "motor-count")]
    pub motor_count: u32,            // 8
    pub preset: String,              // "elrobot-follower"
}

// Drivers struct 追加字段:
// #[derive(Debug, Serialize, Deserialize, Clone)]
// pub struct Drivers {
//     ... 现有字段 ...
//     #[serde(rename = "st3215-sim", skip_serializing_if = "Option::is_none")]
//     pub st3215_sim: Option<St3215SimConfig>,
// }
//
// impl Drivers {
//     pub fn validate(&self) -> Result<(), ConfigError> { ... 互斥检查 ... }
// }
```

**Socket 路径解析（`sim_manager` 的责任，运行时）**：
- `mode = internal` 且 config `socket_path` 为 `None` → 运行时设为 `PathBuf::from(format!("/tmp/norma-sim-{}.sock", std::process::id()))`
- `mode = internal` 且 config `socket_path` 有值 → 直接用该值（对高级用例）
- `mode = external` → config `socket_path` 必填，否则 validate 返回 `ConfigError::MissingExternalSocketPath`

这样 yaml 里**不出现 `${STATION_PID}` 占位符**，避免引入模板引擎。路径生成在 Rust 代码里完成，对用户而言就是"internal 模式不用填，external 模式必须填"。

### 8.3 日志路由（`log-capture` 字段）

| 值 | 行为 | 适合 |
|---|---|---|
| `file`（默认）| sim-server stdout/stderr 重定向到文件 | 生产 dev loop |
| `inherit` | sim-server 继承 Station 的 stdout/stderr | 快速调试 |
| `null` | `/dev/null` | 完全不要 sim 日志 |

Python 侧用 JSON 结构化日志，每行一个 JSON 对象，key 包含 `ts / level / role / msg / ...`。

### 8.4 Makefile 辅助

```makefile
.PHONY: sim-install sim-run sim-standalone sim-test regen-mjcf

sim-install:
	pip install -e software/sim-server/

sim-run: sim-install
	./target/debug/station -c software/station/bin/station/station-sim.yaml

sim-standalone: sim-install
	python -m norma_sim \
	  --mjcf hardware/elrobot/simulation/mjcf/elrobot_follower.xml \
	  --socket /tmp/norma-sim.sock

sim-test:
	cd software/sim-server && pytest

regen-mjcf:
	python hardware/elrobot/simulation/mjcf/gen.py
```

---

## 9. 错误处理 & 故障模式 & 可观测性

### 9.1 故障响应矩阵（核心原则：启动期快错，运行期软错）

**Internal 模式**：
| 时机 | 故障 | 响应 |
|---|---|---|
| **启动** | python 可执行找不到（L2）| **Station exit 1**，`ERROR: sim-server: python3 not found. Set sim-server.python-executable in config.` |
| **启动** | `norma_sim` module 缺失（L3）| **Station exit 1**，`ERROR: sim-server: python -m norma_sim failed: No module named 'norma_sim'. Run: make sim-install` |
| **启动** | mjcf 路径不存在（L3）| **Station exit 1**，sim-server 子进程 exit 1 被 sim_manager 捕获，报错有 mjcf 路径 |
| **启动** | Sentinel 5s 没出现（L2/L3）| **Station exit 1**，`ERROR: sim-server: startup timeout after 5000ms. Check log: ./station_data/sim-server.log` |
| **启动** | Socket 文件已存在且被另一个 sim-server 占用 | sim-server exit 1 `socket already in use`，Station exit 1 |
| **启动** | Config 互斥冲突 | Station exit 1（Drivers::validate 返回 MutuallyExclusive）|
| **启动** | TCP/web server 端口被占用 | 现有 Station 行为是 `panic!`（`main.rs:364, 379`）。**本 spec 要求修复**：先让 sim_manager 启动，然后 bind tcp/web，如果 bind 失败则 sim_manager.shutdown() 必须被调用清理子进程（否则 panic 会留下孤儿）。`tokio::process::Command.kill_on_drop(true)` 是兜底但不是主方案 |
| **启动** | `st3215-sim` driver 初始化（连 socket + handshake）失败 | Station exit 1，sim_manager 触发清理 |

**External 模式（独有的 race 场景）**：
| 时机 | 故障 | 响应 |
|---|---|---|
| **启动** | 配置里 `socket-path` 为空 | **Station exit 1**，`ConfigError::MissingExternalSocketPath` |
| **启动** | 连接外部 sim-server 失败（对方未启动）| **Station exit 1**，`ERROR: st3215-sim: failed to connect to <path>: connection refused. Is sim-server running?` |
| **启动** | 连接成功但 handshake 超时或版本不匹配 | Station exit 1，错误携带服务端报的 `E_PROTOCOL_VERSION` code |
| **启动** | 外部 sim-server 有 stale socket 文件但进程死了 | External 模式下**不清理**外部 socket（Station 无权限管别人的进程）。连 socket 会立即失败 → 同上 connection refused 路径 |

**运行期（两种模式共用）**：
| 时机 | 故障 | 响应 |
|---|---|---|
| **运行** | sim-server 子进程崩溃（exit ≠ 0）| 软错：driver 标记 unhealthy，log ERROR，**Station 继续运行** |
| **运行** | IPC socket 中途断开 | 软错：driver 进入 Crashed 状态，web UI 显示"总线离线" |
| **运行** | Protobuf decode 错 / framing 错 | 关连接 + driver 进入 Crashed |
| **运行** | 单命令不合法（未知 joint / out-of-range）| 局部失败：sim 回 `Error{E_INVALID_COMMAND}`，driver log warn，继续接收其他命令 |
| **运行** | Backpressure overflow | 丢最老的，log warn |
| **运行** | Physics NaN 爆炸 | Python 检测 `numpy.isnan(data.qpos)` → log CRITICAL → exit 1 → Station 走子进程崩溃路径 |
| **运行** | Python 单 session 异常 | Session 隔离关闭，sim-server 本身继续 |
| **运行**（external）| 外部 sim-server 被手动 kill | Station 的 driver 看到 socket 断开 → 进入 Crashed。Station 本身不退出，其他 driver 继续。恢复方式：手动重启 Station |

### 9.2 明确 MVP 不做

- ❌ 自动重启 sim-server
- ❌ Driver 自动重连
- ❌ 健康检查 / heartbeat
- ❌ Circuit breaker
- ❌ Graceful degradation（回落 kinematic mock）

### 9.3 日志

**Station 侧**（`log` + `env_logger`）：事件级别 INFO/WARN/ERROR，由 `RUST_LOG` 控制详细度。

**Python sim-server 侧**：JSON 结构化，默认走 `./station_data/sim-server.log`，关键字段：
- `ts` / `level` / `role="sim-server"` / `msg`
- `sim_time`（MuJoCo data.time）+ `monotonic_stamp_ns`（wall clock），便于对齐
- `client_id`（哪个 driver 连过来）
- 所有 CRITICAL 带完整 traceback

**Debug 环境变量**：
- `RUST_LOG=st3215_sim=debug,sim_manager=debug`
- `NORMA_SIM_LOG_LEVEL=DEBUG`
- `NORMA_SIM_TRACE_IPC=1`（每个 Envelope 收发都 log，体积大）

### 9.4 诊断工具

`software/sim-server/scripts/inspect.py` —— 独立 Python 脚本，连到一个正在运行的 sim-server，订阅 BusState 并 print。用法：

```bash
python software/sim-server/scripts/inspect.py --socket /tmp/norma-sim.sock
```

这是"听筒"工具：当不确定是 driver 问题还是 sim 问题时，用它直接连 sim-server 旁听。sim 能发 driver 收不到 = driver 问题；sim 不发 = sim 问题。

### 9.5 确定性

**MVP 立场：非确定性**。Real-time 模式下 physics 线程节拍受 OS 调度影响。MJCF `option.timestep` 固定，但端到端非 reproducible。

**留的口子（MVP 不启用）**：Python sim-server 支持 `--deterministic --seed N` flag 进入 tick-driven mode（按 Station step 命令推进，不按 wall clock），作为 CLI arg 存在但 MVP-1 不实现。

---

## 10. 测试策略

### 10.1 哲学

**不追求覆盖率，追求关键风险消减**。P0 风险有专门测试，P1 基本测试，P2 靠手工 smoke。

**P0 风险**：
1. ST3215 寄存器字节格式错 → 下游全部解码失败
2. MJCF equality 约束写错 → 夹爪不动
3. 启动时 sim-server 起不来 → 用户第一次体验失败

### 10.2 测试分层

| 层 | 工具 | 范围 |
|---|---|---|
| 1. 静态检查 | `cargo clippy`、`ruff`、`mypy --strict` | 代码风格 + 类型 |
| 2. 单元测试 | `cargo test`、`pytest` | framing、codec、register pack、MJCF 加载、mimic 约束、physics 稳定性、handshake |
| 3. 集成测试 | `pytest` + 子进程 | 真起 sim-server，走完整 handshake → command → state 回环 |
| 4. 手工 smoke | README checklist | End-to-end 包括 Station + web UI |

### 10.3 Rust 单元测试

**`st3215` crate（新增，P0）** —— 在 `protocol/pack_test.rs` (`#[cfg(test)] mod`)：
- `test_pack_length_71_bytes` —— 输出长度恒定
- `test_pack_present_position_at_0x38` —— 已知弧度 → bytes[0x38..0x3A] 是期望的 little-endian u16
- `test_pack_sign_magnitude_negative_speed` —— 负速度的符号位在正确位置
- `test_pack_torque_enable_boolean` —— `bytes[0x28]` 是 0 或 1
- `test_pack_eeprom_preset_fields` —— `preset.model_number` 落在 `0x00..0x02`
- `test_pack_roundtrip_via_port_parser` ★ —— pack 出 71 字节 → 用 `port.rs::scan_motors` 同样的 parse 逻辑解回来 → assert position_rad 误差 < 2 mrad

**`st3215-sim` crate（新增，~4-5 个）** —— 内联 `#[cfg(test)] mod tests`：
- `test_framing_roundtrip` —— length-prefixed 帧读写
- `test_joint_state_translation_to_pack_call` —— driver 调 `st3215::protocol::pack::pack_state_bytes` 时参数映射正确
- `test_ipc_codec_roundtrip` —— Envelope encode/decode

**`station-iface` crate（新增，1 个）**：
- `test_drivers_validate_mutual_exclusion` —— `Drivers::validate()` 在 `st3215.enabled && st3215_sim.enabled` 时返回 `ConfigError::MutuallyExclusive`

### 10.4 Python 单元 + 集成测试（`norma_sim`，~12-15 个）

P0:
- `test_mjcf_loads_without_error`
- `test_all_8_actuators_present`
- **`test_mimic_gripper_equality_works`** ★ —— 驱动 `rev_motor_08` 到 1.0，跑 500 步 sim，assert `rev_motor_08_1 ≈ -0.0115` 和 `rev_motor_08_2 ≈ 0.0115`（容差 2 mm）

P1:
- `test_1000_steps_no_nan` —— 空世界静止
- `test_position_tracking_no_overshoot` —— 跟踪 target，overshoot < 10%
- `test_framing_roundtrip`
- `test_handshake_happy_path`
- `test_handshake_wrong_version`
- `test_handshake_unknown_bus` —— 订阅不存在的 bus → `Error{E_UNKNOWN_BUS}`

集成测试：
- `test_full_loop` —— 起 sim-server 子进程 → 连 socket → handshake → 发命令 → 观察 `position_rad` 朝目标移动（容差）
- `test_subprocess_clean_shutdown` —— SIGTERM 后 socket + sentinel 被清
- **`test_multi_client_fan_out`** ★★ —— 起 sim-server → **两个独立 ClientSession 并发订阅同一 bus** → 发一个命令 → 两者都收到 BusStateUpdate 广播。**为 MVP-2 `usbvideo-sim` 铺路**：证明 sim-server 的多 client fan-out 架构工作
- `test_external_sim_not_running` —— driver 尝试连一个不存在的 socket → 立即失败，错误消息含 "connection refused"

### 10.5 手工 Smoke Test

作为 `software/sim-server/README.md` 交付物。一个 dev 10 分钟走一遍的 checklist（详细列表见附录 A）。

### 10.6 MVP-1 验收标准（Definition of Done）

**功能性**：
- **F1** `station -c station-sim.yaml` 一条命令从干净仓库启动
- **F2** 浏览器 web UI 看到 ElRobot follower 3D 动态响应关节命令
- **F3** 7 个主臂关节在 MuJoCo 物理里正确驱动到目标
- **F4** ★ 第 8 关节（`rev_motor_08`）被驱动时，两个 prismatic mimic（`_08_1`、`_08_2`）通过 equality 约束正确联动
- **F5** `st3215-sim` 写出的 `st3215/inference` 下游零修改可解码

**鲁棒性**：
- **R1** 启动失败快错 + 明确错误 + 下一步建议
- **R2** 运行期崩溃软错，其他 driver 不受影响
- **R3** Ctrl+C 干净关闭（socket + sentinel 都清）
- **R4** config 互斥由 loader 强制

**质量**：
- **Q1** `cargo test -p st3215-sim` 全绿（≥8 测试）
- **Q2** `cd software/sim-server && pytest` 全绿（≥12 测试含集成）
- **Q3** `cargo clippy -p st3215-sim -- -D warnings` 无警告
- **Q4** 手工 smoke test checklist 全过

**判定权**：由项目 maintainer 手工跑 smoke test 签字确认。

---

## 11. Repo Layout 完整清单

### 11.1 MVP-1 结束时的新文件与改动

**新文件**（~35 个）：
```
protobufs/sim/sim.proto                                   [~100 行]

hardware/elrobot/simulation/mjcf/
├── elrobot_follower.xml                                  [~200 行 MJCF]
├── gen.py                                                [~150 行]
└── README.md                                             [~30 行]

software/sim-server/
├── pyproject.toml
├── README.md                                             [含 smoke checklist]
├── norma_sim/
│   ├── __init__.py
│   ├── __main__.py
│   ├── cli.py                                            [~80 行]
│   ├── server.py                                         [~200 行]
│   ├── world.py                                          [~150 行]
│   ├── buses.py                                          [~80 行]
│   ├── time_mode.py                                      [~40 行]
│   ├── logging_setup.py                                  [~30 行]
│   └── ipc/
│       ├── __init__.py
│       ├── framing.py                                    [~60 行]
│       ├── protocol.py                                   [~120 行]
│       └── clients.py                                    [~100 行]
├── tests/
│   ├── conftest.py
│   ├── test_world_loads.py
│   ├── test_mimic_gripper.py                             [★ P0]
│   ├── test_physics_stable.py
│   ├── test_framing.py
│   ├── test_handshake.py
│   └── integration/
│       ├── test_full_loop.py
│       └── test_subprocess_lifecycle.py
└── scripts/
    ├── dev_server.sh
    └── inspect.py

software/drivers/st3215/                                 [✏️ 改: 加 protocol::pack]
├── src/protocol/
│   ├── pack.rs                                           [✨ 新 ~120 行, P0]
│   └── mod.rs                                            [✏️ 改: pub use pack]
└── src/protocol/
    └── pack_test.rs                                      [✨ 新, P0 测试]

software/drivers/st3215-sim/
├── Cargo.toml                                            [deps: st3215]
├── build.rs                                              [prost_build]
└── src/
    ├── lib.rs                                            [~60 行]
    ├── config.rs                                         [~40 行]
    ├── ipc.rs                                            [~100 行]
    ├── codec.rs                                          [~50 行]
    ├── command_loop.rs                                   [~80 行]
    ├── state_loop.rs                                     [~60 行]
    ├── errors.rs                                         [~30 行]
    └── proto/sim.rs                                      [🤖 prost 生成]
    (测试用 #[cfg(test)] mod tests 内联在对应 .rs 文件中, 不单独建 _test.rs)

software/station/bin/station/
├── station-sim.yaml                                      [新配置]
├── station-sim-external.yaml                             [新配置]
└── src/sim_manager.rs                                    [新模块 ~150 行]

docs/superpowers/specs/
└── 2026-04-10-simulation-integration-design.md           [本文档]
```

**改动文件**（~7 个）：
- `Cargo.toml`（根）—— workspace members 追加 `st3215-sim`
- `Makefile` —— 追加 `sim-install` / `sim-run` / `sim-standalone` / `sim-test` / `regen-mjcf`
- `software/drivers/st3215/src/protocol/mod.rs` —— 导出新 `pack` 模块
- `software/station/bin/station/Cargo.toml` —— 追加 `st3215-sim` dep
- `software/station/bin/station/src/main.rs` —— 注入 `sim_manager` + `st3215-sim` 分支 ~40 行增量
- **`software/station/shared/station-iface/src/config.rs`**（不是 `bin/station/src/`）—— 加 `SimServerConfig` + `St3215SimConfig` + `Drivers::validate()` 方法
- `.gitignore` —— 追加 `/tmp/norma-sim*.sock`、`*.ready`、`sim-server.log`

**零修改**：web UI、所有现有 driver、station_py、gremlin、station-iface、所有 URDF/STL/CAD。

**代码量估算**：
- Rust：~700 行（含测试）
- Python：~1000 行（含测试）
- Proto：~100 行
- MJCF/config：~300 行
- **总计 ~2100 行新代码**

### 11.2 依赖与构建顺序

```
L0: hardware/.../mjcf/*.xml  (gen.py 派生)
L0: protobufs/sim/sim.proto
          ↓ make protobuf
L1: Rust sim proto            L1: Python sim proto
          ↓                          ↓
L2: st3215-sim crate          L2: norma_sim package
          ↓                          ↓
L3: station binary (deps st3215-sim)
          ↓
L4: 运行时: station → spawn norma_sim → driver 连 socket
```

### 11.3 首次启动命令序列

```bash
# 一次性准备
pip install -e software/sim-server/
make protobuf
make regen-mjcf                # 仅 URDF 变动时必要
cargo build -p station

# 运行
./target/debug/station -c software/station/bin/station/station-sim.yaml
```

---

## 12. 里程碑拆分

### 12.1 MVP-1 = 本 spec 的完整 scope

参见第 3.1 节 Goals 和第 10.6 节 DoD。

### 12.2 MVP-2（紧接里程碑，本 spec 仅勾勒）

**增量范围**：
- `software/drivers/usbvideo-sim/` 新 crate（~300 行）
- `software/sim-server/norma_sim/cameras.py`（~120 行，MuJoCo Renderer API）
- `protobufs/sim/sim.proto` 扩展 `CameraFrameUpdate` 消息
- MJCF 追加 `<camera>` 标签（挂在 Gripper_Base 和 world）
- Station `main.rs` 追加 `usbvideo-sim` 分支（~20 行）

**新问题要解决**：MuJoCo OpenGL context 初始化、渲染频率（30 FPS）与物理频率（500 Hz）解耦、帧编码对齐（MJPEG / raw RGB，与现有 `usbvideo` 输出格式一致）。

**DoD**：NormVLA 推理回路端到端跑通（关节 + 假图像）。

### 12.3 Future Work（不在本 spec 范围）

- **MVP-3**：桌面 + 可抓物体 + pick-and-place 场景
- **MVP-4**：Leader 臂 URDF（需硬件团队建模）+ 遥操作 sim 版本
- **MVP-5**：Deterministic tick-driven mode + CI 集成 + GitHub Actions
- **升级到场景 B**：数据生成管道、场景随机化、轨迹录制
- **升级到场景 C**：Sim-to-real 策略评估、domain randomization
- **升级到场景 F**：并行 sim、GPU 加速（可能换 Isaac Lab 或 Genesis）

每条都是独立的 brainstorming → spec → implementation cycle。

---

## 13. 风险 & 未解问题

### 13.1 技术风险

| 风险 | 影响 | 缓解 |
|---|---|---|
| `<equality>` polycoef 在 MuJoCo 里数值不稳 | 夹爪联动不收敛，MVP-1 核心 demo 失败 | `test_mimic_gripper_equality_works` P0 测试是早期验证点；**备选方案**：改用 `<tendon><fixed>` —— 如果 equality 失败，`gen.py` 改用 `<tendon name="grip_link" limited="false"><fixed><joint joint="rev_motor_08" coef="1"/><joint joint="rev_motor_08_1" coef="-0.0115"/></fixed></tendon>` + 对应第二条 `_08_2` tendon，tendon 更灵活但 equality 更简洁 |
| ST3215 寄存器字节打包 bug（字段偏移/字节序错误）| 下游 web UI 显示错乱，不能立即识别 | `pack_test.rs::test_pack_roundtrip_via_port_parser` 用 `st3215::port.rs` 的既有 decoder 反验 |
| MuJoCo `position` actuator 的 kp/kv 难调 | 关节抖动或慢响应，观感不好 | MVP 起点 kp=15/kv=0.5，`test_position_tracking_no_overshoot` 作约束，可迭代 |
| Rust prost 和 gremlin_py 生成出的 proto 二进制不兼容 | 跨语言 IPC decode 错 | 两边都基于同一个 `.proto` 源文件；集成测试覆盖 |
| Unix socket 权限 / stale lockfile 导致启动失败 | 首次 UX 受损 | 启动时明确检测 + 清理，有明确错误消息 |
| MuJoCo wheel 在 ARM64（Raspberry Pi）上的可用性 | 无法在项目 SBC target 跑 sim | MVP 只 target x86_64 dev 机器；ARM64 留给后续验证 |
| MVP-2 加 `usbvideo-sim` 时需要**多 client 共享同一 sim-server** | 若 MVP-1 没验证 fan-out，MVP-2 会被迫重构 sim-server 架构 | MVP-1 必须在 Python 集成测试里加 `test_multi_client`，证明两个 ClientSession 并发订阅同一 bus 都能收到广播（§10.4 已纳入）|

### 13.2 未决的小问题

1. **`state_rate_hz=100` 是否够快**：web UI 是 WebSocket push，看实际观感。MVP 起点 100 Hz，可调。
2. **Python sim-server 是否需要单独的 venv 管理**：`make sim-install` 直接 `pip install -e`，假设在当前 Python 环境。dev 可能想用 venv 隔离。MVP 保守：不强制 venv，README 建议用。
3. **`norma-sim` 包名与 `norma_sim` 模块名的差异**：`pyproject.toml` 的 `[project].name` 是 `norma-sim`（PEP 508 规范推荐连字符），而 Python import 是 `norma_sim`（下划线）。README 必须明确说明 `pip install -e` 和 `python -m` 用的名字不一样。

### 13.3 不是风险的东西

- **Rust proto codegen 方式**：已确认是 **prost**（`software/drivers/st3215/build.rs` 使用 `prost_build::Config::new()`）。新 `st3215-sim/build.rs` 照抄即可
- **Python 依赖冲突**：`norma_sim` 只依赖 `mujoco` 和 `numpy`，与现有 `gremlin_py`、`station_py` 无冲突
- **Station 二进制大小**：`st3215-sim` 是普通 Rust crate，增加编译大小 <1 MB
- **构建时间**：新增 crate 编译时间 <30 秒

---

## 14. 已由 review 解决或关闭的开放问题

本节记录第一轮 spec review 提出的 10 个 issue + 10 个 recommendation 的处置情况，避免未来再讨论。

**已修复的事实错误**：
- ✅ **函数签名**（§6.1）：改为 `<T: StationEngine>(normfs, station_engine, ...)`，与 `st3215::start_st3215_driver` 真实签名一致（`driver.rs:293`）
- ✅ **Config 位置**（§6.2, §11.1）：修正为 `station-iface/src/config.rs`（不是 `bin/station/src/config.rs`）；新增 `Drivers::validate()` 方法，不依赖不存在的方法
- ✅ **main.rs 结构**（§6.2）：修正为 flat `#[tokio::main] async fn main()` 的真实结构；增量代码注入 `main()` 的几个关键位置而非虚构的 `Station::run()`
- ✅ **RamRegister 模块路径**（§6.5/6.6）：改为 `st3215::protocol::RamRegister`（`protocol/mod.rs:9` `pub use memory::*`）
- ✅ **prost codegen 方式**（§13.3）：确认是 prost，不再是开放问题
- ✅ **`register_pack` 选址**（§6.1, §6.6）：从 `st3215-sim` 内部迁移到 `st3215::protocol::pack` 公共模块，与 decoder 同处，避免协议知识分散

**已明确的架构取舍**：
- ✅ **Architecture 选 C**（§4）：混合架构是设计决定，不是开放问题，Rust in-process FFI 已经在 §2.3 的 Rejected Alternatives 里明确拒绝
- ✅ **`station-sim.yaml` 独立文件**（§4, §8.2）：设计决定，不再是开放问题
- ✅ **`software/drivers/st3215-sim/` vs `software/sim/`**（§6.1）：设计决定，`st3215-sim` 是 driver，放在 drivers 目录
- ✅ **Multi-client fan-out in MVP-1**（§10.4, §13.1）：MVP-1 必须有 `test_multi_client` 集成测试证明多 client 订阅同一 bus 能正常 fan-out，为 MVP-2 usbvideo-sim 加入铺路

**已明确的 API / 契约细节**：
- ✅ **`register_pack` 字段契约**（§6.6）：71 字节 EEPROM+RAM dump，表格形式列出每个寄存器的填充来源和单位转换
- ✅ **`SimServerConfig` struct 草图**（§8.2.1）：给出 Rust struct 签名、serde rename、默认值函数、SimMode / LogCapture / St3215SimConfig / SimBus 完整结构
- ✅ **Socket 路径解析**（§8.2.1）：Internal 模式运行时注入 PID，yaml 里不出现占位符；External 模式必须在 yaml 里显式配置，由 validate 强制
- ✅ **Failure matrix 的 external mode race**（§9.1）：分成 Internal / External / 运行期共用三部分，明确 connection refused / handshake 超时 / 外部 sim 被 kill 的语义
- ✅ **`kill_on_drop(true)`**（§6.2）：sim_manager spawn subprocess 时强制打开，避免孤儿进程
- ✅ **Drivers serde 兄弟字段**（§8.2）：`st3215` 和 `st3215-sim` 都是 `Option<T>`，同时 enabled 由 validate 拒绝
- ✅ **测试文件约定**（§11.1）：Rust 测试用 `#[cfg(test)] mod tests` 内联，不单独建 `_test.rs` 文件
- ✅ **URDF 每关节 range 由 gen.py 从 URDF 读**（§7.4）：MJCF 示例中的 `ctrlrange` 只是 motor_01 的，gen.py 会为每个关节单独生成

---

## 附录 A：MVP-1 手工 Smoke Test Checklist

放入 `software/sim-server/README.md`。

### Prereqs
- [ ] Python 3.11+ installed
- [ ] `make sim-install` 跑过
- [ ] `make protobuf` 跑过
- [ ] `cargo build -p station` 成功
- [ ] `hardware/elrobot/simulation/mjcf/elrobot_follower.xml` 存在

### Internal 模式
- [ ] `./target/debug/station -c station-sim.yaml --web 0.0.0.0:8889`
- [ ] stderr 看到 `sim-server ready in X.XXs` (<3s)
- [ ] stderr 看到 `handshake complete, confirmed 1 bus, 8 joints`
- [ ] 浏览器打开 http://localhost:8889
- [ ] ElRobot 3D 模型加载
- [ ] 左侧显示 `sim://bus0`
- [ ] 8 个 motor online，position ~0
- [ ] 拖 motor 1 滑条 → 3D 视图 Joint_01 转动
- [ ] **拖 motor 8 (夹爪) → 手指开合** ★ (equality 约束验证)
- [ ] Ctrl+C Station → 干净退出
- [ ] `ls /tmp/norma-sim*` 为空

### External 模式
- [ ] 终端 1：`python -m norma_sim --mjcf ... --socket /tmp/norma-sim.sock`
- [ ] 终端 1 看到 `socket listening` 和 `sentinel written`
- [ ] 终端 2：`./target/debug/station -c station-sim-external.yaml`
- [ ] 终端 2 看到 `handshake complete`
- [ ] 浏览器操作同 Internal
- [ ] Ctrl+C 终端 1 → 终端 2 看到 `driver entered Crashed state`
- [ ] web UI 显示总线离线
- [ ] Station 不崩，sysinfod 继续
- [ ] Ctrl+C 终端 2 → 干净退出

### 故障场景
- [ ] mjcf-path 指向不存在文件 → `ERROR: mjcf file not found`
- [ ] python-executable 设 /nonexistent → `ERROR: python not found`
- [ ] st3215 和 st3215-sim 同时启用 → `ERROR: mutually exclusive`

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
| **Follower / Leader** | Follower 是执行臂，Leader 是遥操作臂（低减速比舵机利于反向驱动）|
| **MJCF** | MuJoCo 的原生场景描述格式（XML），优于 URDF 的功能集 |
| **Equality constraint** | MJCF 里用 `<equality>` 标签定义的约束，替代 URDF `<mimic>` |
| **Sim-server** | 本 spec 引入的 Python 进程（`norma_sim` 包），跑 MuJoCo 物理 |
| **Shim driver** | 本 spec 引入的 Rust driver（`st3215-sim`），连接 sim-server 并做协议翻译 |

---

**本文档结束**

下一步：spec review → writing-plans → 实现。
