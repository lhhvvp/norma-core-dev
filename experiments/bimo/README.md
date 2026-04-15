# Bimo 孵化实验

基于 [the-bimo-project](https://github.com/mekion/the-bimo-project) 研究
**端到端可工作的 sim-to-real 方案**，目标是为 NormaCore 的
`SimToRealAdapter` + pi0 训练流程提炼可复用的方法论与具体技术。

## 动机

Week 1 pipeline 验证完成（pi0 训练雏形 + `norma_sim/sim_to_real.py` 初版），
但真正的 sim-to-real 还没跨过去。Bimo 是一个规模小、完整、经过 hardware
验证的参考样本 —— 双足行走和机械臂操作在动力学上完全不同,但"如何让
仿真训练的策略在真机上直接跑"这一套方法可以迁移。

- Isaac Lab 训练到 ONNX 部署的**完整链路**全部开源，文档声明 walking
  policy 已成功从 sim 直接迁移到真机（无任何 hardware 适配）。
- 域随机化 recipe 完整：backlash、actuator delay、link mass、foot material、
  torque cycling、body push 等全部在 `bimo_task_env.py` 里编码。
- BimoAPI 只有 ~500 行 Python —— 足够小，可以在一个下午读完。

这让它成为一份**学习样本**，而不是要移植的框架。

## 参考项目核心架构

```
the-bimo-project/
├── BimoAPI/                 ← Python 控制库 + 部署脚本
│   └── bimo/
│       ├── bimo.py          ← 核心控制类（MCU serial + cameras）
│       ├── routines.py      ← 时间编排的动作序列（sit/stand）
│       └── examples/
│           └── api_example.py  ← 完整 ONNX 推理循环
├── IsaacLab/                ← 训练环境
│   └── bimo/
│       ├── bimo_task_env.py  ← DirectRLEnv，全部 DR 与 reward
│       ├── bimo_config.py    ← Articulation / DC Motor 配置
│       ├── agents/rsl_rl.py  ← PPO 超参数
│       └── assets/Bimo.usd   ← USD 机器人模型（二进制）
└── MCU/
    └── micro_bimo.ino       ← RP2040 固件（binary serial 协议）
```

| 特性 | 说明 |
|------|------|
| 训练框架 | Isaac Lab 2.3.0 + RSL-RL PPO |
| 观测空间 | 44 dim = 4 步 orient history × 3 + 4 步 action history × 8 |
| 动作空间 | 8 dim 关节角度增量，`clip(-3, 3) × 4/3` |
| 控制频率 | 20 Hz（50 ms 周期） |
| 训练时长 | 2048 并行环境，<15 分钟（作者宣称） |
| 部署依赖 | `pyserial` + `numpy` + `onnxruntime` + `opencv-python` |
| 协议 | 921600 baud binary serial，MCU 侧做硬限位裁剪 |

## 与 NormaCore 的关系

NormaCore 已有 `software/sim-server/norma_sim/sim_to_real.py`（Config:
joint/gripper noise、action_delay、calibration_offset、camera_latency、
obs_drop）。Bimo 提供的增量价值：

| Bimo 有 / NormaCore 没有 | 说明 |
|---|---|
| **Backlash simulation** | 2.4° 反向死区，动作反转时先吃掉死区量再移动 |
| **Link mass randomization** | 每次 reset 全链路刚体质量 ±5%（scale 模式） |
| **Material friction randomization** | 足底接触材料每次 reset 换一组参数 |
| **Torque cycling per reset** | 2.7-2.94 Nm 以 0.01 步长轮换，模拟电压波动 |
| **Periodic body pushes** | 训练期 2-4 秒随机推一次 head（外扰扰动） |

| 两边都有 | 实现差异 |
|---|---|
| Actuator delay | Bimo: 0-1 物理步随机；NormaCore: 固定步数 deque |
| Action noise | Bimo: σ=0.5°；NormaCore: σ=0.01 rad |
| IMU / joint obs noise | Bimo: σ=0.015 rad；NormaCore: σ=0.02 rad |

| Bimo 架构设计亮点 | 对 NormaCore 的启示 |
|---|---|
| Obs 结构在训练 / 部署两侧**同一份定义** | 需要把 `SimToRealAdapter` obs 与 pi0 输入的契约锁死 |
| MCU 侧做硬限位，Python 只是建议 | 对应 NormaCore `capabilities.py` 硬件边界 |
| `act_hist` reset 用原始度数（外 [-1,1]） | episode 起点鲁棒性技巧，可迁移到 pi0 数据生成 |
| ONNX + 50 行循环即可部署 | pi0 运行时依赖重，对比后看是否需要蒸馏部署路径 |

## 实验路线

### Phase 1: 源码研究（已完成）
- 读完 `bimo_task_env.py` 的 DR 与 reward 实现 ✓
- 读完 `api_example.py` 的推理循环与 obs 构造 ✓
- 记录每一项 DR 技术的参数与出处 ✓
- 输出：[`docs/insights.md`](docs/insights.md) — 对 NormaCore 的 6 大类洞察
  与优先级建议

### Phase 2: 针对性提炼
- 选 Bimo 有而 NormaCore 没有的 1-2 项 DR（候选：backlash + mass）
- 在 `SimToRealAdapter` 里加实现
- 对比实验：同一策略，加 / 不加 backlash 的 eval 差异

### Phase 3: 方法论评估
- 对比 Bimo 的 "obs 结构同源" 设计与 NormaCore pi0 数据 pipeline
- 决定是否在 NormaCore 引入类似契约锁
- 架构决策文档

### Phase 4: 迁移 / 归档
- 有价值的内容合入 `norma_sim/sim_to_real.py` 或 training scripts
- 保留本实验作为方法论参考

## 关键技术问题

### Q1: Backlash 模型对 pi0 有意义吗？
Bimo 的 2.4° backlash 针对 STS-3215 servo。NormaCore 用的 elrobot
follower 用什么舵机？backlash 量是否能测出来？如果是直驱或谐波减速，
可能不适用。

### Q2: Isaac Lab 替代 MuJoCo 的性价比？
Bimo 声称 2048 并行 <15 分钟训练。但 NormaCore 已经用 MuJoCo + pi0
（imitation learning），切到 RL 意味着数据生成方式推倒重来。
判断标准：RL 是否能解决 pick 任务的泛化问题？还是 pi0 + demo 扩量更直接？

### Q3: Obs 结构契约锁能否引入 NormaCore？
Bimo 的 `_get_observations`（sim）和 `process_observations`（real）使用
完全相同的 44 维结构，这是 sim-to-real 成功的前提。NormaCore pi0 的 obs
建模是否也能用这种"单源定义"模式？需要看 pi0 数据生成流程。

## 目录结构

```
experiments/bimo/
├── README.md          ← 本文件
├── agent.md           ← AI Agent 技术指南
├── .gitignore         ← 忽略 bimo-ref/
├── bimo-ref/          ← 参考源码（git clone, ~20 个代码文件）
├── src/               ← 自研实验代码（Phase 2+）
├── tests/             ← 对比测试（Phase 2+）
└── docs/              ← Phase 1 笔记与架构决策文档
```
