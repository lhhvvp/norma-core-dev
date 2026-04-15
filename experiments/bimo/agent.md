# Bimo 实验 — Agent 指南

## 源码阅读路径（按优先级）

1. `bimo-ref/IsaacLab/bimo/bimo_task_env.py` — **最先读，最重要**。
   `BimoEnv` 的 `_get_observations` / `_pre_physics_step` / `_get_rewards` /
   `_reset_idx` 四个方法承载了全部 sim-to-real 关键逻辑：域随机化、
   backlash、actuator delay、obs history、reward shaping。文件末尾的
   `@torch.jit.script` 函数是 7 个 reward 组件的实现。
2. `bimo-ref/BimoAPI/examples/api_example.py` — 完整推理循环（只有 115
   行）。看 `update_buffers` / `process_observations` / `process_actions`，
   确认它与 `bimo_task_env.py::_get_observations` 的 obs 构造字节一致。
   **这是 sim-to-real 契约锁的最佳教材。**
3. `bimo-ref/BimoAPI/bimo/bimo.py` — 硬件层封装。重点看 `servo_min/max/
   centers`、`deg2servo` / `servo2deg`、`request_state_data`（结构体解包
   `<4f4H1H8H8h8h8H8h8B`）、`lock_heading`。
4. `bimo-ref/IsaacLab/bimo/bimo_config.py` — `DCMotorCfg` 的
   stiffness=35, damping=0.8, saturation=2.94 Nm。参数量少，速读。
5. `bimo-ref/IsaacLab/bimo/agents/rsl_rl.py` — PPO 超参数，actor/critic
   [512, 256, 128] ELU。作为训练配方参考。
6. `bimo-ref/IsaacLab/README.md` — 官方对 obs 结构、reward 组件、DR
   策略的英文说明（权威来源）。
7. `bimo-ref/MCU/README.md` — 二进制 serial 协议文档（不需要写入
   NormaCore，但有助于理解硬件边界）。
8. `bimo-ref/BimoAPI/bimo/routines.py` — 预编程时间插值动作，与 RL 无关，
   扫一眼即可。

## 架构要点

- **Obs 结构单源定义**：44 维 = 4 步 orient history × 3 + 4 步 action
  history × 8。sim 端 (`_get_observations`) 与 real 端 (`process_
  observations`) 必须完全一致，这是 sim-to-real 成功的契约前提。
- **动作是增量而非绝对值**：`cmd_actions += clip(action, -3, 3) * 4/3`
  （每步最多 4°）。降低了动作空间学习难度，但要求 `cmd_actions` 在
  reset 时初始化到 `stand_pose`。
- **域随机化分层**：
  - **Per-step**：IMU orient 噪声（σ=0.015）、actuator 噪声（σ=0.5°）、
    actuator delay（0-1 物理步随机）
  - **Per-reset**：link mass ±5%、foot pad 材料（static_friction 0.4-0.9、
    restitution 0-0.05）、torque 档位轮换（2.7-2.94 Nm）
  - **Per-interval**：周期性 head 推力（2-4 秒随机一次，±0.2 m/s）
  - **Always-on**：backlash 2.4° 反向死区
- **Reward = 7 项加权求和**：weights 在 `BimoEnvCfg.weights` 按任务预设
  （walk / turn / stop），运行时归一化到总和 1。walk preset：
  `[1, 1, 1, 0, 1, 1, 1]` —— 对应 orientation / height / joint_pos /
  sigmoid_extra / feet_height / velocity / deviation。
- **Termination**：head 高度 < 0.1m 或 abs(roll) > 0.95 或 abs(pitch) >
  0.95 rad。
- **初始化的鲁棒性技巧**：`act_hist[env_ids, :] = base_pose[0]`，代码注释
  明确说"用原始度数值，不做 [-1,1] scale，让策略学会从任意 history
  启动，显著改善 sim-to-real"。

## 关键文件速查

| 文件 | 职责 |
|------|------|
| `IsaacLab/bimo/bimo_task_env.py` | `BimoEnv` + 7 reward 函数 + DR 全部实现 |
| `IsaacLab/bimo/bimo_config.py` | `BIMO_CFG` ArticulationCfg（USD + DCMotor） |
| `IsaacLab/bimo/agents/rsl_rl.py` | PPO 超参数 |
| `IsaacLab/bimo/assets/Bimo.usd` | Isaac Sim 机器人模型（二进制，不可读） |
| `BimoAPI/bimo/bimo.py` | MCU serial + 相机 + 坐标转换 |
| `BimoAPI/bimo/routines.py` | `BimoRoutines.perform()` 时间插值动作 |
| `BimoAPI/examples/api_example.py` | 完整 ONNX 推理循环（与 task_env obs 对齐） |
| `MCU/micro_bimo.ino` | RP2040 固件（Arduino，binary serial） |

## NormaCore 对标分析

| Bimo 组件 | NormaCore 对应 | 差异 / 启示 |
|---|---|---|
| `bimo_task_env.py` DR | `norma_sim/sim_to_real.py` SimToRealAdapter | Bimo 在训练环境里做 DR，NormaCore 在 adapter wrapper 里做 — 两种路线都行，但 Bimo 的 link mass / material / torque 是 NormaCore adapter 目前表达不了的（它只包装 obs/action，不碰物理量） |
| `BIMO_CFG` DCMotorCfg | elrobot_follower MJCF + `capabilities.py` | Bimo：DCMotorCfg 显式 stiffness/damping/saturation；NormaCore：MJCF ctrlrange + capabilities.py 硬限 |
| `api_example.py` 推理循环 | station 实时控制路径 | Bimo 依赖极轻（onnxruntime + pyserial），NormaCore pi0 依赖极重 — 对比后看是否需要蒸馏部署路径 |
| `routines.py` 时间插值 | `sim_worlds/*.yaml` pose sequences | 功能相同，Bimo 代码化，NormaCore 声明式 |
| `bimo.py` quaternion→euler | `norma_sim` 里的对应函数 | Bimo 在 Python 和 torch 两边各实现一次，可对比实现是否一致 |
| `lock_heading()` yaw 归零 | — | NormaCore 暂无对应，机械臂任务未必需要 |

## 改造方向（Phase 2+ 候选）

- **优先级 1：Backlash 模型**
  Bimo 的实现是最简形式（方向反转时先消耗死区量，`_pre_physics_step`
  里 `direction_changed` 的分支）。可以复制到 `SimToRealAdapter`，但
  需要先测量 elrobot 实机的 backlash 量。
- **优先级 2：Per-reset mass / friction 随机化**
  需要 NormaCore 的仿真后端支持动态改物理参数。需要看
  `MuJoCoWorld` 是否暴露 `model.body_mass` / `geom_friction` 的写入接口；
  如果没有，需要新增。
- **优先级 3：Obs 结构契约锁**
  把 `SimToRealAdapter` 的 obs shape 定义抽到一个共享模块，让训练
  环境与 pi0 推理路径都从同一个地方读。
- **优先级 4：act_hist 启动鲁棒性**
  pi0 的数据生成可能可以引入相同思路，让 episode 第一帧的 action
  history 带一点扰动。
- **不需要**：Isaac Lab 迁移、RSL-RL PPO 栈、RP2040 固件、BimoAPI 控制
  库、行走 reward 函数、Bimo.usd 模型。

## 运行环境

```bash
# 本实验以源码研究为主，不需要真的跑 Bimo
ls experiments/bimo/bimo-ref/IsaacLab/bimo/bimo_task_env.py  # 主研究文件

# 如果想验证 BimoAPI 能否 import（独立于 Isaac Lab）
cd experiments/bimo/bimo-ref/BimoAPI
pip install -e .                                              # 装 mekion-bimo
python -c "from bimo import Bimo; help(Bimo)"                 # 验证 API

# 完整训练（非本实验目标，仅留作参考）
#   需要 Isaac Lab 2.3.0 + 高端 GPU
#   cp -r IsaacLab/bimo $ISAAC_LAB/source/isaaclab_tasks/isaaclab_tasks/direct/
#   ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
#       --task Bimo --num_envs 2048 --headless
```

## 关键参数参考表

从 `bimo_task_env.py` 提取的 DR 关键参数，供 Phase 2 对照：

| 参数 | 值 | 位置 |
|------|----|----|
| `decimation` | 10 | `BimoEnvCfg` |
| `dt`（物理步） | 0.005 s | `BimoEnvCfg` |
| 控制频率 | 20 Hz | `dt × decimation = 50 ms` |
| `actuator_delay_max` | 1 物理步 | `BimoEnvCfg` |
| `backlash` | 2.4° | `BimoEnvCfg` |
| `orient_noise` σ | 0.015 | `BimoEnv.__init__` |
| `actuator_noise` σ | 0.5° | `BimoEnv.__init__` |
| `torque_range` | 2.7-2.94 Nm, 24 步 | `BimoEnv.__init__` |
| link mass scale | 0.95-1.05 | `events.randomize_link_mass` |
| push velocity | ±0.2 m/s（walk） | `push_velocities[obj]` |
| push interval | 2-4 s | `events.periodic_push` |
| foot static_friction | 0.4-0.9 | `_setup_scene` |
| foot restitution | 0-0.05 | `_setup_scene` |
| ideal height | 0.381 m | `height_reward` |
| termination height | 0.1 m | `_get_dones` |
| termination tilt | 0.95 rad | `_get_dones` |
