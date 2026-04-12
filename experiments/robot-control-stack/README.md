# Robot Control Stack (RCS) 孵化实验

基于 [Robot Control Stack](https://github.com/RobotControlStack/robot-control-stack)
研究统一 Sim-Real 控制接口的最佳实践，目标是为 NormaCore 引入教科书级别的
硬件抽象层。

## 动机

NormaCore 当前架构让 Web UI 通过 ST3215 servo 协议（步数）控制仿真，
导致 slider 映射错位（已修复 ctrlrange 自动计算）。RCS 的 wrapper 模式
直接用物理单位（弧度），天然不存在这个问题。

## 参考项目核心架构

```
Application (RL policy / teleoperation)
    ↓ Gymnasium API (action = radians)
SimEnvCreator → RobotWrapper → GripperWrapper → CameraWrapper
    ↓                              ↓
MuJoCo Sim                    Hardware Driver (C++)
```

| 特性 | 说明 |
|------|------|
| 统一接口 | Gymnasium env，action/obs 全部物理单位 |
| 支持机器人 | Franka FR3, xArm7, UR5e, **SO101** |
| 仿真 | MuJoCo（含 viewer GUI） |
| 硬件 | C++ 底层驱动，Python binding |
| 零 ROS 依赖 | 同步执行模型，不依赖 ROS 中间件 |
| Digital twin | sim + real 并行，碰撞检测守护 |

## 与 NormaCore 的关系

| NormaCore 痛点 | RCS 怎么做 |
|---|---|
| Web UI 说硬件协议 (steps) | Gymnasium API 直接用弧度 |
| Bridge 转换有 range 映射问题 | 无 bridge，sim 直通 |
| 加新机器人要写 preset | extension 模式，MJCF + config 即可 |
| Gripper 行程映射复杂 | GripperWrapper 统一处理 min/max |

## SO101 在 RCS 中的集成

```python
# RCS SO101 示例 — 6 行创建环境
robot_cfg = sim.SimRobotConfig()
robot_cfg.robot_type = rcs.common.RobotType.SO101
gripper_cfg = sim.SimGripperConfig()
gripper_cfg.min_actuator_width = -0.17453  # rad (MJCF ctrlrange min)
gripper_cfg.max_actuator_width =  1.74533  # rad (MJCF ctrlrange max)

env = SimEnvCreator()(robot_cfg=robot_cfg, gripper_cfg=gripper_cfg, ...)
obs, info = env.reset()
obs, reward, done, trunc, info = env.step(action_in_radians)
```

**注意：** Gripper 的 min/max 直接从 MJCF ctrlrange 来。不需要 offset、
不需要 steps 转换、不需要 bridge preset。这就是教科书做法。

## 实验路线

### Phase 1: 源码研究
- 读 `python/rcs/envs/base.py` — 统一环境接口
- 读 `python/rcs/envs/sim.py` — 仿真后端实现
- 读 `python/rcs/sim/sim.py` — MuJoCo 封装
- 读 `extensions/rcs_so101/` — SO101 硬件驱动
- 读 `examples/so101/` — SO101 用法示例

### Phase 2: 最小验证
- 用 RCS 跑 SO101 仿真
- 对比 NormaCore + mjviser 的行为差异
- 评估 Gymnasium wrapper 模式的实用性

### Phase 3: 适配评估
- 评估是否将 RCS 的 wrapper 模式引入 NormaCore
- 或直接用 RCS 替代 norma_sim + bridge 层
- 架构决策文档

## 目录结构

```
experiments/robot-control-stack/
├── README.md          ← 本文件
├── agent.md           ← AI Agent 技术指南
├── .gitignore         ← 忽略 rcs-ref/
└── rcs-ref/           ← 参考源码 (845 files)
```
