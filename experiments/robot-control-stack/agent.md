# Robot Control Stack 实验 — Agent 指南

## 源码阅读路径（按优先级）

1. `rcs-ref/python/rcs/envs/base.py` — 统一环境接口，ControlMode、RelativeTo 等核心抽象。**最先读。**
2. `rcs-ref/python/rcs/envs/sim.py` — 仿真环境实现，MuJoCo 如何接入 Gymnasium
3. `rcs-ref/python/rcs/envs/creators.py` — SimEnvCreator，环境工厂 + wrapper 组合
4. `rcs-ref/python/rcs/sim/sim.py` — MuJoCo 底层封装（model/data/step/render）
5. `rcs-ref/examples/so101/so101_env_joint_control.py` — SO101 完整示例
6. `rcs-ref/extensions/rcs_so101/` — SO101 硬件驱动扩展
7. `rcs-ref/python/rcs/camera/interface.py` — 相机抽象接口（sim/hw 统一）

## 架构要点

- **Gymnasium wrapper 堆叠**：SimEnv → RobotWrapper → GripperWrapper → CameraWrapper
- **物理单位全程贯穿**：action/observation 都是弧度，无步数转换
- **C++ 底层 + Python 上层**：性能关键路径在 C++（pybind11），控制逻辑在 Python
- **Extension 模式**：每个机器人是一个 pip-installable extension package
- **GripperConfig 直接用 ctrlrange**：`min_actuator_width = -0.17453` (rad)
- **Digital Twin**：sim 和 hw 可以并行跑在同一个 process 里

## 关键文件速查

| 文件 | 职责 |
|------|------|
| `envs/base.py` | Gymnasium env 基类，ControlMode，action/obs space |
| `envs/sim.py` | SimEnv — MuJoCo 仿真环境 |
| `envs/creators.py` | SimEnvCreator — wrapper 工厂 |
| `sim/sim.py` | MuJoCo model/data 封装 |
| `camera/interface.py` | CameraInterface — sim/hw 统一相机 |
| `camera/sim.py` | MuJoCo 渲染相机 |
| `camera/hw.py` | RealSense 硬件相机 |
| `extensions/rcs_so101/` | SO101 硬件驱动 + IK |

## NormaCore 对标分析

| RCS 组件 | NormaCore 对应 | 差异 |
|---|---|---|
| `envs/base.py` (Gymnasium) | norma_sim + bridge | RCS 直接弧度，NormaCore 走 steps |
| `sim/sim.py` (MuJoCo) | `norma_sim/world/model.py` | 类似 |
| `extensions/rcs_so101` | `sim-bridges/st3215-compat-bridge` | RCS 是 extension，NormaCore 是 bridge |
| `camera/interface.py` | station usb-video | RCS 统一接口，NormaCore 独立模块 |
| GripperConfig | bridge preset YAML | RCS 用 ctrlrange 弧度，NormaCore 用 steps |

## 改造方向

- **可直接复用**：Gymnasium wrapper 模式、GripperConfig 设计、extension 模式
- **需要适配**：C++ 底层替换为 NormaCore 的 Rust 后端
- **不需要**：ROS 替代方案（NormaCore 已有 IPC）、OMPL 运动规划

## 运行环境

```bash
cd experiments/robot-control-stack/rcs-ref
pip install -ve .                    # 安装核心
pip install -ve extensions/rcs_so101  # 安装 SO101 扩展
python examples/so101/so101_env_joint_control.py
```
