# mjviser 实验 — Agent 指南

## 源码阅读路径（按优先级）

1. `mjviser-ref/src/mjviser/scene.py` — ViserMujocoScene，渲染基础层。从 MjModel 创建几何体，从 MjData 更新位置。处理相机追踪、contact、geom/site group 切换。**最先读。**
2. `mjviser-ref/src/mjviser/viewer.py` — Viewer，活跃查看器。拥有仿真循环、实时节拍、回放控制（暂停/步进/重置/速度）、关节/actuator slider、keyframe。接受 `step_fn`/`render_fn`/`reset_fn` 回调。
3. `mjviser-ref/src/mjviser/conversions.py` — MuJoCo→trimesh 转换工具（mesh 提取、纹理、cube map、primitives、heightfields）。
4. `mjviser-ref/src/mjviser/__main__.py` — CLI 入口，加载模型调用 `Viewer(model, data).run()`。
5. `mjviser-ref/examples/` — 用法示例。

## 架构要点

- 两层架构：ViserMujocoScene（渲染） → Viewer（仿真 + UI）
- Viser 是 WebSocket + WebGL 框架，浏览器端渲染，Python 端计算
- geom group 切换在 scene 层实现（`create_groups_gui`）
- contact/force 可视化在 scene 层实现
- joint/actuator slider 在 viewer 层实现
- 回调扩展：`step_fn`（仿真更新）、`render_fn`（帧渲染）、`reset_fn`（状态重置）

## 关键文件速查

| 文件 | 职责 |
|------|------|
| `scene.py` | ViserMujocoScene — 几何创建、位置更新、group 切换、contact 显示 |
| `viewer.py` | Viewer — 仿真循环、UI 控件、slider、keyframe |
| `conversions.py` | MuJoCo mesh/primitive → trimesh 几何转换 |
| `__main__.py` | CLI 入口 (`uvx mjviser model.xml`) |
| `CLAUDE.md` | 开发工作流 + 架构说明 |

## NormaCore 验证脚本思路

```python
# experiments/mjviser/scripts/view_so101.py
from mjviser import Viewer
import mujoco

model = mujoco.MjModel.from_xml_path(
    "hardware/elrobot/simulation/vendor/therobotstudio/SO101/scene.xml")
data = mujoco.MjData(model)
Viewer(model, data).run()
# → 浏览器打开，可交互查看 SO101
# → 切换 geom group 看碰撞 mesh
# → 打开 contact 显示看接触力
```

```python
# experiments/mjviser/scripts/view_elrobot.py
from mjviser import Viewer
import mujoco

model = mujoco.MjModel.from_xml_path(
    "hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml")
data = mujoco.MjData(model)
Viewer(model, data).run()
```

## 改造方向

- **可直接复用**：Viewer 的 group 切换、contact 显示、joint slider — 这些是我们需要的核心功能
- **需要适配**：与 norma_sim 的 IPC 对接（如果要实时查看仿真状态）
- **不需要**：CLI fuzzy matching、robot_descriptions 库集成

## 运行环境

```bash
cd experiments/mjviser/mjviser-ref
pip install -e .          # 或 uv pip install -e .
mjviser ../../hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml
# → http://localhost:8012
```
