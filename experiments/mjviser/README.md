# mjviser 孵化实验

基于 [mjviser](https://github.com/mujocolab/mjviser) 研究 Web 端 MuJoCo
3D 可视化能力，目标是为 NormaCore 引入浏览器内的碰撞几何调试、接触力
可视化、和交互式仿真查看。

## 动机

- WSL2 环境下 `python3 -m mujoco.viewer` 无法正常工作（OpenGL 问题）
- Station Web UI 只渲染 URDF 视觉 mesh，无法显示 MuJoCo 碰撞几何
- 调试碰撞问题（如 SO-100 gripper 不闭合、自碰撞凸包）需要看到
  collision group、contact point、contact force — 目前只能用 headless
  渲染截图，效率低

## 参考项目核心架构

```
mjviser
├── src/mjviser/
│   ├── viewer.py          ← 核心 Viewer 类，封装 MuJoCo + Viser
│   └── cli.py             ← CLI 入口 (uvx mjviser model.xml)
├── examples/              ← 用法示例
└── pyproject.toml         ← pip install mjviser
```

| 特性 | 说明 |
|------|------|
| 渲染方式 | Viser (WebGL)，浏览器访问 |
| 安装 | `pip install mjviser` 或 `uvx mjviser` |
| API | `Viewer(model, data).run()` |
| 扩展点 | `step_fn`, `render_fn`, `reset_fn` 回调 |
| 限制 | 不支持拖拽 body / 键盘回调（需 upstream Viser 支持） |

## 与 NormaCore 的关系

| 当前痛点 | mjviser 能解决 |
|---|---|
| WSL2 无法用 mujoco.viewer | 浏览器访问，无需 OpenGL |
| 看不到碰撞 mesh / 接触力 | Viser 支持 group 切换 + contact 可视化 |
| headless 截图效率低 | 实时交互，拖动相机 |
| Station Web UI 无 MuJoCo 原生渲染 | 可作为独立调试工具或集成到 station |

## 实验路线

### Phase 1: 源码研究与独立验证
- 安装 mjviser，用 SO-100 和 ElRobot 的 scene.xml 跑通
- 验证：碰撞 group 切换、contact point 显示、joint slider
- 评估：渲染质量、延迟、WSL2 兼容性

### Phase 2: 最小可用封装
- 写脚本自动加载 NormaCore 的机器人模型
- 添加 collision/visual group 切换 UI
- 添加 contact force 显示开关

### Phase 3: 集成评估
- 评估是否可以替代 station-viewer 的 Three.js URDF 渲染
- 或作为独立的 "debug viewer" 工具存在
- 对接 norma_sim 的实时仿真数据

### Phase 4: 迁移 / 归档
- 如果价值大：抽取核心功能到 station 或独立工具
- 如果价值有限：保留实验作为参考

## 关键技术问题

### Q1: Viser 在 WSL2 下是否稳定？
Viser 是纯 WebSocket + WebGL，理论上不依赖本地 GPU。需要验证。

### Q2: 能否同时显示 visual 和 collision group？
mjviser 是否暴露 MjvOption.geomgroup 控制？需要读源码确认。

### Q3: 与 norma_sim 的实时对接
mjviser 的 `step_fn` 回调能否接入 norma_sim 的 IPC？还是只能独立跑？

## 目录结构

```
experiments/mjviser/
├── README.md          ← 本文件
├── agent.md           ← AI Agent 技术指南
├── mjviser-ref/       ← 参考源码 (git clone)
├── src/               ← 自研适配代码（Phase 2+）
├── scripts/           ← 验证脚本
└── docs/              ← 补充文档
```
