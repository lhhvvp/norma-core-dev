# experiments/ — 孵化实验目录

NormaCore 的技术孵化空间。每个子目录是一个独立实验，
研究外部项目/技术，验证可行性，最终孵化为可集成到主系统的功能。

## 孵化流程

```
发现有价值的开源项目/技术
    │
    ├── 1. 建目录，clone 参考源码到 *-ref/ 子目录
    ├── 2. 写 README.md（面向人）+ agent.md（面向 AI）
    ├── 3. Phase 1: 源码研究，理解核心架构
    ├── 4. Phase 2: 最小验证，跑通关键路径
    ├── 5. Phase 3: 适配封装，对接 station / sim-server
    └── 6. Phase 4: 迁移到主系统，实验归档
```

## 标准目录结构

```
experiments/<实验名>/
├── README.md              # 必须 — 孵化总览
├── agent.md               # 必须 — AI Agent 技术指南
├── <项目名>-ref/          # 参考源码 (git clone)
├── src/                   # 自研代码（Phase 2+ 产出）
├── tests/                 # 测试
└── scripts/               # 实验脚本
```

## 当前实验一览

| 实验 | 方向 | 参考源码 | 阶段 |
|------|------|---------|------|
| mjviser | Web MuJoCo 可视化 | mjviser-ref/ | Phase 1 |
| robot-control-stack | Sim-Real 统一控制接口 | rcs-ref/ | Phase 1 |

## 与主系统的关系

```
norma-core/
├── software/station/      ← Station + Web UI (集成目标)
├── software/sim-server/   ← MuJoCo 仿真服务
├── hardware/elrobot/      ← 机器人模型 + vendor
└── experiments/           ← 本目录：技术孵化
```
