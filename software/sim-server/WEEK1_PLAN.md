# Week 1: ACT 训练闭环验证 — GPU 机器执行计划

> 目标：在 GPU 机器上跑通 **数据生成 → ACT 训练 → 仿真评估** 完整闭环
> 前置：3090 GPU、Ubuntu、Python 3.11+
> 预计用时：3-5 天

---

## Day 0: 环境搭建

### 0.1 克隆仓库

```bash
git clone <norma-core-repo> ~/proj/norma-core
cd ~/proj/norma-core/software/sim-server
```

### 0.2 安装依赖

```bash
# 基础 sim 依赖
pip install -e .

# LeRobot (v0.5+, 包含 ACT + pi0-FAST)
pip install lerobot

# 如果 lerobot 版本 < 0.5，用 git 安装最新版:
# pip install git+https://github.com/huggingface/lerobot.git

# GPU 推理
pip install torch torchvision  # CUDA 版本
```

### 0.3 验证仿真能跑

```bash
cd ~/proj/norma-core/software/sim-server

# 跑测试
PYTHONPATH=. python3 -m pytest tests/ -x -q

# 手动启动 sim 确认无报错
PYTHONPATH=. python3 -m norma_sim \
  --manifest ../../hardware/elrobot/simulation/manifests/norma/therobotstudio_so101_tabletop.scene.yaml \
  --socket /tmp/norma-sim-test.sock \
  --physics-hz 500 --mode stepping
# Ctrl+C 退出
```

### 0.4 验证现有 demo 录制脚本

```bash
PYTHONPATH=. python3 scripts/record_scripted_demo.py
# 应输出: datasets/norma_sim_pick_demo/ 下有 2 个 episode
```

---

## Day 1-2: 批量数据生成器

### 1.1 创建 `scripts/batch_generate.py`

核心逻辑：在 `record_scripted_demo.py` 基础上扩展。

**需要实现的功能**：

```python
# 参数
NUM_EPISODES = 200       # 目标 episode 数
CAMERAS = ["top", "wrist.top"]
FPS = 30
DATASET_DIR = "datasets/norma_sim_pick_v1"
REPO_ID = "norma/sim_pick_v1"

# 关键：Domain Randomization
# 每个 episode 随机化以下参数：
RANDOMIZE = {
    # 1. 目标位置变化 — 改变 waypoint 里的 shoulder_pan 和关节角度
    "target_shoulder_pan": (-0.8, 0.8),   # rad, 改变 carry/release 的方向
    "target_elbow_flex":   (1.2, 1.6),    # rad, 改变 approach 深度

    # 2. 运动速度变化 — 改变每段 waypoint 的 interpolation 步数
    "speed_factor": (0.7, 1.3),           # 步数乘以这个系数

    # 3. 微小关节扰动 — 给每步 action 加噪声
    "action_noise_std": 0.02,             # rad, 模拟人类操作不精确

    # 4. 起始姿态微扰
    "home_noise_std": 0.05,               # rad, 每次 home 位置略有不同
}
```

**Waypoint 生成逻辑**（从 `record_scripted_demo.py` 的 WAYPOINTS 扩展）：

```python
def generate_waypoints(rng: np.random.Generator) -> list:
    """生成一组随机化的 pick-and-place waypoints."""
    # 基础参数
    pan = rng.uniform(-0.8, 0.8)        # 在哪个方向放置
    approach_flex = rng.uniform(1.2, 1.6)  # 多深
    lift_flex = rng.uniform(0.8, 1.2)      # 抬多高
    speed = rng.uniform(0.7, 1.3)

    def s(base_steps):
        return max(10, int(base_steps * speed))

    home_noise = rng.normal(0, 0.05, size=5)
    home = [h + n for h, n in zip([0.0]*5, home_noise)]

    return [
        ("home",     home,                                           0,   s(30)),
        ("above",    [0.0, -0.6, 1.3, -0.1, 0.0],                  0,   s(40)),
        ("approach", [0.0, -0.6, approach_flex, 0.0, 0.0],          0,   s(30)),
        ("grasp",    [0.0, -0.6, approach_flex, 0.0, 0.0],        100,   s(20)),
        ("lift",     [0.0, -0.6, lift_flex, -0.3, 0.0],           100,   s(40)),
        ("carry",    [pan, -0.4, 0.8, -0.2, 0.0],                 100,   s(40)),
        ("release",  [pan, -0.4, 0.8, -0.2, 0.0],                   0,   s(20)),
        ("home",     home,                                           0,   s(40)),
    ]
```

**数据写入格式**（LeRobot v3 — 复用 `record_scripted_demo.py` 的 `LeRobotDataset.create()` 模式）：

```python
features = {
    "observation.state": {
        "dtype": "float32",
        "shape": (6,),                  # 5 joints + 1 gripper
        "names": {"motors": motor_names},
    },
    "action": {
        "dtype": "float32",
        "shape": (6,),
        "names": {"motors": motor_names},
    },
    "observation.images.top": {
        "dtype": "image",
        "shape": (480, 640, 3),
        "names": ["height", "width", "channel"],
    },
    "observation.images.wrist.top": {
        "dtype": "image",
        "shape": (480, 640, 3),
        "names": ["height", "width", "channel"],
    },
}

dataset = LeRobotDataset.create(
    repo_id=REPO_ID,
    fps=FPS,
    features=features,
    root=DATASET_DIR,
    robot_type="norma_sim",
    use_videos=True,          # v3 用 MP4, 比 images 省空间
    image_writer_processes=2,  # GPU 机器多核并行编码
    image_writer_threads=4,
)
```

**每个 episode 的录制循环**（和 `record_scripted_demo.py` 基本相同）：

```python
for ep in range(NUM_EPISODES):
    rng = np.random.default_rng(seed=ep)
    waypoints = generate_waypoints(rng)

    # 录制 episode (复用现有逻辑)
    current_joints = waypoints[0][1][:5]
    current_gripper = waypoints[0][2]

    for wp_name, target_joints, target_gripper, n_steps in waypoints:
        for step in range(n_steps):
            t = (step + 1) / n_steps
            joints = interpolate(current_joints, target_joints[:5], t)
            gripper = current_gripper + (target_gripper - current_gripper) * t

            # 加动作噪声
            joints = [j + rng.normal(0, 0.02) for j in joints]

            # send_action → get_observation → add_frame
            # ... (同 record_scripted_demo.py)

            frame["task"] = "pick up the red cube and place it to the side"

        current_joints = target_joints[:5]
        current_gripper = target_gripper

    dataset.save_episode()
    print(f"Episode {ep+1}/{NUM_EPISODES} done")

dataset.consolidate()  # 合并 parquet chunks
```

### 1.2 运行数据生成

```bash
cd ~/proj/norma-core/software/sim-server
PYTHONPATH=. python3 scripts/batch_generate.py

# 预期输出:
#   datasets/norma_sim_pick_v1/
#   ├── data/
#   │   ├── train-00000-of-00001.parquet
#   │   └── videos/
#   │       ├── observation.images.top/
#   │       └── observation.images.wrist.top/
#   └── meta/
#       ├── info.json
#       └── episodes.jsonl
#
# 200 episodes × ~260 frames/ep ≈ 52,000 frames
# 预计耗时: 30-60 分钟 (视 MuJoCo 渲染速度)
```

### 1.3 验证数据集

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset(repo_id="norma/sim_pick_v1", root="datasets/norma_sim_pick_v1")
print(f"Episodes: {ds.num_episodes}")
print(f"Frames: {ds.num_frames}")
print(f"Features: {list(ds.features.keys())}")
print(f"FPS: {ds.fps}")

# 抽查一帧
frame = ds[0]
print(f"State shape: {frame['observation.state'].shape}")   # (6,)
print(f"Action shape: {frame['action'].shape}")              # (6,)
print(f"Image shape: {frame['observation.images.top'].shape}")  # (3, 480, 640)
```

---

## Day 2-3: ACT 训练

### 2.1 训练配置

LeRobot 的 ACT 训练通过 hydra config 驱动。创建自定义 config：

```bash
# 方式 A: 命令行参数覆盖 (推荐, 简单)
cd ~/proj/norma-core/software/sim-server

python3 -m lerobot.scripts.train \
  --policy.type=act \
  --dataset.repo_id=norma/sim_pick_v1 \
  --dataset.root=datasets/norma_sim_pick_v1 \
  --dataset.local_files_only=true \
  --training.batch_size=8 \
  --training.num_workers=4 \
  --training.steps=50000 \
  --training.save_interval=10000 \
  --training.log_interval=100 \
  --training.eval_freq=0 \
  --output_dir=outputs/act_pick_v1 \
  --device=cuda
```

**关键参数说明**：

| 参数 | 值 | 说明 |
|------|---|------|
| `batch_size` | 8 | 3090 24GB 跑 ACT 很宽裕, 可以试 16 |
| `steps` | 50000 | ACT 一般 20K-50K 就收敛 |
| `chunk_size` | 100 (默认) | ACT action chunk 大小 |
| `n_action_steps` | 100 (默认) | 执行多少步 |
| `eval_freq` | 0 | 先关掉 online eval, 训完再手动评估 |

### 2.2 监控训练

```bash
# TensorBoard
tensorboard --logdir outputs/act_pick_v1

# 关注指标:
# - train/loss: 应稳步下降
# - train/l1_loss: action 预测的 L1 误差
# 预计 3090 上 50K steps 需要 2-4 小时
```

### 2.3 训练完成后检查

```bash
ls outputs/act_pick_v1/checkpoints/
# 应有: 010000/ 020000/ 030000/ 040000/ 050000/ (last)

# 每个 checkpoint 目录结构:
# ├── config.json
# ├── model.safetensors
# └── config.yaml
```

---

## Day 3-4: 仿真闭环评估

### 3.1 创建 `scripts/eval_policy.py`

基于现有 `test_policy_eval.py` 改造，支持：
- 加载本地训练的 checkpoint（不只是 HuggingFace remote）
- 计算成功率指标
- 多 episode 统计

**核心改动**：

```python
"""Evaluate a trained policy in NormaSimEnv.

Usage:
    PYTHONPATH=. python3 scripts/eval_policy.py \
      --checkpoint outputs/act_pick_v1/checkpoints/050000 \
      --episodes 20 \
      --render-port 8012
"""
import argparse
import time
import numpy as np
import torch

def load_policy(checkpoint_path: str, device: str = "cuda"):
    """Load ACT policy from local checkpoint."""
    from lerobot.policies.act.modeling_act import ACTPolicy
    policy = ACTPolicy.from_pretrained(checkpoint_path)
    policy.eval()
    return policy.to(device)

def evaluate_episode(robot, policy, device, max_steps=300):
    """Run one episode, return success metrics."""
    # reset env 需要在 NormaSimRobot 层面加 reset() 方法
    # 或者断开重连 (简单但慢)

    actions_taken = []
    states_seen = []

    for step_i in range(max_steps):
        obs = robot.get_observation()

        # 组装 batch (同 test_policy_eval.py)
        state = torch.tensor([
            obs["shoulder_pan.pos"],
            obs["shoulder_lift.pos"],
            obs["elbow_flex.pos"],
            obs["wrist_flex.pos"],
            obs["wrist_roll.pos"],
            obs["gripper.pos"],
        ], dtype=torch.float32).unsqueeze(0).to(device)

        batch = {"observation.state": state}

        # 添加图像
        for cam_name in ["top", "wrist.top"]:
            key = f"observation.images.{cam_name}"
            if key in obs:
                img = obs[key].astype(np.float32) / 255.0
                img = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).to(device)
                batch[key] = img

        with torch.no_grad():
            action = policy.select_action(batch)

        action_np = action.squeeze(0).cpu().numpy()
        action_dict = {
            "shoulder_pan.pos": float(action_np[0]),
            "shoulder_lift.pos": float(action_np[1]),
            "elbow_flex.pos": float(action_np[2]),
            "wrist_flex.pos": float(action_np[3]),
            "wrist_roll.pos": float(action_np[4]),
            "gripper.pos": float(action_np[5]),
        }
        robot.send_action(action_dict)

        actions_taken.append(action_np.copy())
        states_seen.append(state.squeeze(0).cpu().numpy().copy())

    return {
        "actions": np.array(actions_taken),
        "states": np.array(states_seen),
    }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--max-steps", type=int, default=300)
    parser.add_argument("--render-port", type=int, default=0)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()

    device = torch.device(args.device)
    policy = load_policy(args.checkpoint, device)

    from norma_sim.lerobot_robot import NormaSimRobot, NormaSimRobotConfig

    REPO_ROOT = "../../"
    MANIFEST = f"{REPO_ROOT}/hardware/elrobot/simulation/manifests/norma/therobotstudio_so101_tabletop.scene.yaml"

    config = NormaSimRobotConfig(
        manifest_path=MANIFEST,
        physics_hz=500,
        action_hz=30,
        render_port=args.render_port,
        cameras=["top", "wrist.top"],
    )

    results = []
    for ep in range(args.episodes):
        robot = NormaSimRobot(config)
        robot.connect()

        result = evaluate_episode(robot, policy, device, args.max_steps)
        results.append(result)

        robot.disconnect()
        print(f"Episode {ep+1}/{args.episodes} done")

    # 基础分析: action 分布是否合理
    all_actions = np.concatenate([r["actions"] for r in results])
    print(f"\n=== Evaluation Summary ({args.episodes} episodes) ===")
    print(f"Action stats:")
    for i, name in enumerate(["shoulder_pan", "shoulder_lift", "elbow_flex",
                               "wrist_flex", "wrist_roll", "gripper"]):
        col = all_actions[:, i]
        print(f"  {name:15s}: mean={col.mean():+.3f} std={col.std():.3f} "
              f"min={col.min():+.3f} max={col.max():+.3f}")

if __name__ == "__main__":
    main()
```

### 3.2 运行评估

```bash
cd ~/proj/norma-core/software/sim-server

# 无渲染, 纯数字评估
PYTHONPATH=. python3 scripts/eval_policy.py \
  --checkpoint outputs/act_pick_v1/checkpoints/050000 \
  --episodes 20

# 有渲染, 肉眼观察
PYTHONPATH=. python3 scripts/eval_policy.py \
  --checkpoint outputs/act_pick_v1/checkpoints/050000 \
  --episodes 5 \
  --render-port 8012
# 然后浏览器打开 http://localhost:8012 观察
```

### 3.3 评估标准

Week 1 的成功标准 **不是** 高成功率，而是：

| 检查项 | 通过条件 |
|--------|---------|
| 数据生成 | 200 episodes, ~52K frames, 无报错 |
| 数据格式 | LeRobot v3 可加载, state/action/image shape 正确 |
| ACT 训练 | loss 收敛 (从 ~1.0 降到 <0.1) |
| 推理闭环 | checkpoint 能加载, 输出合理范围的 action |
| 行为观察 | 机械臂有明确的运动意图 (不是随机抖动) |

**即使成功率是 0%**，只要上述 5 条都通过，管线就验证成功——Week 2 换 pi0-FAST 时数据和评估代码直接复用。

---

## Day 4-5: 调试 & 迭代

### 常见问题 & 排查

**问题 1: loss 不下降**
```bash
# 检查数据归一化
python3 -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(repo_id='norma/sim_pick_v1', root='datasets/norma_sim_pick_v1')
frame = ds[0]
print('state range:', frame['observation.state'].min().item(), frame['observation.state'].max().item())
print('action range:', frame['action'].min().item(), frame['action'].max().item())
"
# state 和 action 应在 [-3.14, 3.14] 范围内 (弧度)
# gripper 在 [0, 100] (LeRobot scale)
# 如果有 NaN 或极端值, 数据生成有 bug
```

**问题 2: 推理时 action 值不合理 (全零/爆炸)**
```bash
# 确认 checkpoint 的 normalization stats 和数据集匹配
ls outputs/act_pick_v1/checkpoints/050000/
# 应有 config.json, model.safetensors
# config.json 里的 input_features/output_features 应和数据集 features 一致
```

**问题 3: MuJoCo 渲染报错**
```bash
# 确认 GPU 上有 EGL 支持 (offscreen rendering)
python3 -c "import mujoco; print(mujoco.__version__)"
# 如果 offscreen 不工作, 设置:
export MUJOCO_GL=egl
# 或 (CPU fallback, 慢但一定能跑):
export MUJOCO_GL=osmesa
```

**问题 4: CUDA OOM**
```bash
# 降低 batch_size
--training.batch_size=4
# 或降低 image 分辨率 (需要同时改数据生成和训练)
```

### 迭代方向

如果基础管线通了，可以在 Day 5 尝试：

1. **增加数据多样性**：更大的关节角范围、更多 waypoint 变体
2. **调整 chunk_size**：试试 50 vs 100，看哪个 loss 更低
3. **加 action noise**：从 0.02 rad 增加到 0.05，看是否提升泛化
4. **降低图像分辨率**：480×640 → 240×320，加速训练（对 ACT 通常影响不大）

---

## Week 2 预告 (Week 1 完成后)

```
Week 1 产出:
  ✓ 数据管线 (batch_generate.py)
  ✓ 训练管线 (lerobot train)
  ✓ 评估管线 (eval_policy.py)
  ✓ ACT baseline 成功率数字

Week 2 只需要改:
  1. 训练命令: policy.type=act → policy.type=pi0fast
  2. 数据集: 加 language_instruction 字段 (pi0 需要)
  3. 训练参数: batch_size=8 → batch_size=1 (LoRA, 3090 限制)
  4. 评估: 相同的 eval_policy.py, 只换 checkpoint 路径
```

---

## 文件清单 (需要创建/修改的文件)

| 文件 | 状态 | 说明 |
|------|------|------|
| `scripts/batch_generate.py` | **新建** | 批量数据生成器 |
| `scripts/eval_policy.py` | **新建** | 通用 policy 评估脚本 |
| `test_policy_eval.py` | 不改 | 保留作为参考 |
| `record_scripted_demo.py` | 不改 | 保留作为参考 |
| `norma_sim/lerobot_robot.py` | 可能微调 | 如果需要加 `reset()` 方法 |
| `norma_sim/gym_env.py` | 不改 | 已完备 |

---

## 快速参考命令

```bash
# 0. 环境
cd ~/proj/norma-core/software/sim-server
export PYTHONPATH=.
export MUJOCO_GL=egl  # GPU offscreen rendering

# 1. 数据生成
python3 scripts/batch_generate.py

# 2. 验证数据
python3 -c "from lerobot.datasets.lerobot_dataset import LeRobotDataset; ds = LeRobotDataset(repo_id='norma/sim_pick_v1', root='datasets/norma_sim_pick_v1'); print(f'{ds.num_episodes} eps, {ds.num_frames} frames')"

# 3. 训练 ACT
python3 -m lerobot.scripts.train \
  --policy.type=act \
  --dataset.repo_id=norma/sim_pick_v1 \
  --dataset.root=datasets/norma_sim_pick_v1 \
  --dataset.local_files_only=true \
  --training.batch_size=8 \
  --training.steps=50000 \
  --output_dir=outputs/act_pick_v1 \
  --device=cuda

# 4. 评估
python3 scripts/eval_policy.py \
  --checkpoint outputs/act_pick_v1/checkpoints/050000 \
  --episodes 20

# 5. (可选) 带可视化评估
python3 scripts/eval_policy.py \
  --checkpoint outputs/act_pick_v1/checkpoints/050000 \
  --episodes 5 --render-port 8012
```
