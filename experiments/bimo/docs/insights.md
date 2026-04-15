# Bimo 学习洞察 — 对 NormaCore 的启示

读完 `bimo-ref/` 全部源码（~20 文件，BimoAPI + IsaacLab + MCU 三部分）后，
对照 NormaCore 当前代码库整理的洞察。所有 file:line 都经过原文确认。

本文档是 Phase 1 的**结论输出**，对应 `README.md` 里声明的"输出：
docs/domain-randomization-notes.md"（实际存为 `insights.md`，含 DR 之外
的方法论与架构分析）。

> **修订记录（2026-04-15 Codex 独立验证）**
>
> 本文档经过 Codex（codex-rescue 子 agent，6.77M tokens、92% cache hit）
> 独立验证。5 条核心事实声明**全部被标记为 "partially correct"** ——
> 方向都对，但细节处有多个 load-bearing 错误。关键修正：
>
> 1. **Section 零**：pi0 问题比原文描述更严重 —— 不只是"部署时 Station
>    加载了错的 policy"，而是**训练侧的数据契约在部署之前就已经坏**。
>    `train_pi0_openpi.py:39-64` 把图像 repack 成 `images.cam_high`，
>    但 openpi 模型期望 `image.*` + `image_mask` 结构
>    （`openpi/transforms.py:185-191`, `openpi/models/model.py:109-129`）。
> 2. **Section 1.1**：MuJoCo `mj_setConst` API 存在（`mujoco.h:226-227`），
>    直接写 `body_mass`/`geom_friction` 需要刷新衍生字段
>    （`body_subtreemass`, `body_invweight0`, `dof_invweight0`, `dof_M0`）。
>    仓库里已有的 sysid 代码
>    (`elrobot_follower/measurements/sysid/mujoco_sim.py:121-128`)
>    是"mutate → reset data → simulate"，不是"write and go"。
>    原估算 2-3 天 → 修正为 **4-7 天**。
> 3. **Section 1.2**：STS-3215 虽同型号，但 vendor 的 backlash class
>    **定义了但未启用**（`VENDOR.md:53-55`）。表述应改为"硬件兼容
>    起点"而非"直接可迁移"。
> 4. **Section 2.1**：`lerobot_helpers.py:84-101` **不是独立构造器**，
>    而是两个 backend 共用的 adapter。**真正的漂移在脚本里的手写
>    state/action 向量构造**：`gen_dataset_v21.py:99-105`、
>    `run_experiment.py:135-143`、`record_scripted_demo.py:145-156`、
>    `batch_generate.py:172-175`。原估算 1-2 天 → 修正为 **2-4 天**。
> 5. **Section 3.1**：**最严重的错误**。两个致命问题：(a) pi0 **不吃
>    action history**，输入是 images + state + prompt
>    （`openpi/models/pi0_config.py:63-86`），原文描述错误；
>    (b) **起点扰动已经存在**于 `gen_dataset_v21.py:34-50` 和
>    `pick_and_place.py:23-31,44-55` —— 我说的"最小改动最具体回报"
>    指向的是一个**已经实现的功能**。该条降级为"仅验证覆盖"。
> 6. **Codex 发现的关键遗漏**（本文原版未覆盖）：**NormaCore 没有
>    任务成功度量指标**。`pick_and_place.py:67-69` 的
>    `check_success() -> None`（字面上返回 None），
>    `eval_policy.py:126-156` 只打印 action/state 统计。**这比任何
>    sim2real 改进都更紧迫 —— 没有成功度量指标，所有 sim2real 工作
>    都无法验证是否生效。** 详见新加的 §1.3。
>
> Codex 还特别指出：所有 effort 估算都是 unprofiled，违反了
> `MEMORY.md` 中 "Perf Estimation Discipline — always profile first"
> 的警示。下文保留原估算作为反面教材，但第五节的优先级表格已整体
> 替换为 Codex 修订版。

---

## 零、先看一个 Bimo 无关、但被这次学习暴露出的独立风险

**训练用 pi0，部署用 ACT —— 两条路从来没接通。**

- 训练脚本：`software/sim-server/scripts/train_pi0_openpi.py`（openpi / JAX / pi0）
- 部署脚本：`software/sim-server/test_policy_station.py:78-84`
  ```python
  from lerobot.policies.act.modeling_act import ACTPolicy
  model_id = "CursedRock17/so101_block_grab_act"
  policy = ACTPolicy.from_pretrained(model_id)
  ```
  **硬编码 HuggingFace 上别人的 ACT 预训练模型**，与本地 pi0 训练产物
  完全无关。

Week 1 memory 说"pi0 training 雏形完成"，但 pi0→真机的链路**其实不
存在**。Bimo 学习揭示的核心道理——训练与部署必须共享 obs schema——
在 NormaCore 这里是更严重的问题：**训练与部署用的是不同的 policy
框架**。

### Codex 验证发现的更深层问题：训练侧已经坏了

原文只讨论了"部署侧加载了错的 policy"。Codex 查了 pi0 训练流程，
**发现训练侧的数据契约本身就不匹配 openpi 的期望**：

- `software/sim-server/scripts/train_pi0_openpi.py:39-64` 把图像
  repack 成 `images.cam_high`，**但 openpi 模型期望的是 `image.*`
  + `image_mask` 结构**
  （`.venv-openpi/lib/python3.12/site-packages/openpi/transforms.py:185-191`,
  `openpi/models/model.py:109-129`）
- 参考正确的 adapter 实现：
  `.venv-openpi/lib/python3.12/site-packages/openpi/policies/aloha_policy.py:42-76`
- NormaCore 的 `train_pi0_openpi.py` **完全没有写 robot-specific
  data transform**

**含义**：即使把 Station 侧的 policy loader 从 ACT 换成 pi0，加载
进来的 pi0 checkpoint 也会因为 data transform 缺失而 forward 失败。
"第 0 优先级"不是换 loader，而是**修复 pi0 的整个 data contract**。

### 建议（修订后）

1. **决策先行**：pi0 是要真的部署，还是只做训练实验？
2. 如果真要部署，按以下顺序修：
   a. **先修训练侧的 data transform** —— 参考
      `openpi/policies/aloha_policy.py` 写 NormaCore 的
      `norma_policy.py`，把 `images.cam_high` 重映射为 openpi 期望
      的 `image.*` + `image_mask` 结构
   b. 然后改 `test_policy_station.py:78-85` 的 policy loader ——
      从硬编码 ACT 改为 config-driven
   c. 最后加 **training→deployment contract 测试** —— 训练脚本产出
      的 checkpoint 必须能被部署脚本加载并跑一次 forward，否则 CI
      挂掉
3. 中间格式选型（仅在决定部署时才考虑）：
   - 选项 A：ONNX。Bimo 的做法，runtime 只要 4 行 `onnxruntime` 调用。
     pi0 的 VLA 架构可能太重，JAX→ONNX 链路复杂
   - 选项 B：`PolicyProtocol`。给 JAX pi0 套一层协议，让 Station
     侧按协议加载。不解耦框架，但保留 pi0 的全部能力

**修正后的工作量估算**：原 1-3 天 → **3-6 天**（Codex）。
原估算没有考虑 data transform 这一层的工作。

这是 Bimo 学习之外的独立风险，但正是读 `api_example.py`（bimo 只用
50 行 onnxruntime 完成部署）才让这个问题显形。

---

## 一、直接引入（Introduce）

### 1.1 物理层域随机化 — 工作量 ~~2-3 天~~ **4-7 天**，价值极高

> **Codex 修正**：下方"直接写 body_mass 就能改"是错的。需要刷新
> 衍生字段。详见下一段的 warning。

**现状**：`SimToRealAdapter`（`software/sim-server/norma_sim/sim_to_real.py:160-215`）
只包装 obs/action，**碰不到物理参数**。训练时机器人 mass、friction、
armature 都是同一组值。

**背景**：`MuJoCoWorld.model` 是公开属性
（`software/sim-server/norma_sim/world/model.py:24-28`），可以访问
`model.body_mass` / `model.geom_friction` / `model.dof_armature`，
但直接写**不够**。

> ⚠️ **Codex 验证发现的 MuJoCo 刷新语义问题**
>
> MuJoCo 3.3.1 有 `mj_setConst` API（`mujoco.h:226-227`），用于
> 在编辑 model 常量后刷新衍生字段。`mjModel` 内部存储的衍生字段
> 包括 `body_subtreemass`、`body_invweight0`、`dof_invweight0`、
> `dof_M0`（`mjmodel.h:724-727, 771-775, 797`），这些在改 mass
> 或 armature 之后需要重算。
>
> 仓库里已有的 sysid 代码证明这一点 ——
> `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/sysid/mujoco_sim.py:121-128, 148-149, 192-193`
> 的模式是"mutate fields → reset data → simulate"，不是"write and go"。
>
> 因此 `PhysicsRandomizer.randomize_on_reset()` 的正确实现路径：
> ```python
> def randomize_on_reset(self, model, data):
>     # 1. 写 model 常量
>     model.body_mass[...] *= ...
>     model.geom_friction[...] = ...
>     # 2. 刷新衍生字段
>     mujoco.mj_setConst(model, data)  # 或 mj_resetData + mj_forward
>     # 3. 然后才能安全 step
> ```
> Codex 还指出：对 `geom_friction` 的 live mutation 是否总是需要
> 刷新，**静态阅读无法证明**，需要运行时实证。这意味着实施时要留
> 时间做"改一类参数 → 跑 N 步 → 对比未改参数的行为差异"这种实证
> 测试，而不是信任 docs。

**Bimo 对应**（`IsaacLab/bimo/bimo_task_env.py`）：
- Link mass 每次 reset 按 scale ±5%（`events.randomize_link_mass`）
- 足底材料每次 reset 采样（static_friction 0.4-0.9, restitution 0-0.05，
  见 `_setup_scene` 200-218 行）
- 24 档电压 torque 轮转（2.7-2.94 Nm，见 `BimoEnv.__init__` 145-152 行）
- Head 周期性推力扰动（2-4 秒一次，±0.2 m/s，见 `events.periodic_push`）

### 建议实现

新建 `software/sim-server/norma_sim/physics_randomizer.py`，挂到
`FastSim.reset()`：

```python
class PhysicsRandomizer:
    def __init__(self, config: PhysicsRandomizerConfig, rng: np.random.Generator):
        ...

    def randomize_on_reset(self, model, data):
        # Per-reset: mass scale, friction sample, armature scale, torque level
        for i in self._body_ids:
            model.body_mass[i] *= self.rng.uniform(0.95, 1.05)
        for i in self._geom_ids:
            model.geom_friction[i, 0] = self.rng.uniform(0.4, 0.9)
        ...
```

与 `SimToRealAdapter` **并列但不替代**：
- `SimToRealAdapter` 管 obs/action 层（现有，不动）
- `PhysicsRandomizer` 管物理层（新增）

这样 `SimToRealConfig.mild()/aggressive()` 的语义可以扩展到物理层。

### Pick 任务的参数优先级

主要成本在调参不在代码。对 pick 任务，建议优先级：

1. **物体 mass / friction** —— 抓取阻力直接来自这里
2. **桌面 friction** —— 滑动与翻倒
3. **机械臂 armature** —— 响应动态，影响轨迹平滑度
4. **link mass** —— 次要，臂 inertia 影响轨迹但不决定成功率

---

### 1.2 Backlash 死区模型 — 工作量 ~~半天~~ **0.5-1.5 天** + 标定

> **Codex 修正**：表述从"直接可迁移"改为"硬件兼容起点"。

**硬件事实**：NormaCore 的 elrobot follower + SO-101 用的都是 STS-3215
舵机（与 Bimo 同一型号）：
- `hardware/elrobot/README.md:46-50`
- `hardware/elrobot/simulation/vendor/therobotstudio/SO101/joints_properties.xml:2-10`
- `so101_new_calib.xml:51,65,77,91,102,115`（joint 用 class `sts3215`）

Backlash 客观存在。**但注意**：vendor 树里已经定义了 backlash class
但**尚未在模型上启用**
（`vendor/therobotstudio/VENDOR.md:53-55`,
`joints_properties.xml:7-10`）。这意味着：
- 代码迁移成本是"小"（Bimo 的 15 行 tensor 逻辑直接可用）
- 但实施前需要先搞清楚为什么 vendor 把这个 class 禁用了 —— 可能
  是因为参数还没标定，也可能是因为有别的已知问题

**Bimo 实现**（`IsaacLab/bimo/bimo_task_env.py::_pre_physics_step`，
267-294 行），核心只有 ~15 行：

```python
delta = self.cmd_actions - self.gear_position
direction = torch.sign(delta)
direction_changed = (direction != self.last_direction) & (self.last_direction != 0)
movement = torch.where(
    direction_changed,
    torch.clamp(torch.abs(delta) - self.cfg.backlash, min=0) * direction,
    delta,
)
self.gear_position += movement
self.last_direction = torch.where(delta != 0, direction, self.last_direction)
```

**关键设计**：只在方向反转时吃死区（`direction_changed` 为真时），
直行时完全无影响。这是教科书做法，不是用"每步恒定噪声"来模拟
backlash（那是错的，会在直行时也降低精度）。

### 建议实现

两个挂载点可选：
- **方案 A**：放进新建的 `PhysicsRandomizer` 作为 `always-on` 模块
  （和 per-reset 随机化共用配置）
- **方案 B**：作为 `SimToRealAdapter.step()` 的一个状态变量
  （需要新增 `_prev_direction` / `_gear_position` 跟踪）

推荐 **A**，因为 backlash 本质是物理量（不是 obs 降级），和
mass/friction 同层。

### 前置工作

标定一次 elrobot 实机的 backlash 量 —— 手动反向转动关节，观察
position feedback 的死区宽度（应该在 1-3° 之间）。之后写进 config。

---

### 1.3 任务成功度量指标 —— 工作量 1-3 天，价值极高（Codex 发现的遗漏）

> 本节由 Codex 独立验证时指出，是原文最严重的遗漏。

**现状**：NormaCore **没有任何任务成功度量指标**：

- `software/sim-server/norma_sim/tasks/pick_and_place.py:67-69` ——
  `check_success() -> None`（方法签名就是返回 None）
- `software/sim-server/scripts/eval_policy.py:126-156` —— eval 脚本
  只打印 action/state 统计信息，**完全没有成功率计算**

**含义**：你无法回答"这个策略比上一个策略好吗？"，也无法回答"加了
domain randomization 之后真机表现更好了吗？"。**所有 sim2real 工作
本质上都在盲目推进，因为没有测量目标。**

**Bimo 对应**（`bimo_task_env.py:335-353`）：

```python
def _get_dones(self):
    truncated = self.episode_length_buf >= self.max_episode_length - 1
    head_heights = self.bimo.data.root_pos_w[:, 2]
    height_termination = head_heights < 0.1
    euler_angles = quaternion_to_euler(self.bimo.data.root_quat_w)
    orientation_termination = (abs(euler_angles[:, 0]) > 0.95) | (abs(euler_angles[:, 1]) > 0.95)
    terminated = height_termination | orientation_termination
    return terminated, truncated
```

虽然这是 RL termination 条件（bimo 的 success = 没有 fall），但**同
样的结构可以直接用于 manipulation**：

- `check_success()` 应返回 bool，基于物体是否抵达目标位置、是否在
  gripper 内、gripper 是否闭合等
- `eval_policy.py` 应汇总多 episode 的 success rate，而不是打印统计

**建议实现**：

1. 在 `pick_and_place.py` 里实现 `check_success()`：
   ```python
   def check_success(self) -> bool:
       object_pos = self.get_body_pos(self.config.object_name)
       target_pos = self.config.target_pose[:3]
       distance = np.linalg.norm(object_pos - target_pos)
       grasped = self.gripper_closed and self.object_in_gripper()
       return distance < self.config.success_threshold and grasped
   ```

2. 在 `eval_policy.py` 的 rollout 循环末尾调用 `task.check_success()`，
   累加到 `successes` 计数，最终打印 `success_rate = successes / n_episodes`。

3. 加 CI 测试：scripted demonstration 上 success_rate 必须 = 1.0，
   防止度量指标本身有 bug。

**工作量**：Codex 估算 1-3 天 unprofiled。实际成本可能取决于 pick
任务的 success 判定有多复杂（物体 pose、grasp stability、target
tolerance 都需要定义）。

**为什么是新的首要任务**：下文所有建议（物理 DR、obs schema、backlash
模型）都以"能测量效果"为前提。没有这个度量指标，连"哪条建议真的有
用"都无法判断 —— 不应该在没有 instrument 的情况下 tune sim2real。

---

#### 1.3.1 实施记录（2026-04-15）

本节已**基本实施完毕**。修改的文件（all in
`software/sim-server/`）：

| 文件 | 改动 |
|---|---|
| `norma_sim/fast_sim.py` | 加 `tracked_objects: list[str]` 参数，`__init__` 里用 `mj_name2id(mjOBJ_BODY, name)` 解析为 body id，`_build_obs` 里导出 `obs["object.<name>.pos"]` 和 `obs["object.<name>.quat"]`（从 `data.xpos`/`data.xquat`） |
| `norma_sim/lerobot_helpers.py::sim_obs_to_lerobot` | 加 `object.*` 键透传到 LeRobot flat dict（和 `camera.*` 并列） |
| `norma_sim/lerobot_robot.py::NormaSimRobotConfig` | 加 `tracked_objects: list[str]` 字段，在 `_connect_fast()` 传给 FastSim |
| `norma_sim/tasks/pick_and_place.py` | 加 `object_body_name="cube"` / `object_initial_pos=(0.20, 0.0, 0.025)` / `success_horizontal_displacement=0.03` / `success_min_height=-0.05` 四个 config 字段；实现 `check_success(obs) -> bool` 使用这些 config |
| `scripts/eval_policy.py` | 实例化 `PickAndPlace`、传 `tracked_objects=[task.object_body_name]`、`run_episode` 跟踪 `peak_object_z` 和 `initial_object_z`、`print_summary` 同时汇报 **loose** 和 **strict** 两个 success rate |

所有改动通过 `ast.parse` 语法检查。FastSim 构造时 `cube` body 成功
解析为 body_id=9，obs 透传到最终 LeRobot dict —— infrastructure
验证通过。

#### 1.3.2 验证过程中的意外重大发现：Scripted demo 从不抓 cube

跑 50 个 seed 的 scripted pick-and-place demo（复制 `gen_dataset_v21.py`
的 `generate_waypoints` 和 `ACTION_NOISE=0.02`），测到：

```
success_rate (loose, horizontal displacement > 3cm): 30/50 = 60%
peak_z (over all 50 episodes):
  min  = 0.025000 m
  max  = 0.025000 m
  mean = 0.025000 m    ← 六位小数精确等于初始 z
lifted > initial + 5mm: 0/50
```

**Cube 在所有 50 个 episode 里从未离开桌面**。peak_z 在六位小数下
精确等于初始 0.025m。

**含义**：
1. Task description: `"pick up the red cube and place it to the side"`
2. 实际物理行为：机械臂从未真正**抓起** cube。在 ~60% 的 episode 里，
   它**刮擦/推动** cube 侧向 3-6 cm；在另外 40% 里，连擦都没擦到。
3. **pi0 训练数据实际内容 ≠ 任务描述**。50 个标为"pick and place"的
   demonstration 里，有 30 个是"push sideways"，20 个是"miss"，
   **0 个是真正的 pick**。
4. 这是比 Codex 发现的 pi0 data transform contract 问题**更深层**的
   issue —— 即使 data contract 修好，demonstration 本身也无法教
   pi0 真正去 pick。

**loose vs strict success metric 设计**：因为发现了这个问题，
`eval_policy.py` 的汇报实现为**两个指标并列**：

- **Loose**：cube 水平位移 > 3cm AND not fallen — 原 `check_success`
  返回的值。当前基线 60%。
- **Strict**：loose 条件 AND `peak_object_z > initial_object_z + 5mm`
  在 episode 过程中曾发生过 —— 需要 rollout 过程中持续 tracking
  `peak_z`，`check_success` 的 stateless obs-only 协议做不了，由
  `run_episode` 在循环里记录。当前基线 **0%**。
- 当 loose > 0% 但 strict = 0% 时，`print_summary` 明确打印：
  `"WARNING: loose-success episodes never lifted the object — arm
  is pushing, not picking"`。

这两个指标并列是**诚实的 instrument** —— 不粉饰数据、不隐藏"推 vs
抓"的差别，让问题持续可见直到 trajectory 被真正修好。

#### 1.3.3 新的 v2 前置工作：修 scripted trajectory（已诊断，待 mjviser 调参）

上述发现意味着 `§0 pi0 data contract` 修复之前，还有一个**更紧迫
的前置问题**：**`pick_and_place` 的 scripted trajectory 必须真正
能抓起 cube**。已在 2026-04-15 session 中深度诊断，找到两层根因。

##### 根因 Layer 1：PD 追踪延迟

FastSim 的 `physics_hz=500, action_hz=30` 意味着每个
`sim.step()` 调用 = 16 个 physics steps = 32ms。实测发现
**关节从一个 waypoint 的 target 到完全 settle 到新 waypoint
的 target，需要 ~60 个 `sim.step()` 调用**（~2 秒）。

原 trajectory 每个 waypoint 只有 20-40 步 —— **arm 永远追不上
waypoint，永远在"路上"**。测量：在 "grasp" waypoint 开始时，
关节实际位置和目标位置相差最大 **8cm** 的 EE 距离。

**修法**：要么每个 waypoint 加 60+ 步 settle hold，要么降低
`physics_hz`（但影响接触物理精度）。建议前者。

##### 根因 Layer 2：Gripper body 与 cube 碰撞（更严重）

即使用 FK 找到**精确**的 HOVER / GRASP 关节配置
（`HOVER=[0, -0.80, 0.75, 1.20, 0]` → EE (0.200, 0, 0.078)；
`GRASP=[0, -0.55, 1.00, 0.80, 0]` → EE (0.201, 0, 0.020)），并加
60 步 settle，**cube 仍然在下降过程中被 gripper body 推走**：

```
phase           ee                   cube                 peak_z
hover settled   (0.200, 0, 0.078)    (0.200, 0, 0.020)    0.025
descent end     (0.220, 0, 0.037)    (0.205, 0, 0.018)    0.025
settle step 30  (0.218, 0, 0.035)    (0.193, 0, 0.019)    0.025
settle step 40  (0.201, 0, 0.020)    (0.164, 0, 0.019)    0.025  ← cube 已跑
settle step 50  (0.201, 0, 0.020)    (0.156, 0, 0.020)    0.025  ← 推到 44mm 外
grasp (close)   (0.201, 0, 0.020)    (0.156, 0, 0.020)    0.025  ← 夹空气
```

**推断**：`gripper` body 或其子 geom 在下降过程中与 cube 发生
碰撞。Gripper 的 `moving_jaw_so101_v1` body 在 GRASP 位置时
位于 (0.205, 0.018, 0.100)，即 x 方向对齐但 z 比 cube 高 8cm。
所以碰撞不是来自 moving_jaw，而是来自其他 gripper 子 geom
（可能是固定 jaw 或 wrist 连接件）。

**修法需要视觉工具**：
- 用 `experiments/mjviser/`（已 clone）启动 web viewer，加载
  `hardware/elrobot/simulation/vendor/therobotstudio/SO101/scene_tabletop.xml`
- 手动 slider 移到 HOVER 配置，**肉眼确认 gripper 几何体相对
  cube 的位置**
- 找出哪个 geom 正在和 cube 碰撞，改用其他 approach angle
  （可能需要调 `wrist_flex` 让 gripper 朝下而不是朝前，
  或改用非纯垂直的 approach 轨迹）
- 可能还需要检查 `wrist_roll` 让 jaw 横向而不是纵向
- 调好后把 joint config 写回 `pick_and_place.py`

##### 诊断阶段已固化的代码资产

本次 session 的诊断代码已经证明了以下事实（如果 mjviser session
结果和这些矛盾，应该先重新验证）：

- `mj_name2id(mjOBJ_BODY, "cube")` → body id = 9，`data.xpos[9]`
  即 cube 实时位置
- `mj_name2id(mjOBJ_SITE, "gripperframe")` → site id = 1，
  `data.site_xpos[1]` 即 gripper tip 实时位置
- SO101 的 6 个 joint qposadr 依次是 0, 1, 2, 3, 4, 5
  （shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
  wrist_roll, gripper），qposadr[6] 是 cube 的 free joint (7-dof)
- Gripper actuator ctrlrange = [-0.175, +1.745]，command 0 = 全开，
  command 1 = 全闭
- 在 HOVER 和 GRASP 之间纯 FK（不跑物理）给出的 EE 位置与
  MuJoCo forward kinematics 严格一致 —— 问题不在 FK 而在动力学

##### 估算修正

- 原估算 **1-2 天** → 诊断后 **0.5-1 天（在有 mjviser 的前提下）**
- 没有 mjviser 则工作量爆炸为 3-5 天（盲调）
- 此条目**必须在 §1.1 物理 DR 之前**完成，因为没有能真正 pick
  的 demonstration，整个 sim2real 方向都是假的

**下一个 session 的起点**：启动 mjviser → 加载 scene_tabletop.xml
→ 手动拖 slider 到 `[0, -0.80, 0.75, 1.20, 0]` (HOVER) →
用 group 切换看碰撞 geom → 迭代直到视觉确认 jaws 夹住 cube →
把新 config 写回 `norma_sim/tasks/pick_and_place.py`。

---

## 二、重构（Refactor）

### 2.1 Obs 与 state/action 向量构造的多点漂移 — 工作量 ~~1-2 天~~ **2-4 天**

> **Codex 修正**：原文说的"三处独立构造"是错的。`lerobot_helpers`
> 不是独立构造器，是两个 backend 共用的 adapter。**真正的漂移
> 在脚本层面的手写向量构造**，覆盖面比原估计大。

**修正后的现状**：

**raw obs 构造**（两处，当前基本一致）：

| 位置 | 角色 | 输出 |
|---|---|---|
| `fast_sim.py:123-165` `_build_obs` | raw sim obs | `{"joints", "gripper", "camera.<name>"}` |
| `gym_env.py:354-400` `_snapshot_to_obs` | IPC → obs | 同 keys，从 WorldSnapshot 构造 |

两者在 SO101 场景下大致对齐（都输出 float64 `joints/gripper` + HWC
`uint8` camera 图像），但存在**非 SO101 能力类型的潜在漂移**：
`gym_env.py:377-380` 用 ctrlrange 重新归一化 gripper，
`SnapshotBuilder` 已经把 qpos 转成了 capability-space `position_value`
（`world/snapshot.py:52-57`），`GRIPPER_PARALLEL` 的归一化在
`world/capabilities.py:89-110`。SO101 目前避开了这个问题只是因为
它的 manifest 没有 annotations，自动走 `REVOLUTE_POSITION`
（`manifest.py:127-132,197-200,230-237`）。

**raw-obs → LeRobot adapter**（共用，不是独立构造）：

- `lerobot_helpers.py:84-101` `sim_obs_to_lerobot` 和 `:113-129,160-175`
  的 `build_state_vector` / action 转换
- 两个 backend 都通过 `lerobot_robot.py:126-127,140-141,196-199` 调用
  这个 adapter

**真正的漂移在下面 4 个脚本里手写 state/action 向量构造**：

| 脚本 | 行 | 做什么 |
|---|---|---|
| `scripts/gen_dataset_v21.py` | 99-105 | 数据生成时手写 state/action 向量，**没有用 `build_state_vector` helper** |
| `scripts/run_experiment.py` | 135-143 | 实验 runner 手写相同逻辑 |
| `scripts/record_scripted_demo.py` | 145-156 | 手写 demonstration 记录 |
| `scripts/batch_generate.py` | 172-175 | 批量生成再写一遍 |

这些脚本每一个都是"从 sim_obs dict 按自己的方式构造 state/action
tensor"，和 `lerobot_helpers.build_state_vector` 的逻辑**平行但未
调用**。任何一处改了关节顺序或维度定义，四处都要同步 —— 但目前没有
任何机制保证同步。

**Bimo 对比**：obs 结构在 `_get_observations`（sim）和
`process_observations`（runtime）两处定义，但作者用 **44 维固定布局
做了硬约束** —— 任何一侧漂移都会立刻在 inference 维度对不上时 crash。
NormaCore 目前完全没有这种强制机制，**且漂移面比 Bimo 广得多**（6+
处，不是 2 处）。

### 建议实现

1. 新建 `software/sim-server/norma_sim/obs_schema.py`：

   ```python
   @dataclass(frozen=True)
   class RobotObservationSchema:
       joint_names: tuple[str, ...]
       gripper_names: tuple[str, ...]
       camera_specs: dict[str, tuple[int, int, int]]  # name → (H, W, C)

       def validate(self, obs: dict) -> None:
           """Raise if obs doesn't conform."""
           ...
   ```

2. 三个构造器全部 **return 符合 schema 的 dict**。`sim_obs_to_lerobot`
   变成"按 schema 字段映射"，而不是"按 key 猜字段"。

3. 加一个 pytest：`test_obs_schema_consistency.py`，跨三条路径的
   obs 必须通过 `schema.validate()` + 跨路径 `assert_equal_shape`。

本质是类型化现有 dict，不改行为。收益：从此 obs 漂移在 CI 挂掉，
不靠代码审查。

---

## 三、增强（Enhance）

### 3.1 ~~act_hist 启动鲁棒性~~ Pi0 起点扰动覆盖度验证 — 工作量 0.5-1 天（**降级**）

> **Codex 修正 —— 本节包含本文最严重的两个错误**：
>
> 1. **pi0 不吃 action history**。原文说 "pi0 是 VLA，没有显式 action
>    history（只看图像+语言+prior action）"—— 这是错的。pi0 的输入是
>    **images + state + prompt**（`openpi/models/pi0_config.py:63-86`,
>    `openpi/models/model.py:55-68, 83-100`, `openpi/transforms.py:252-266`），
>    没有 prior action。`action_sequence_keys` 是用来生成 target action
>    chunks 的（`openpi/training/config.py:85-88`,
>    `openpi/training/data_loader.py:143-148`,
>    `openpi_client/action_chunk_broker.py:10-49`），**不是**把历史动作
>    喂回模型。
>
> 2. **起点扰动已经实现**了：
>    - `scripts/gen_dataset_v21.py:34-50, 83-100` —— 已有 `home`
>      随机化逻辑
>    - `software/sim-server/norma_sim/tasks/pick_and_place.py:23-31, 44-55` ——
>      通过 `home_noise_std` 实现同样的效果
>
>    所以原文说的"最小改动最具体回报"指向的是一个**已经在代码里的
>    功能**。
>
> 本节因此**从"紧急新增"降级为"验证覆盖度"**。下面的正文已改写。

**Bimo 的技巧**（仅作参考，不是要引入）：`bimo_task_env.py:398-401`
在 reset 时故意用非归一化原始度数填充 `act_hist`，让策略学会从任意
history 启动。这对 Bimo 有效是因为 Bimo 的 obs **包含 4 步 action
history**，RL 策略需要对起点 history 鲁棒。

**对 NormaCore 的实际意义**：pi0 不吃 action history，所以这个技巧
本身不适用。但同一思想的另一个应用 —— **episode 起点姿态扰动** ——
在 NormaCore 已经实现：

- `gen_dataset_v21.py:34-50` 和 `:83-100` 有 `home` 随机化
- `tasks/pick_and_place.py:23-31` 和 `:44-55` 有 `home_noise_std`

### 修订后的建议

不是"加新代码"，而是"验证已有的起点扰动覆盖了所有数据生成路径"：

1. 跑一次 `gen_dataset_v21.py` 的代码路径审查，确认每个 episode 的
   起点姿态**确实**被随机化
2. 对比 old task path (`gen_dataset_v21.py:34-50`) 和 new task path
   (`pick_and_place.py:23-31`) 的 noise_std 值，确认一致
3. 在 eval script 里加一个断言：检测 100 个 episode 的 initial joint
   values 方差 > 预期阈值（确保扰动没被意外关闭）

**工作量**：0.5-1 天，全部是验证工作，不是新代码。

**为什么保留这一节**：删掉它会丢失一个反面教材 —— **我在写原文时
没有 grep 代码就假设某个功能不存在**。这正是 `MEMORY.md` 里
"always profile first" 原则的教训：**先验证现状再提建议**。

---

### 3.2 ONNX 部署路径作为备选轻量 runtime — 仅记录

**现状**：`test_policy_station.py` 180 行，用 LeRobot + torch +
station_py。

**Bimo 对比**：`api_example.py` 115 行，只依赖 `onnxruntime` +
`pyserial` + `numpy` + `opencv`。

**何时有价值**：边缘设备（NUC / Jetson）部署时，torch + LeRobot 太重。
**现在不建议做**，但记录这个选项。前置条件：
- pi0 → ONNX 转换链路存在（JAX 侧需要 `jax2tf` 或等效）
- 或者切到 ACT 之后用 LeRobot 的 `policy.export_onnx()`

---

## 四、方法论层（值得写入 CLAUDE.md 的原则）

### 4.1 DR 分层意识

Bimo 的域随机化有**清晰的时间尺度分层**：

| 层级 | 时机 | Bimo 实现 | NormaCore 现状 |
|---|---|---|---|
| Per-step | 每物理步 | orient noise, actuator noise, action delay | ✓ SimToRealAdapter._degrade |
| Per-reset | 每 episode | mass, friction, torque level | **缺失** |
| Per-interval | 周期触发 | 2-4 秒 body push | **缺失** |
| Always-on | 持续偏置 | backlash 死区 | **缺失** |

NormaCore 当前的 DR 集中在 per-step 和一点点 per-reset（calibration
offset），**完全没有 per-interval 和 always-on 类别**。建议 1.1（物理层
DR）本质补全 per-reset；建议 1.2（backlash）本质补全 always-on。

这个分层法可以作为 `norma_sim/sim_to_real.py` 的文档化框架，让未来
加 DR 的人有清晰的归位。

### 4.2 训练-部署契约是代码强制，不是代码审查

Bimo 把"obs 格式两侧一致"做到两处硬编码的 44 维布局—— **代码强制，
维度不对立刻 crash**。NormaCore 目前完全靠命名约定 + 代码审查 + 信任。
建议 2.1（obs schema 重构）的**本质就是把这个约定变成代码强制**。

推广到"零"的发现—— 训练/部署的 policy class 一致性也应该是代码强制。
如果有一个 `PolicyProtocol` + load-time assert，
`test_policy_station.py` 不会悄悄漂回 HuggingFace ACT。

### 4.3 动作增量 vs 绝对位置

Bimo 用 `cmd += clip(action, -3, 3) * 4/3`—— 每步最多动 4° 的增量。这是
**RL-friendly** 设计：让 action space 平滑、step-to-step bounded、探索
过程不会抽搐。

pi0 / ACT 用绝对关节位置—— **imitation-friendly** 设计：和
demonstration 数据天然对齐，teacher forcing 直接用 target。

**不建议改**，但需要知道这个设计分岔的存在。如果未来 NormaCore 想在
pi0 之上加 RL fine-tuning 层（SFT → RLHF-for-robots），delta action
是起点，需要 wrapper 把绝对位置转成相对增量。

---

## 五、优先级综合（Codex 修订版）

> **修订要点**：
> 1. 新增 #1.3 task success metric 为第二优先级 —— 没有这个度量，
>    其他所有 sim2real 工作都无法验证
> 2. #0 pi0 契约扩展为 "端到端修复"，不是"只换 loader"，工作量翻倍
> 3. #3.1 从第二优先级**整体移除**，降级为"验证覆盖度"
> 4. 所有 effort 估算上调，并显式标记为 "unprofiled estimate"

| # | 事项 | 类别 | 工作量（unprofiled） | 价值 | 时机 |
|---|---|---|---|---|---|
| 1.3 | ~~补 task success metric~~ **已实施 + 发现 trajectory 本身是坏的** | 引入 | 已完成 | **极高** | **2026-04-15 完成** |
| **1.3.3** | **修 scripted pick trajectory**（demo 从未真正抓起 cube，peak_z 恒等于初始 z） | 根因修复 | **1-2 天** | **极高 — 阻塞其他所有 sim2real 工作** | **立即** |
| 0 | **修复 pi0 data contract 端到端**（training transform + deployment loader + CI 契约测试） | 架构 | **3-6 天** | **极高** | 与 1.3.3 并列（两个是不同 layer 的问题） |
| 2.1 | Obs / state / action 向量构造统一化（**覆盖脚本手写构造**） | 重构 | **2-4 天** | 高 | #0 和 #1.3.3 完成后 |
| 1.1 | 物理层 DR（mass / friction / armature，带 `mj_setConst` 刷新） | 引入 | **4-7 天** | 高 | 在 #1.3.3 有真正的 demo 之后 |
| 1.2 | Backlash 死区模型 | 引入 | **0.5-1.5 天** + 标定 | 中 | 和 1.1 一起 |
| 3.1 | ~~gen_dataset 起点扰动~~ **已存在，仅验证覆盖度** | 验证 | **0 天新代码 + 0.5-1 天验证** | 低 | 任何时候 |
| 3.2 | ONNX 轻量部署 | 增强 | N/A | 低 | 边缘设备上线时 |
| 4.1 | DR 分层原则入 CLAUDE.md | 方法论 | 半小时 | 中 | 完成 1.1 后 |
| 4.2 | Policy 契约强制机制 | 方法论 | 和 #0 合并 | 高 | 和 #0 一起 |

**最紧迫三项（修订后）**：

1. **#0** — **修复 pi0 data contract 端到端**。不只换 loader，训练侧
   的 data transform 就是坏的。
2. **#1.3** — **补 task success metric**。这是原文完全遗漏的项，
   Codex 独立验证时发现。**没有这个，其他 sim2real 改进都是盲目
   tuning** —— 你无法判断"加了物理 DR 之后真机表现更好"还是"更差"。
3. **#2.1** — Obs / state / action 向量构造统一化，范围比原估计大。

**不再是最紧迫的项**：
- ~~#3.1 gen_dataset 起点扰动~~ —— 已经实现了。
- **#1.1 物理 DR** 从"下周做"降到"在 #1.3 就绪之后"—— 因为没有 success
  metric 就无法验证物理 DR 是否真的有效。

**Codex 关于估算纪律的批评**：所有数字都标为 "unprofiled estimate"，
对应 `MEMORY.md` 中 `feedback_perf_estimation.md` 的 "always profile
first" 原则。原文在没有 success metric 的情况下给每条建议打"价值"
标签本身就是不科学的 —— **价值 = 实验可以测量的效果**，没有度量
指标就没有价值可言。

---

## 六、未被采纳的 Bimo 设计（避免将来有人重新提）

以下 Bimo 设计**有意不引入**，记录原因供未来参考：

| Bimo 设计 | 不引入原因 |
|---|---|
| Isaac Lab 训练框架 | NormaCore 已经押注 MuJoCo + pi0（imitation），切换 RL 意味着数据生成路径推倒重来，ROI 不成立 |
| RSL-RL PPO 栈 | 同上，且 pi0 已有成熟训练流程 |
| Delta action (`cmd += clip * 4/3`) | pi0 是 imitation learning，数据是绝对位置。改动需要重写 data pipeline |
| Reward shaping (7 项加权) | imitation learning 无 reward，完全不适用 |
| 44 维固定 obs 布局 | pi0 是 VLA，obs 是图像+文本+状态，不是固定维向量 |
| MCU 固件层 | NormaCore 走 station + IPC 架构，不管 MCU |
| `lock_heading()` yaw 归零 | 机械臂任务不涉及 base orientation |
| Bimo.usd 模型格式 | NormaCore 用 MJCF，USD 只和 Isaac Sim 相关 |
| `routines.py` 时间插值动作 | NormaCore 用 `sim_worlds/*.yaml` 声明式 pose，功能对等，不需要重复 |

---

## 七、Codex 独立验证报告（2026-04-15）

本文档的每一条事实声明都经过 Codex（codex-rescue 子 agent）独立
验证。Codex 读了原版 insights.md 全文，交叉验证了所有 file:line
citation，并独立读取 NormaCore + openpi + MuJoCo 源码得出结论。

**任务元数据**：
- Session ID: `019d8d6c-db1d-7e21-9e7e-c2bf04d78468`
- 总 tokens: 6.77M（cache hit 92%）
- Context window usage: 52% (136K / 258K)
- Rate limit usage: 主窗口 1%, 周窗口 26%

### 5 条事实声明的验证结果

所有声明均被标记为 "partially correct" —— 方向正确但有细节错误。

| # | 声明 | 结论 | 关键纠正 |
|---|---|---|---|
| 1 | pi0/Station 部署断层 | partially correct | 不只部署坏了，**训练侧 data transform 就已坏** —— `train_pi0_openpi.py:39-64` 的 `images.cam_high` 不符合 `openpi/transforms.py:185-191` 期望的 `image.*` + `image_mask` 结构 |
| 2 | Obs 三处独立构造 | partially correct | `lerobot_helpers.py` 是共用 adapter 不是独立构造器。**真正的漂移**在 `gen_dataset_v21.py:99-105`、`run_experiment.py:135-143`、`record_scripted_demo.py:145-156`、`batch_generate.py:172-175` 的**脚本级手写向量构造** |
| 3 | MuJoCo 物理参数直写生效 | partially correct | MuJoCo 3.3.1 有 `mj_setConst` API (`mujoco.h:226-227`)，需要刷新 `body_subtreemass` / `body_invweight0` / `dof_invweight0` / `dof_M0` 等衍生字段。仓库里的 `mujoco_sim.py:121-128, 148-149, 192-193` 证明要"mutate → reset data → simulate" |
| 4 | STS-3215 backlash 直接可迁移 | partially correct | 同型号正确，但 vendor 的 backlash class **定义了但未启用**（`VENDOR.md:53-55`, `joints_properties.xml:7-10`）。表述应改为"硬件兼容起点" |
| 5 | act_hist 启动扰动对 pi0 有用 | partially correct | (a) pi0 **不吃 action history** —— 输入是 images + state + prompt (`openpi/models/pi0_config.py:63-86`)；(b) 起点扰动**已经实现**在 `gen_dataset_v21.py:34-50` 和 `pick_and_place.py:23-31, 44-55` |

### Codex 发现的关键遗漏（原文零覆盖）

**NormaCore 没有任务成功度量指标** —— 这是 Codex 独立 critique 阶段
发现的最重要问题：

- `software/sim-server/norma_sim/tasks/pick_and_place.py:67-69` ——
  `check_success() -> None`（方法体就是 return None）
- `software/sim-server/scripts/eval_policy.py:126-156` —— eval 只
  打印 action/state 统计，**没有成功率计算**
- `software/sim-server/test_policy_eval.py:32-38, 129-131` —— 同样
  没有 success 判定

**含义**：本文原版在没有可测量 success 指标的情况下给每条建议打
"价值"、"紧迫度"、"ROI" —— 这从方法论上就是不科学的。没有
instrument 不应该 tune 任何东西。

已作为新的 §1.3 加入，与 #0 并列为最高优先级。

### Codex 发现的其他可迁移 Bimo 设计（原文漏掉）

原文第六节列了 9 条"不采纳"的 Bimo 设计。Codex 复审后指出以下几条
实际上**可以迁移**，原文分析过度保守：

1. **显式 termination / success 契约**（`bimo_task_env.py:335-353`）
   —— 直接对应新的 §1.3，Bimo 虽然是 locomotion 任务但 `done` 结构
   可以直接用于 manipulation。Unprofiled estimate: 1-3 天。

2. **共享 runtime preprocessor / observation ABI**（
   `bimo_task_env.py:228-265` + `api_example.py:31-50, 80-111`）——
   Bimo 的 sim obs builder 和 runtime example 共用同一份 history
   buffer 契约。NormaCore 的 pi0 路径**没有对等的共享 adapter**，
   这是 #0 的更深层含义。Unprofiled estimate: 2-4 天。

3. **时间尺度分层的 perturbation 配置**（
   `bimo_task_env.py:87-115, 267-303`）—— Bimo 清晰地分 reset-time /
   interval / always-on 三类。NormaCore 的
   `experiment.py:70-113, 153-175` 有配置面但没有对等的物理 reset /
   interval 逻辑入口。Unprofiled estimate: 1-2 天 config plumbing，
   标定成本另计。

4. **确认不可迁移**（原文第六节的判断 Codex 同意）：
   - Bimo 的 7 项 reward shaping (`bimo_task_env.py:40-46, 304-333`)
   - 44 维固定 obs 向量 —— pi0 是 images + state + prompt，不是固定
     维度向量

### Effort 估算纪律（Codex 明确批评）

Codex 对照 `MEMORY.md` 中 `feedback_perf_estimation.md` 的 "always
profile first, apply Amdahl's law before estimating speedups" 原则，
明确指出：

> "Inference: all numbers below are unprofiled estimates. The
> document assigns urgency/value without a measurable success signal;
> current task/eval code lacks one."

所有修正后的估算：

| 项 | 原估算 | Codex 修正 | 修正原因 |
|---|---|---|---|
| #0 | 1-3 天 | **3-6 天** | 不只换 loader，训练侧 transform 也需重写 |
| #1.1 | 2-3 天 | **4-7 天** | 需要 MuJoCo `mj_setConst` 刷新语义验证 |
| #1.2 | 0.5 天 + 标定 | **0.5-1.5 天** + 标定 | 缺乏跨 backend 的 stateful backlash 注入器 |
| #1.3 | （新增） | **1-3 天** | success 判定可能需要定义物体 pose / grasp stability / target tolerance |
| #2.1 | 1-2 天 | **2-4 天** | 覆盖面扩大到 6+ 处，不是原以为的 3 处 |
| #3.1 | 0.5 天 | **0 天新代码 + 0.5-1 天验证** | 已经存在 |

### 方法论教训

Codex 的 critique 显示本文原版的几个系统性错误：

1. **没先验证现状就提建议** —— #3.1 的"最小改动最具体回报"指向了
   已经存在的功能。**教训**：提建议前应该 `grep` 一遍现有代码。

2. **对外部库 API 的语义假设不足** —— #1.1 默认"`model.body_mass[i] *=
   x` 就生效"，没有查 MuJoCo docs 或现有 sysid 代码。**教训**：
   引用库函数时要读至少一个现有调用点。

3. **没先建 instrument 就提优化** —— 整个文档在没有 success metric
   的前提下排优先级。**教训**：没有 benchmark 不应 tune。

这些教训本身可能值得作为 feedback 存入 MEMORY.md。

---

## 附录：关键文件索引

### Bimo 侧

- `bimo-ref/IsaacLab/bimo/bimo_task_env.py` — DR 和 reward 全部实现
- `bimo-ref/IsaacLab/bimo/bimo_config.py` — ArticulationCfg + DCMotorCfg
- `bimo-ref/IsaacLab/bimo/agents/rsl_rl.py` — PPO 超参数
- `bimo-ref/BimoAPI/bimo/bimo.py` — MCU serial + 相机封装
- `bimo-ref/BimoAPI/examples/api_example.py` — ONNX 推理循环（~115 行）

### NormaCore 侧 — 原文引用

- `software/sim-server/norma_sim/sim_to_real.py` — 现有 adapter（225 行）
- `software/sim-server/norma_sim/fast_sim.py:123-165` — `_build_obs`
- `software/sim-server/norma_sim/gym_env.py:354-400` — `_snapshot_to_obs`
- `software/sim-server/norma_sim/lerobot_helpers.py:84-101` — `sim_obs_to_lerobot`（共用 adapter，非独立构造）
- `software/sim-server/norma_sim/world/model.py:24-28` — MuJoCoWorld
- `software/sim-server/norma_sim/world/capabilities.py:89-110` — GRIPPER_PARALLEL 归一化
- `software/sim-server/scripts/gen_dataset_v21.py` — 数据生成入口
- `software/sim-server/scripts/train_pi0_openpi.py` — pi0 训练入口
- `software/sim-server/test_policy_station.py:78-84,219-221` — 硬编码 ACT 的部署脚本
- `software/sim-server/scripts/eval_policy.py:42-56,97-103,126-156` — sim 评估脚本

### Codex 验证新引用的文件

- `software/sim-server/scripts/gen_dataset_v21.py:34-50,83-100,99-105` —— 起点扰动已实现 + 手写 state/action 向量
- `software/sim-server/scripts/run_experiment.py:135-143,226-250` —— 手写向量构造 + ACT-specific runtime
- `software/sim-server/scripts/record_scripted_demo.py:145-156` —— 手写 state/action 构造
- `software/sim-server/scripts/batch_generate.py:172-175` —— 同上
- `software/sim-server/test_policy_eval.py:32-38,129-131` —— 无 success 判定
- `software/sim-server/norma_sim/tasks/pick_and_place.py:23-31,44-55,67-69` —— home_noise_std 已实现 + `check_success() -> None`
- `software/sim-server/norma_sim/world/snapshot.py:52-57` —— capability-space position_value
- `software/sim-server/norma_sim/world/manifest.py:127-132,197-200,230-237` —— SO101 走 REVOLUTE_POSITION
- `software/sim-server/experiments/pick_v1_pi0.yaml:27-34` —— pi0 训练 config
- `software/sim-server/norma_sim/experiment.py:70-113,153-175` —— experiment config surface
- `software/sim-server/norma_sim/lerobot_helpers.py:113-129,160-175` —— `build_state_vector` helper (未被脚本调用)
- `software/sim-server/norma_sim/lerobot_robot.py:97-141,165-178,196-199` —— backend 选择 + adapter 调用点
- `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/sysid/mujoco_sim.py:121-128,148-149,192-193` —— 已有的正确 mutate→reset→simulate 模式
- `hardware/elrobot/simulation/vendor/therobotstudio/VENDOR.md:17-23,31-34,53-55` —— backlash class 定义但未启用
- `hardware/elrobot/simulation/vendor/therobotstudio/SO101/joints_properties.xml:2-10,7-10` —— STS3215 + backlash 未启用
- `hardware/elrobot/simulation/vendor/therobotstudio/SO101/so101_new_calib.xml:51,65,77,91,102,115` —— joint class sts3215
- `hardware/elrobot/README.md:46-50` —— STS(S)3215 舵机声明
- `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml:3-6` —— follower MJCF
- `hardware/elrobot/simulation/vendor/therobotstudio/SO101/README.md:20-23` —— SO101 文档
- `hardware/elrobot/simulation/manifests/norma/therobotstudio_so101.scene.yaml:10-13` —— SO101 manifest

### openpi / MuJoCo 外部库

- `.venv-openpi/lib/python3.12/site-packages/openpi/models/pi0_config.py:63-86` —— pi0 输入定义
- `.venv-openpi/lib/python3.12/site-packages/openpi/models/model.py:55-68,83-100,109-129` —— ModelInputs 结构
- `.venv-openpi/lib/python3.12/site-packages/openpi/models/pi0.py:216-279` —— pi0 forward
- `.venv-openpi/lib/python3.12/site-packages/openpi/policies/policy.py:68-95` —— 推理接口
- `.venv-openpi/lib/python3.12/site-packages/openpi/policies/aloha_policy.py:42-76` —— 正确的 robot-specific transform 参考
- `.venv-openpi/lib/python3.12/site-packages/openpi/transforms.py:185-191,252-266` —— image.* 期望 + state + prompt transforms
- `.venv-openpi/lib/python3.12/site-packages/openpi/training/config.py:85-88` —— action_sequence_keys
- `.venv-openpi/lib/python3.12/site-packages/openpi/training/data_loader.py:143-148` —— 数据加载
- `.venv-openpi/lib/python3.12/site-packages/openpi_client/action_chunk_broker.py:10-49` —— action chunking，非 history input
- `/home/yuan/proj/robomotion/.venv/lib/python3.12/site-packages/mujoco/include/mujoco/mujoco.h:226-227` —— `mj_setConst` API
- `/home/yuan/proj/robomotion/.venv/lib/python3.12/site-packages/mujoco/include/mujoco/mjmodel.h:724-727,771-775,797` —— 衍生字段 (`body_subtreemass`, `body_invweight0`, `dof_invweight0`, `dof_M0`)
