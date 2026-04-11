# MVP-2 Menagerie → ElRobot Parameter Comparison Table

| | |
|---|---|
| **日期** | 2026-04-12 |
| **状态** | Phase 2 research spike 产物 |
| **Menagerie source** | mujoco_menagerie/trs_so_arm100 @ commit `c771fb04055d805f20db0eab6cb20b67555887d0` |
| **Target** | hardware/elrobot/simulation/elrobot_follower.xml |

## 拓扑对照

### Menagerie SO-ARM100（5 revolute + 1 gripper，**共 6 actuators**）

All joints inherit `class="so_arm100"` defaults: `frictionloss="0.1"`, `armature="0.1"`.
Actuator default: `kp="50"`, `dampratio="1"`, `forcerange="-3.5 3.5"`.
Note: `dampratio="1"` is **not** an explicit `damping` value — MuJoCo computes implicit
damping as `2 * sqrt(kp * effective_inertia)` per actuator; there is no `<joint damping="…">`
attribute in the XML. The damping is purely actuator-side via `dampratio`.

| Menagerie joint | 类型 | axis (body frame) | range (rad) | armature | frictionloss | actuator kp | actuator forcerange | 备注 |
|---|---|---|---|---|---|---|---|---|
| Rotation | hinge | `0 1 0` (Y) | `-1.92 1.92` | 0.1 | 0.1 | 50 | `-3.5 3.5` | Base yaw; class `Rotation` |
| Pitch | hinge | `1 0 0` (X) | `-3.32 0.174` | 0.1 | 0.1 | 50 | `-3.5 3.5` | Shoulder pitch; class `Pitch` |
| Elbow | hinge | `1 0 0` (X) | `-0.174 3.14` | 0.1 | 0.1 | 50 | `-3.5 3.5` | Elbow pitch; class `Elbow` |
| Wrist_Pitch | hinge | `1 0 0` (X) | `-1.66 1.66` | 0.1 | 0.1 | 50 | `-3.5 3.5` | Wrist pitch; class `Wrist_Pitch` |
| Wrist_Roll | hinge | `0 1 0` (Y) | `-2.79 2.79` | 0.1 | 0.1 | 50 | `-3.5 3.5` | Wrist roll; class `Wrist_Roll` |
| Jaw | hinge | `0 0 1` (Z) | `-0.174 1.75` | 0.1 | 0.1 | 50 | `-3.5 3.5` | Single-DOF revolute gripper; class `Jaw`; **no tendon / equality / weld / mimic** |

**Gripper confirmation:** Menagerie's gripper is a plain `<joint name="Jaw">` with a `<position>` actuator.
No `<tendon>`, `<equality>`, or mimic structure exists anywhere in so_arm100.xml.

**`<option>` block:** `<option cone="elliptic" impratio="10"/>`. No explicit `timestep`, `integrator`,
`solver`, `iterations`, or `tolerance` — these all use MuJoCo defaults
(timestep=0.002, integrator=Euler, solver=Newton, iterations=100, tolerance=1e-8).

---

### ElRobot (7 revolute + 1 gripper = 8 actuators)

URDF joint axes are expressed in parent-link frames. Signs of ~-X vs +X do not change functional
analogy — they reflect local-frame conventions, not a different rotation axis.

The 8 revolute joints in kinematic order:

| ElRobot joint | URDF child link | axis (URDF, parent frame) | axis ≈ | range (rad) | Menagerie analog | armature | frictionloss | actuator kp | actuator forcerange | 来源 |
|---|---|---|---|---|---|---|---|---|---|---|
| rev_motor_01 | Joint_01_1 | `0.0 -0.009 0.9999` | ≈ +Z | `-1.5509 1.5509` | Rotation | 0.1 | 0.1 | 50 | `-3.5 3.5` | menagerie direct (base yaw; functionally analogous; axis convention differs: Menagerie uses Y, ElRobot uses Z due to different base frame orientation) |
| rev_motor_02 | Joint_02_1 | `-0.9999 -0.000 0.0087` | ≈ -X | `-1.6122 1.6122` | **无对应** | 0.1 | 0.1 | 50 | `-3.5 3.5` | nearest-neighbor (Pitch): same shoulder zone, X-axis family |
| rev_motor_03 | Joint_03_v1_1 | `-0.9998 -0.000 0.0175` | ≈ -X | `-1.7610 1.7610` | Pitch | 0.1 | 0.1 | 50 | `-3.5 3.5` | menagerie direct (shoulder pitch; same X-axis) |
| rev_motor_04 | Joint_04_v1_1 | `-0.9998 0.0086 0.0175` | ≈ -X | `-1.7533 1.7533` | Elbow | 0.1 | 0.1 | 50 | `-3.5 3.5` | menagerie direct (elbow pitch; same X-axis) |
| rev_motor_05 | Joint_05_v1_1 | `-0.0086 -0.9999 0.000` | ≈ -Y | `-3.1907 2.6998` | **无对应** | 0.1 | 0.1 | 50 | `-3.5 3.5` | nearest-neighbor (Wrist_Pitch): first joint in wrist zone, X-axis family in wrist region |
| rev_motor_06 | Joint_06_v1_1 | `-0.9997 0.0173 0.0175` | ≈ -X | `-1.3775 1.7641` | Wrist_Pitch | 0.1 | 0.1 | 50 | `-3.5 3.5` | menagerie direct (wrist pitch; X-axis, wrist zone) |
| rev_motor_07 | Gripper_Base_v1_1 | `-0.0171 -0.9998 0.0090` | ≈ -Y | `-3.0710 2.7796` | Wrist_Roll | 0.1 | 0.1 | 50 | `-3.5 3.5` | menagerie direct (wrist roll; Y-axis; same zone) |
| rev_motor_08 | Gripper_Gear_v1_1 | `0.0171 0.9998 -0.009` | ≈ +Y | `0.0 2.2028` | Jaw | 0.1 | 0.1 | 50 | `-3.5 3.5` | menagerie direct (gripper motor; axis convention differs: Menagerie Z, ElRobot Y — different local frame; functionally equivalent single gripper drive joint) |

**Gripper mimic joints (NOT mapped to Menagerie actuators):**

| URDF joint | type | child link | axis | range (m) | mimic |
|---|---|---|---|---|---|
| rev_motor_08_1 | prismatic | Gripper_Jaw_02_v1_1 | `-0.9997 0.0173 0.0175` | `-0.0255 0.0` | `rev_motor_08 × -0.0115` |
| rev_motor_08_2 | prismatic | Gripper_Jaw_01_v1_1 | `-0.9997 0.0173 0.0175` | `0.0 0.0255` | `rev_motor_08 × +0.0115` |

These 2 URDF `<mimic>` joints translate to `<tendon><fixed>` + `<equality><tendon>` in the
hand-written MJCF (MVP-1 structure, P0 invariant). They are not actuated independently.

**Independent ElRobot joints (no Menagerie analog): 2** — rev_motor_02, rev_motor_05

*(The plan anticipated 3; the actual count is 2. ElRobot has 8 actuators vs Menagerie's 6 = 2 extra.
The plan's "3 extra" was an off-by-one guess — the gripper difference (tendon vs plain revolute) is
structural, not a count of extra revolute joints.)*

---

## 参数继承策略

1. **`<option>`**: 直接继承 Menagerie 显式值 `cone="elliptic"`, `impratio="10"`. 其余参数
   (timestep, integrator, solver, iterations, tolerance) 不设置，使用 MuJoCo 默认值，与
   Menagerie 行为一致（Menagerie 也未显式设置这些）。

2. **`<default>` classes**:
   - Menagerie 有 classes: `so_arm100` (root), `Rotation`, `Pitch`, `Elbow`, `Wrist_Pitch`,
     `Wrist_Roll`, `Jaw`, `visual`, `motor_visual`, `collision`, `finger_collision`
   - ElRobot 的 MJCF 将为 8 个关节建立对应 classes（可沿用相同命名或用 ElRobot 命名），
     继承 `so_arm100` 的 joint defaults (`frictionloss="0.1"`, `armature="0.1"`) 和
     actuator defaults (`kp="50"`, `dampratio="1"`, `forcerange="-3.5 3.5"`)

3. **Actuator kp/kv/forcerange**: Menagerie 默认值 (`kp="50"`, `dampratio="1"`,
   `forcerange="-3.5 3.5"`) 作为 ElRobot 所有关节的 baseline。若 Phase 2 smoke test 过不了
   Floor 4 step response，允许 per-joint 微调 kp，但 **armature 必须保持 Menagerie 值 0.1**。

4. **Gripper mimic**: Menagerie 使用 plain single-DOF revolute joint `Jaw` + `<position>`
   actuator（在 Chunk 1 + Chunk 4 中已确认，无 tendon / equality / weld）。ElRobot 的 gripper
   结构不同 — 保留 MVP-1 的 `<tendon><fixed>` + `<equality><tendon>` mimic 结构（2 个 mimic
   joints `rev_motor_08_1`, `rev_motor_08_2`，P0 不可破）。**不尝试匹配 Menagerie 的单关节
   gripper — ElRobot 的 2-finger parallel gripper 机械结构需要 tendon mimic。**

---

## 独有关节的估值依据

| ElRobot 关节 | URDF axis | 估值来源 (nearest-neighbor) | 备注 |
|---|---|---|---|
| rev_motor_02 | ≈ -X (parent frame) | **Pitch** | Shoulder zone 中唯一额外关节（shoulder roll）；轴方向 (-X) 与 Menagerie Pitch (+X) 同属 X-axis family，是 shoulder 区最近邻。范围 ±1.61 rad 比 Pitch (-3.32~0.17) 窄，但 physics 参数（armature/frictionloss）直接继承 Pitch。 |
| rev_motor_05 | ≈ -Y (parent frame) | **Wrist_Pitch** | Wrist zone 第一关节（lower arm / elbow-to-wrist transition），轴方向 (-Y) 与 Wrist_Pitch (+X) 不同族，但在此运动链位置无更接近的 Menagerie 关节；Wrist_Roll (+Y) 轴向更近但功能上 rev_motor_06 才是 Wrist_Roll 对应物。选 Wrist_Pitch 作为 nearest-neighbor（同处 wrist 入口，质量/惯量量级相近）。 |

---

## ElRobot URDF 链接与惯性数据（Task 5.2 参考）

| ElRobot joint | child link | mass (kg) | ixx | iyy | izz |
|---|---|---|---|---|---|
| rev_motor_01 | Joint_01_1 | 0.05706 | 2e-05 | 2e-05 | 1e-05 |
| rev_motor_02 | Joint_02_1 | 0.05084 | 2.4e-05 | 3.4e-05 | 1.7e-05 |
| rev_motor_03 | Joint_03_v1_1 | 0.03822 | 1.1e-05 | 1.8e-05 | 1.5e-05 |
| rev_motor_04 | Joint_04_v1_1 | 0.05152 | 3.7e-05 | 2.1e-05 | 4.7e-05 |
| rev_motor_05 | Joint_05_v1_1 | 0.01756 | 3e-06 | 5e-06 | 5e-06 |
| rev_motor_06 | Joint_06_v1_1 | 0.03882 | 1.6e-05 | 1.6e-05 | 2.2e-05 |
| rev_motor_07 | Gripper_Base_v1_1 | 0.06244 | 2.2e-05 | 2.9e-05 | 3.1e-05 |
| rev_motor_08 | Gripper_Gear_v1_1 | 0.0039 | ~0 | ~0 | ~0 |

Motor link masses (ST3215_N_v1_1, all 0.055 kg) are separate URDF links connected by fixed joints;
they will be merged into the parent body in the MJCF (MuJoCo does not support fixed URDF joints
natively — they collapse into the parent body's inertia).

base_link mass: 0.03944 kg (floor-mounted base, modeled as worldbody child).

---

## Risk notes

- **M1 axis convention mismatch**: Menagerie `Rotation` is Y-axis (body frame), ElRobot `rev_motor_01`
  is ~Z-axis (URDF parent frame). Both implement base yaw but in different frame conventions. Physics
  parameters (armature, kp) are still directly inherited — the axis difference only affects the MJCF
  `<joint axis="…">` attribute, not parameter values.

- **M5 axis is -Y, nearest-neighbor is Wrist_Pitch (X)**: Axis family mismatch. If wrist dynamics feel
  under-damped for M5, consider using Wrist_Roll (Y-axis, closer axis direction) as fallback. However
  Wrist_Roll is already the direct analog for M7; using it for both M5 and M7 is acceptable since all
  Menagerie joints share the same armature/frictionloss/kp anyway.

- **All Menagerie joints share identical physics parameters** (armature=0.1, frictionloss=0.1, kp=50,
  forcerange=-3.5 3.5). The nearest-neighbor distinction for M2 and M5 is therefore moot for Phase 2
  — any Menagerie joint gives the same values. If per-joint tuning becomes necessary in Phase 3,
  re-evaluate.

- **Gripper_Gear_v1_1 inertia is ~0**: The gear link has negligible inertia (mass=0.0039 kg,
  all inertia components listed as 0.0 in URDF). MuJoCo may warn about near-singular inertia.
  Apply a small minimum (e.g., diaginertia="1e-6 1e-6 1e-6") in the MJCF.

- **URDF mimic joints are prismatic, not revolute**: rev_motor_08_1 and rev_motor_08_2 are
  `type="prismatic"` in the URDF with `<mimic>`. In the MJCF they become slide joints driven by
  tendon + equality. The multiplier values (-0.0115 and +0.0115, units m/rad) set the gear ratio
  for the `<equality><tendon>` element.

- **Fixed URDF joints (Rigid 1, 3, 7, 9, 11, 13, 15, 17)**: These 8 fixed joints connect ST3215
  motor bodies to adjacent structural links. They carry no DOF and must be collapsed (inertia summed
  into parent) when writing the MJCF. Each ST3215_N_v1_1 link (0.055 kg) must be merged with its
  adjacent joint link or motor body.

- **`<option>` implicit defaults**: Menagerie does not set `timestep`, `integrator`, or `solver`.
  MuJoCo 3.x defaults are timestep=0.002s, integrator=Euler, solver=Newton. If ElRobot's heavier
  joints (M4 elbow 0.052 kg + merged motor 0.055 kg ≈ 0.107 kg) show instability, consider
  explicit `timestep="0.001"` — but do not change unless Phase 2 Floor 4 test fails.

- **Menagerie `kp=50` may be too low for ElRobot's heavier links**: Menagerie's heaviest link is
  Upper_Arm at 0.162 kg. ElRobot's merged elbow link (Joint_04 + ST3215_4 + ST3215_5) is ~0.16 kg —
  comparable. However ElRobot's shoulder (M3/M4 zone with merged motors) may be ~0.22 kg total.
  If Floor 4 step response fails, raise kp for M3/M4 (suggested range: 60-100) while keeping
  armature=0.1.
