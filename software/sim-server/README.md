# norma-sim

Python simulation server for NormaCore. Launched as a subprocess by the
Rust `sim-runtime` crate's `ChildProcessBackend` (SimMode::Internal) or
run standalone via `python -m norma_sim ...` (SimMode::External).

Zero references to any legacy servo-driver crate name — the
capability-keyed schema from `protobufs/sim/world.proto` is the only
vocabulary used inside.

## Installation

From the repo root (no `pip install -e` — Ubuntu 24.04 PEP 668 blocks it
and all dependencies are already in system site-packages):

```bash
PYTHONPATH=software/sim-server python3 -c "import norma_sim; print('OK')"
```

For testing:

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/
```

## Running standalone (Scenario B)

```bash
PYTHONPATH=software/sim-server python3 -m norma_sim \
    --manifest hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml \
    --socket /tmp/norma-sim-dev.sock \
    --physics-hz 500 \
    --publish-hz 100
```

## Manual browser smoke test (MVP-2)

MVP-2 has two complementary smoke tests. Both run from the repo root
after Rust + Python artifacts are built.

### Prerequisites

- `rustup stable` toolchain + `cargo` on PATH
- `nasm` + `clang` 18+ (for `turbojpeg-sys` + `norm-uvc-sys` bindgen)
- `python3 -c "import mujoco, numpy, yaml, pytest"` all succeed
- `software/station/clients/station-viewer/dist/` exists
  (`cd software/station/clients/station-viewer && yarn install && yarn build`
  if absent — required for `rust-embed`'s `Asset::get` to compile)
- `cargo build -p station` succeeds

### Phase 1 baseline — Menagerie walking skeleton

Validates that the station infrastructure works with any MuJoCo-valid
MJCF (hypothesis A: "infra is robot-agnostic"). Run periodically after
any change to station / sim-runtime / bridge code — it is the permanent
regression fixture for assumption A.

```bash
PYTHONPATH=software/sim-server ./target/debug/station \
    -c software/station/bin/station/station-sim-menagerie.yaml \
    --web 0.0.0.0:8889
```

**⚠ `PYTHONPATH` is load-bearing.** Without it, the `python3 -m norma_sim`
subprocess silently fails to find its module and station hangs at
`"Starting sim-runtime (mode=Internal, startup_timeout_ms=5000)"` with
no error message — the 5s handshake just times out.

- [ ] Browser at `http://localhost:8889` shows Menagerie SO-ARM100's
  **6 motors** (Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw)
- [ ] Dragging any slider smoothly rotates the corresponding joint in 3D
- [ ] `pytest software/sim-server/tests/integration/test_menagerie_walking_skeleton.py`
  passes (6 tests)

### Phase 2 — ElRobot full 8-motor demo

The MVP-2 exit criterion. Validates that ElRobot's sim env is usable for
future policy training (spec §3.2 Ceiling).

```bash
PYTHONPATH=software/sim-server ./target/debug/station \
    -c software/station/bin/station/station-sim.yaml \
    --web 0.0.0.0:8889
```

Expected startup log lines:

```
Starting sim-runtime (mode=Internal, startup_timeout_ms=5000)
sim-runtime started: elrobot_follower
st3215_compat_bridge bridge started: robot_id=elrobot_follower legacy_bus_serial=sim://bus0 motors=8
WebSocket server listening on 0.0.0.0:8889
```

- [ ] Browser connects, shows 8 motors (M1-M8) populated on `sim://bus0`
- [ ] Switch control source dropdown → **(Web-controlled)**
- [ ] Drag **M1 (Shoulder Pitch)** slowly through full range
  - smooth response, no oscillation, no jitter
  - arm holds position when slider released (no droop)
  - this was the MVP-1 regression — Phase 2's Menagerie physics fork fixes it
- [ ] Repeat for M2 (Shoulder Roll), M3 (Shoulder Yaw), M4 (Elbow)
- [ ] Repeat for M5 (Wrist Roll), M6 (Wrist Pitch), M7 (Wrist Yaw)
- [ ] Drag **M8 (Gripper)** slider 0 → 1 → 0
  - primary joint + both mimic jaws (`rev_motor_08_1`, `rev_motor_08_2`)
    open/close in sync via the `<tendon>` + `<equality>` block (P0 invariant)
  - no NaN artifacts (jaws don't disappear or fly apart)
- [ ] Multi-motor test: drag M1 + M4 + M8 simultaneously, no interference
- [ ] Kill station (Ctrl+C), restart, verify arm returns to home pose

### Side-by-side visual comparison (Ceiling §3.2 item 8, advisory)

In separate terminals:

```bash
python3 -m mujoco.viewer hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml
python3 -m mujoco.viewer hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml
```

- [ ] ElRobot response quality ≈ Menagerie SO-ARM100 quality
  (no obvious "ours is worse" artifacts — no extra jitter, no weird
  motions, no obviously-wrong physics)

If ElRobot looks materially worse, record the discrepancy but do not
block — Floor §3.1 acceptance tests
(`pytest software/sim-server/tests/integration/test_elrobot_acceptance.py`)
are the hard gate for MVP-2.

### Troubleshooting

- **Station hangs at `"Starting sim-runtime"` with no further output** →
  `PYTHONPATH=software/sim-server` not set. The `python3 -m norma_sim`
  subprocess can't find its module and fails silently; station waits out
  the 5s startup handshake then exits.
- **`cargo build -p station` fails with `Asset::get not found`** →
  `software/station/clients/station-viewer/dist/` missing. Run
  `cd software/station/clients/station-viewer && yarn install && yarn build`.
- **Web UI shows "connect a robot" empty state** → bridge's
  `register_queue` not firing (MVP-1 regression guard — commit `84ca47a`).
  Check station logs for `st3215_compat_bridge bridge started`.
- **Gripper sliders move but the jaws don't** → tendon equality regression
  on the P0 gripper mimic. Run
  `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py -v`
  and inspect the `<tendon>` + `<equality>` blocks in
  `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml`.

## Architecture

See `docs/superpowers/specs/2026-04-10-simulation-integration-design.md`
for the full v2 spec. The short summary:

- **sim-runtime** (Rust) — hosts `SimulationRuntime`, owns a
  pluggable `WorldBackend`, exposes `subscribe_snapshots() /
  send_actuation() / subscribe_health()` over a crate-public API.
- **norma_sim** (this package) — pluggable Python `WorldBackend`,
  speaks the generic `norma_sim.world.v1` proto over a UDS bound to
  `$NORMA_SIM_SOCKET_PATH`.
- **st3215-compat-bridge** (Rust) — subscribes to the legacy global
  `commands` queue, translates `StcSt3215Command` bytes into generic
  `ActuationCommand`s, and packs sim-runtime snapshots back into
  71-byte `st3215/inference` payloads that legacy clients cannot
  distinguish from real hardware.

The hard architecture invariants (`make check-arch-invariants`) are
what keep these layers from leaking into each other.
