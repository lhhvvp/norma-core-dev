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
    --manifest hardware/elrobot/simulation/elrobot_follower.scene.yaml \
    --socket /tmp/norma-sim-dev.sock \
    --physics-hz 500 \
    --publish-hz 100
```

## Scenario A smoke-test checklist (manual)

The full end-to-end acceptance for Chunk 8's P0 demo. Run from the repo
root after all Rust + Python artifacts are built.

1. **Prerequisites**
   - `rustup stable` toolchain installed + `cargo` on PATH
   - `nasm` + `clang` (for `turbojpeg-sys` + `norm-uvc-sys` bindgen)
   - `python3 -c "import mujoco, numpy, yaml, pytest"` all succeed
   - `software/station/clients/station-viewer/dist/` exists
     (`cd software/station/clients/station-viewer && yarn install &&
     yarn build` if absent)

2. **Generated artifacts**
   ```bash
   make protobuf           # regenerates all proto bindings
   make regen-mjcf         # regenerates elrobot_follower.xml
   ```
   Both must exit 0.

3. **Build**
   ```bash
   cargo build -p station
   ```
   Must compile cleanly. Warnings OK.

4. **Launch Scenario A**
   ```bash
   ./target/debug/station -c software/station/bin/station/station-sim.yaml
   ```
   Expected log lines on stderr:
   ```
   Starting sim-runtime (mode=Internal, startup_timeout_ms=5000)
   sim-runtime started: elrobot_follower_empty
   Drivers started
   st3215_compat bridge started
   ```

5. **Web UI**
   - Open `http://localhost:8889` in a browser
   - ElRobot follower 3D model loads
   - The model renders in a neutral pose (all joints at 0)

6. **★ P0 gripper demo**
   - Drag the "Motor 1" slider → Joint_01 rotates in the 3D view
   - Drag the "Motor 8" (Gripper) slider →
     Gripper_Jaw_01 and Gripper_Jaw_02 **open/close** via the MJCF
     tendon-based `<equality>` polycoef constraints from Chunk 1
     (commit `a76b2fe`)

7. **Clean shutdown**
   - Ctrl+C in the Station terminal
   - Expected final log: `norma_sim shut down cleanly`
   - `ls /tmp/norma-sim*` → no matches (TempRuntimeDir was cleaned up)

If any step fails, see the chunk-specific troubleshooting notes below.

### Troubleshooting

- **Step 3 fails with "station-viewer/dist does not exist"** → run
  step 1's `yarn install && yarn build`.
- **Step 4 fails with "socket did not appear within 5000ms"** → the
  Python subprocess crashed during startup. Look for the
  `sim-backend.log` the config's `log-capture: File` creates, or
  re-run with `--log-capture inherit` to see stderr live.
- **Step 5 web UI shows "no connection"** → Station's web server is
  on 8889 by default but the CLI flag `--web` must be passed; check
  `station --help`.
- **Step 6 gripper sliders move but the jaws don't** → regression on
  the tendon equality; run the pytest P0 demo
  `pytest software/sim-server/tests/world/test_mimic_gripper.py -v`
  and investigate `hardware/elrobot/simulation/worlds/gen.py` Phase
  2c if it fails.

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
