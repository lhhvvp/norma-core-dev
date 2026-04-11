# norma-sim

Python simulation server for NormaCore. Launched as a subprocess by the
Rust `sim-runtime` crate's `ChildProcessBackend` (or run standalone via
`python -m norma_sim ...` for Scenario B external mode).

Zero references to any legacy servo-driver crate — the capability-keyed
schema is the only vocabulary used inside.

Installation (from repo root):

```bash
pip install -e software/sim-server/
python -c "import norma_sim; print('OK')"
```

See `docs/superpowers/specs/2026-04-10-simulation-integration-design.md`
for the full design. A smoke-test checklist lands in Chunk 8.
