"""Proto import shim.

The gremlin-generated Python bindings live at
``target/gen_python/protobuf/sim/world.py`` (produced by
``make protobuf``) and import ``shared.gremlin_py.gremlin`` at the top.
Both paths are relative to the repo root, which is NOT on
``PYTHONPATH`` when norma_sim is consumed as an installed package.

This shim:
  1. Resolves the repo root by walking up from this file.
  2. Inserts it at the front of ``sys.path`` (no-op if already there).
  3. Imports the generated module as ``world_pb``.

Every other norma_sim module that needs proto types imports from
here instead of touching ``sys.path`` directly, keeping the mess in
a single location. If the build pipeline later switches from
gremlin_py to google.protobuf bindings, only this file changes.
"""
from __future__ import annotations

import sys
from pathlib import Path

_repo_root = Path(__file__).resolve().parents[4]
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

# noqa: E402 — intentional post-sys.path import.
from target.gen_python.protobuf.sim import world as world_pb  # noqa: E402

__all__ = ["world_pb"]
