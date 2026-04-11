"""JSON structured logging (spec §12.3).

All log records emitted by norma_sim should be single-line JSON on
stderr so the Rust `ChildProcessBackend` can redirect them to a log
file and downstream tooling can parse them without heuristics.
"""
from __future__ import annotations

import json
import logging
import sys
from datetime import datetime, timezone
from typing import Any


class JsonFormatter(logging.Formatter):
    """Emit a one-line JSON object per log record.

    Standard fields: ts, level, component, msg. Any extra dict
    attached via `logging.LogRecord.extra_fields` is merged at the
    top level. If the record carries exception info, a serialised
    `exc` field is appended.
    """

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "ts": datetime.now(timezone.utc).isoformat(timespec="microseconds"),
            "level": record.levelname,
            "component": record.name,
            "msg": record.getMessage(),
        }
        extras = getattr(record, "extra_fields", None)
        if isinstance(extras, dict):
            payload.update(extras)
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False, default=str)


def configure_logging(level: str = "INFO") -> None:
    """Install a single stderr handler with JsonFormatter as the root
    handler. Clears any previously installed handlers so CLI entry
    points can call this idempotently.
    """
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(JsonFormatter())
    root = logging.getLogger()
    for h in list(root.handlers):
        root.removeHandler(h)
    root.addHandler(handler)
    root.setLevel(level)
