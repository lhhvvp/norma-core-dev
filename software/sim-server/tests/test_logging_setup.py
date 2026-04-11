"""Smoke tests for logging_setup.JsonFormatter."""
import io
import json
import logging

from norma_sim.logging_setup import JsonFormatter, configure_logging


def _capture_log(record_fn) -> str:
    """Attach a JsonFormatter handler to a fresh logger, invoke
    record_fn(logger), and return the raw emitted string."""
    logger = logging.getLogger("norma_sim.test")
    logger.handlers.clear()
    logger.setLevel(logging.DEBUG)
    buf = io.StringIO()
    handler = logging.StreamHandler(buf)
    handler.setFormatter(JsonFormatter())
    logger.addHandler(handler)
    logger.propagate = False
    try:
        record_fn(logger)
    finally:
        logger.removeHandler(handler)
    return buf.getvalue()


def test_json_formatter_emits_expected_fields():
    line = _capture_log(lambda lg: lg.info("hello world")).strip()
    parsed = json.loads(line)
    assert parsed["level"] == "INFO"
    assert parsed["component"] == "norma_sim.test"
    assert parsed["msg"] == "hello world"
    assert "ts" in parsed
    # ISO-8601 with microseconds, UTC
    assert parsed["ts"].endswith("+00:00") or "Z" in parsed["ts"]


def test_json_formatter_includes_exception():
    def emit(lg):
        try:
            raise RuntimeError("boom")
        except RuntimeError:
            lg.exception("caught")

    line = _capture_log(emit).strip()
    parsed = json.loads(line)
    assert parsed["level"] == "ERROR"
    assert parsed["msg"] == "caught"
    assert "exc" in parsed
    assert "RuntimeError" in parsed["exc"]
    assert "boom" in parsed["exc"]


def test_json_formatter_merges_extra_fields():
    def emit(lg):
        lg.info("with extras", extra={"extra_fields": {"world_tick": 42, "robot_id": "a"}})

    line = _capture_log(emit).strip()
    parsed = json.loads(line)
    assert parsed["world_tick"] == 42
    assert parsed["robot_id"] == "a"


def test_configure_logging_is_idempotent():
    # Calling twice should not duplicate handlers on the root logger.
    configure_logging("DEBUG")
    n_first = len(logging.getLogger().handlers)
    configure_logging("DEBUG")
    n_second = len(logging.getLogger().handlers)
    assert n_first == n_second == 1
