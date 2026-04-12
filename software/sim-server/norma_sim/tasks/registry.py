"""Task registry — lookup tasks by name."""
from __future__ import annotations

from .pick_and_place import PickAndPlace

# Add new tasks here
REGISTRY: dict[str, object] = {
    "pick_and_place": PickAndPlace(),
}


def get_task(name: str):
    """Get a task by name. Raises KeyError if not found."""
    if name not in REGISTRY:
        available = ", ".join(REGISTRY.keys())
        raise KeyError(f"Unknown task: {name!r}. Available: {available}")
    return REGISTRY[name]
