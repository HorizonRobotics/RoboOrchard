"""Shared JSON calibration-store I/O."""

from __future__ import annotations
import json
import os
import stat
import tempfile
from pathlib import Path
from typing import Any


def load_store(path: str | os.PathLike[str]) -> dict[str, Any]:
    """Load a calibration store, returning an empty store if absent."""
    store_path = Path(path)
    if not store_path.exists():
        return {}
    with store_path.open() as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"calibration store {store_path} must be an object")
    return value


def write_store(path: str | os.PathLike[str], store: dict[str, Any]) -> None:
    """Atomically replace a calibration store with formatted JSON."""
    store_path = Path(path)
    store_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        existing = store_path.stat()
    except FileNotFoundError:
        existing = None
    fd, temporary = tempfile.mkstemp(
        dir=store_path.parent,
        prefix=f".{store_path.name}.",
        suffix=".tmp",
    )
    try:
        if existing is not None:
            os.fchmod(fd, stat.S_IMODE(existing.st_mode))
            current = os.fstat(fd)
            if (current.st_uid, current.st_gid) != (
                existing.st_uid,
                existing.st_gid,
            ):
                os.fchown(fd, existing.st_uid, existing.st_gid)
        with os.fdopen(fd, "w") as stream:
            json.dump(store, stream, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, store_path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
