#!/usr/bin/env python3
"""Load shared filesystem defaults for calibration tools."""

from __future__ import annotations
from pathlib import Path

DEFAULT_URDF = (
    (Path(__file__).resolve().parents[1] / "configs" / "piper_urdf_path.txt")
    .read_text()
    .strip()
)
