"""Common CSV result schema for scripted robot evaluations."""

from __future__ import annotations
import csv
from pathlib import Path
from typing import Sequence

RESULT_SCHEMA_VERSION = 1
JOINT_COUNT = 7


def _joint_fields(prefix: str) -> list[str]:
    return [f"{prefix}{index}" for index in range(1, JOINT_COUNT + 1)]


RESULT_FIELDS = [
    "schema_version",
    "scenario",
    "mode",
    "t",
    "phase",
    "trajectory_t",
    *_joint_fields("left_cmd"),
    *_joint_fields("left_meas"),
    *_joint_fields("right_cmd"),
    *_joint_fields("right_meas"),
]


class ResultWriter:
    """Line-buffered writer that preserves partial hardware runs."""

    def __init__(self, path: str, scenario: str, mode: str) -> None:
        result_path = Path(path).expanduser()
        result_path.parent.mkdir(parents=True, exist_ok=True)
        self.path = result_path
        self.scenario = scenario
        self.mode = mode
        self._file = result_path.open("w", newline="", buffering=1)
        self._writer = csv.DictWriter(self._file, fieldnames=RESULT_FIELDS)
        self._writer.writeheader()

    @staticmethod
    def _values(values: Sequence[float] | None) -> list[float | str]:
        result = [float(value) for value in list(values or [])[:JOINT_COUNT]]
        return result + [""] * (JOINT_COUNT - len(result))

    def write(
        self,
        elapsed: float,
        *,
        left_command: Sequence[float] | None,
        left_measured: Sequence[float] | None,
        right_command: Sequence[float] | None,
        right_measured: Sequence[float] | None,
        phase: str = "trajectory",
        trajectory_time: float | None = None,
    ) -> None:
        row = {
            "schema_version": RESULT_SCHEMA_VERSION,
            "scenario": self.scenario,
            "mode": self.mode,
            "t": f"{float(elapsed):.6f}",
            "phase": phase,
            "trajectory_t": (
                ""
                if trajectory_time is None
                else f"{float(trajectory_time):.6f}"
            ),
        }
        for prefix, values in (
            ("left_cmd", left_command),
            ("left_meas", left_measured),
            ("right_cmd", right_command),
            ("right_meas", right_measured),
        ):
            row.update(
                zip(
                    _joint_fields(prefix),
                    self._values(values),
                    strict=True,
                )
            )
        self._writer.writerow(row)

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
