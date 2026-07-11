"""Schema and registry for scripted hardware evaluation scenarios."""

from __future__ import annotations
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 1
SCENARIO_DIRECTORY = Path(__file__).resolve().parent / "scenarios"
SUPPORTED_MODES = {"sine", "waypoints", "circle"}


class ScenarioError(ValueError):
    """A scenario definition is missing, malformed, or unsafe."""


def _number(value: Any, field: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ScenarioError(f"{field} must be a number, got {value!r}.")
    result = float(value)
    if minimum is not None and result < minimum:
        raise ScenarioError(f"{field} must be >= {minimum}, got {result}.")
    return result


def _integer(value: Any, field: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ScenarioError(f"{field} must be an integer, got {value!r}.")
    if minimum is not None and value < minimum:
        raise ScenarioError(f"{field} must be >= {minimum}, got {value}.")
    return value


def _waypoints(value: Any, field: str) -> list[list[float]]:
    if not isinstance(value, list) or len(value) < 2:
        raise ScenarioError(f"{field} must contain at least two waypoints.")
    rows: list[list[float]] = []
    for index, row in enumerate(value, start=1):
        if not isinstance(row, list) or len(row) != 7:
            raise ScenarioError(
                f"{field}[{index}] must have 7 values (6 joints + gripper)."
            )
        rows.append([_number(item, f"{field}[{index}]") for item in row])
    return rows


@dataclass(frozen=True)
class Scenario:
    """Validated scenario definition."""

    name: str
    label: str
    description: str
    mode: str
    parameters: dict[str, Any]
    ui: dict[str, Any]
    safety: dict[str, Any]
    tools: dict[str, Any]
    path: Path

    def ros_parameters(self) -> dict[str, Any]:
        """Return executor parameters with ROS-compatible flat arrays."""
        result = {"mode": self.mode}
        for key, value in self.parameters.items():
            if key in {"waypoints_left", "waypoints_right"}:
                result[key] = [item for row in value for item in row]
            else:
                result[key] = value
        if "use_current_state" in self.safety:
            result["use_current_state"] = bool(
                self.safety["use_current_state"]
            )
        return result


def _scenario_directory(directory: str | Path | None) -> Path:
    if directory is None or not str(directory).strip():
        return SCENARIO_DIRECTORY
    return Path(directory).expanduser()


def _validate(payload: Any, path: Path) -> Scenario:
    if not isinstance(payload, dict):
        raise ScenarioError(f"{path} must contain a JSON object.")
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise ScenarioError(
            f"{path} needs schema_version={SCHEMA_VERSION}, got "
            f"{payload.get('schema_version')!r}."
        )
    name = payload.get("name")
    if not isinstance(name, str) or not name:
        raise ScenarioError(f"{path}: name must be a non-empty string.")
    if path.stem != name:
        raise ScenarioError(
            f"{path}: filename stem {path.stem!r} must match name {name!r}."
        )
    label = payload.get("label")
    description = payload.get("description")
    if not isinstance(label, str) or not label:
        raise ScenarioError(f"{path}: label must be a non-empty string.")
    if not isinstance(description, str) or not description:
        raise ScenarioError(f"{path}: description must be a non-empty string.")

    executor = payload.get("executor")
    if not isinstance(executor, dict):
        raise ScenarioError(f"{path}: executor must be an object.")
    mode = executor.get("mode")
    if mode not in SUPPORTED_MODES:
        raise ScenarioError(
            f"{path}: executor.mode must be one of "
            f"{sorted(SUPPORTED_MODES)}, got {mode!r}."
        )
    parameters = executor.get("parameters", {})
    if not isinstance(parameters, dict):
        raise ScenarioError(f"{path}: executor.parameters must be an object.")
    parameters = dict(parameters)

    if mode == "waypoints":
        left = _waypoints(parameters.get("waypoints_left"), "waypoints_left")
        right = _waypoints(
            parameters.get("waypoints_right"), "waypoints_right"
        )
        if len(left) != len(right):
            raise ScenarioError(
                "waypoints_left and waypoints_right must have equal counts."
            )
        parameters["waypoints_left"] = left
        parameters["waypoints_right"] = right
        parameters["peak_velocity"] = _number(
            parameters.get("peak_velocity"), "peak_velocity", minimum=0.05
        )
        parameters["max_accel"] = _number(
            parameters.get("max_accel"), "max_accel", minimum=0.1
        )
        parameters["settle_s"] = _number(
            parameters.get("settle_s", 0.0), "settle_s", minimum=0.0
        )
        indices = parameters.get("pass_through_indices", [])
        if not isinstance(indices, list):
            raise ScenarioError("pass_through_indices must be a list.")
        validated_indices = [
            _integer(item, "pass_through_indices", minimum=1)
            for item in indices
        ]
        if any(item > len(left) for item in validated_indices):
            raise ScenarioError(
                "pass_through_indices contains an index beyond the "
                f"{len(left)} configured waypoints."
            )
        parameters["pass_through_indices"] = validated_indices
    elif mode == "circle":
        for side in ("left", "right"):
            key = f"circle_trajectory_{side}"
            value = parameters.get(key)
            if not isinstance(value, str) or not value:
                raise ScenarioError(f"{path}: {key} must be a path string.")
    else:
        parameters["duration_s"] = _number(
            parameters.get("duration_s"), "duration_s", minimum=0.1
        )
        parameters["amplitude_scale"] = _number(
            parameters.get("amplitude_scale", 1.0),
            "amplitude_scale",
            minimum=0.0,
        )
        parameters["frequency_scale"] = _number(
            parameters.get("frequency_scale", 1.0),
            "frequency_scale",
            minimum=0.0,
        )

    ui = payload.get("ui", {})
    safety = payload.get("safety", {})
    tools = payload.get("tools", {})
    for value, field in ((ui, "ui"), (safety, "safety"), (tools, "tools")):
        if not isinstance(value, dict):
            raise ScenarioError(f"{path}: {field} must be an object.")
    return Scenario(
        name=name,
        label=label,
        description=description,
        mode=mode,
        parameters=parameters,
        ui=dict(ui),
        safety=dict(safety),
        tools=dict(tools),
        path=path,
    )


def load_scenario(
    name_or_path: str | Path,
    directory: str | Path | None = None,
) -> Scenario:
    """Load a scenario by registry name or explicit JSON path."""
    candidate = Path(name_or_path).expanduser()
    if candidate.suffix != ".json" and candidate.parent == Path("."):
        candidate = _scenario_directory(directory) / f"{candidate.name}.json"
    if not candidate.exists():
        raise ScenarioError(f"scenario does not exist: {candidate}")
    try:
        payload = json.loads(candidate.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ScenarioError(
            f"invalid scenario JSON {candidate}: {exc}"
        ) from exc
    return _validate(payload, candidate)


def list_scenarios(
    directory: str | Path | None = None,
) -> list[Scenario]:
    """Return all valid registry scenarios in stable name order."""
    root = _scenario_directory(directory)
    if not root.is_dir():
        raise ScenarioError(f"scenario directory does not exist: {root}")
    scenarios = [load_scenario(path) for path in root.glob("*.json")]
    return sorted(
        scenarios,
        key=lambda scenario: (
            int(scenario.ui.get("order", 1000)),
            scenario.name,
        ),
    )
