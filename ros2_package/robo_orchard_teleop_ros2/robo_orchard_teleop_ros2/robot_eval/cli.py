"""Single command-line interface for scripted robot evaluations."""

from __future__ import annotations
import argparse
import datetime
import json
import os
import shlex
from pathlib import Path

from robo_orchard_teleop_ros2.robot_eval.schema import (
    Scenario,
    ScenarioError,
    list_scenarios,
    load_scenario,
)


def _repo_file(relative: str) -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / relative
        if candidate.exists():
            return candidate
    raise ScenarioError(
        f"cannot locate {relative!r} from installed module {here}; "
        "run from a source checkout mounted at /opt/roboorchard."
    )


def _executor_command() -> list[str]:
    try:
        script = _repo_file(
            "ros2_package/robo_orchard_teleop_ros2/"
            "robo_orchard_teleop_ros2/scripted/joint_master.py"
        )
    except ScenarioError:
        return [
            "ros2",
            "run",
            "robo_orchard_teleop_ros2",
            "scripted_joint_master",
        ]
    return ["python3", str(script)]


def _ros_value(value) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, list):
        return json.dumps(value, separators=(",", ":"))
    return str(value)


def _run_command(args, scenario: Scenario) -> list[str]:
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    result_path = args.result_path or (
        f"/data/holobrain/robot_eval/{scenario.name}_{stamp}.csv"
    )
    params = {
        "scenario": scenario.name,
        "result_path": result_path,
    }
    if args.scenario_dir:
        params["scenario_directory"] = args.scenario_dir
    optional = {
        "speed_scale": args.speed_scale,
        "laps": args.laps,
        "duration_s": args.duration_s,
        "amplitude_scale": args.amplitude_scale,
        "frequency_scale": args.frequency_scale,
    }
    params.update(
        {key: value for key, value in optional.items() if value is not None}
    )
    for item in args.ros_param:
        if "=" not in item:
            raise ScenarioError(
                f"--ros-param must be NAME=JSON_OR_SCALAR, got {item!r}."
            )
        name, raw = item.split("=", 1)
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            value = raw
        params[name] = value
    command = _executor_command() + ["--ros-args"]
    for name, value in params.items():
        command.extend(["-p", f"{name}:={_ros_value(value)}"])
    return command


def _tool_command(
    action: str, scenario: Scenario, extra: list[str]
) -> list[str]:
    tool = scenario.tools.get(action)
    if tool != "circle":
        raise ScenarioError(
            f"scenario {scenario.name!r} does not support {action}."
        )
    script = Path(__file__).with_name("circle.py")
    python = (
        "/opt/robot-venv/bin/python3"
        if action in {"plan", "analyze"}
        and Path("/opt/robot-venv/bin/python3").exists()
        else "python3"
    )
    return [python, str(script), action, *extra]


def _print_command(command: list[str]) -> None:
    print(" ".join(shlex.quote(part) for part in command))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario-dir",
        default="",
        help="override the packaged scenario registry",
    )
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list")
    show = sub.add_parser("show")
    show.add_argument("scenario")

    run = sub.add_parser("run")
    run.add_argument("scenario")
    run.add_argument("--speed-scale", type=float)
    run.add_argument("--laps", type=int)
    run.add_argument("--duration-s", type=float)
    run.add_argument("--amplitude-scale", type=float)
    run.add_argument("--frequency-scale", type=float)
    run.add_argument("--result-path", default="")
    run.add_argument("--ros-param", action="append", default=[])
    run.add_argument("--dry-run", action="store_true")

    for action in ("plan", "analyze"):
        tool = sub.add_parser(action)
        tool.add_argument("scenario")
        tool.add_argument("--dry-run", action="store_true")

    args, extra = parser.parse_known_args()
    directory = args.scenario_dir or None
    try:
        if args.command == "list":
            if extra:
                parser.error(f"unrecognized arguments: {' '.join(extra)}")
            for scenario in list_scenarios(directory):
                print(f"{scenario.name:12} {scenario.label}")
            return
        scenario = load_scenario(args.scenario, directory)
        if args.command == "show":
            if extra:
                parser.error(f"unrecognized arguments: {' '.join(extra)}")
            print(
                json.dumps(
                    {
                        "name": scenario.name,
                        "label": scenario.label,
                        "description": scenario.description,
                        "mode": scenario.mode,
                        "parameters": scenario.parameters,
                        "ui": scenario.ui,
                        "safety": scenario.safety,
                        "tools": scenario.tools,
                        "path": str(scenario.path),
                    },
                    indent=2,
                )
            )
            return
        if args.command == "run":
            if extra:
                parser.error(f"unrecognized arguments: {' '.join(extra)}")
            command = _run_command(args, scenario)
        else:
            command = _tool_command(args.command, scenario, extra)
        if args.dry_run:
            _print_command(command)
            return
        os.execvp(command[0], command)
    except ScenarioError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
