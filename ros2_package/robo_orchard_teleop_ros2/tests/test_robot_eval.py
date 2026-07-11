import csv
import json
from types import SimpleNamespace

import pytest

from robo_orchard_teleop_ros2.robot_eval.circle import _theta_profile
from robo_orchard_teleop_ros2.robot_eval.cli import (
    _run_command,
    _tool_command,
)
from robo_orchard_teleop_ros2.robot_eval.results import (
    RESULT_SCHEMA_VERSION,
    ResultWriter,
)
from robo_orchard_teleop_ros2.robot_eval.schema import (
    ScenarioError,
    list_scenarios,
    load_scenario,
)


def test_packaged_scenarios_are_ordered_and_valid():
    scenarios = list_scenarios()
    assert [scenario.name for scenario in scenarios] == [
        "sine",
        "stress",
        "circle",
    ]


def test_stress_scenario_is_the_single_ros_parameter_source():
    scenario = load_scenario("stress")
    params = scenario.ros_parameters()
    assert params["mode"] == "waypoints"
    assert len(params["waypoints_left"]) == 7 * 7
    assert params["waypoints_left"] == params["waypoints_right"]
    assert params["pass_through_indices"] == [3]
    assert params["duration_s"] == 0.0
    assert params["use_current_state"] is True


def test_invalid_pass_through_index_is_rejected(tmp_path):
    payload = json.loads(load_scenario("stress").path.read_text())
    payload["name"] = "bad"
    payload["executor"]["parameters"]["pass_through_indices"] = [99]
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(payload))
    with pytest.raises(ScenarioError, match="beyond"):
        load_scenario(path)


def test_cli_run_passes_scenario_and_operator_overrides_only():
    scenario = load_scenario("stress")
    args = SimpleNamespace(
        scenario_dir="",
        result_path="/tmp/result.csv",
        speed_scale=0.6,
        laps=2,
        duration_s=None,
        amplitude_scale=None,
        frequency_scale=None,
        ros_param=[],
    )
    command = _run_command(args, scenario)
    joined = " ".join(command)
    assert "scenario:=stress" in joined
    assert "speed_scale:=0.6" in joined
    assert "laps:=2" in joined
    assert "waypoints_left" not in joined
    assert "pass_through_indices" not in joined


def test_circle_tool_is_owned_by_robot_eval_package():
    scenario = load_scenario("circle")
    command = _tool_command("plan", scenario, ["--side", "left"])

    assert command[1].endswith("robot_eval/circle.py")
    assert "calibration" not in command[1]


def test_circle_theta_profile_closes_at_requested_laps():
    times, angles = _theta_profile(laps=2, period=4.0, ramp=1.0, rate=100.0)

    assert times[0] == 0.0
    assert times[-1] == pytest.approx(9.0)
    assert angles[0] == 0.0
    assert angles[-1] == pytest.approx(4.0 * 3.141592653589793)


def test_common_result_writer_preserves_both_arms(tmp_path):
    path = tmp_path / "result.csv"
    writer = ResultWriter(str(path), scenario="stress", mode="waypoints")
    writer.write(
        1.25,
        left_command=[0.1] * 7,
        left_measured=[0.09] * 7,
        right_command=[-0.1] * 7,
        right_measured=[-0.09] * 7,
        phase="trajectory",
        trajectory_time=1.0,
    )
    writer.close()

    row = next(csv.DictReader(path.open()))
    assert int(row["schema_version"]) == RESULT_SCHEMA_VERSION
    assert row["scenario"] == "stress"
    assert row["mode"] == "waypoints"
    assert float(row["trajectory_t"]) == pytest.approx(1.0)
    assert float(row["left_cmd1"]) == pytest.approx(0.1)
    assert float(row["right_meas7"]) == pytest.approx(-0.09)
