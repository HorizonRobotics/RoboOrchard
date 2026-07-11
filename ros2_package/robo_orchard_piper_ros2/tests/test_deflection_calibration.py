# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

import json

import pytest

from robo_orchard_piper_ros2.ros_bridge import (
    GravityCompensationError,
    resolve_deflection_calibration,
    resolve_friction_calibration,
    resolve_gravity_scale_calibration,
)

STORE = {
    "left": {
        "25": {
            "offset_stiffness": [0.0, 232.0, 291.0, 51.0, 0.0, 0.0],
            "deflection_table": {"2": [[1.34, 0.0041], [9.67, 0.0321]]},
        },
        "40": {
            "offset_stiffness": [0.0, 207.0, 418.0, 93.0, 0.0, 0.0],
            "deflection_table": {},
        },
    },
    "right": {
        "25": {
            "offset_stiffness": [0.0, 241.0, 289.0, 41.0, 0.0, 0.0],
            "deflection_table": {"2": [[1.34, 0.0039], [9.67, 0.0315]]},
        }
    },
}


@pytest.fixture
def store_path(tmp_path):
    path = tmp_path / "deflection_calibrations.json"
    path.write_text(json.dumps(STORE))
    return str(path)


def test_exact_kp_match(store_path):
    stiffness, table_json = resolve_deflection_calibration(
        store_path, "left", 25.0
    )
    assert stiffness == [0.0, 232.0, 291.0, 51.0, 0.0, 0.0]
    assert json.loads(table_json) == {"2": [[1.34, 0.0041], [9.67, 0.0321]]}


def test_integer_vs_float_kp_key(store_path):
    # "25" in the file must match the mit_kp double parameter 25.0.
    stiffness, _ = resolve_deflection_calibration(store_path, "right", 25)
    assert stiffness[1] == 241.0


def test_empty_table_serializes_to_empty_string(store_path):
    _, table_json = resolve_deflection_calibration(store_path, "left", 40.0)
    assert table_json == ""


def test_uncalibrated_kp_lists_available(store_path):
    with pytest.raises(GravityCompensationError) as exc:
        resolve_deflection_calibration(store_path, "left", 30.0)
    msg = str(exc.value)
    assert "mit_kp=30" in msg
    assert "[25, 40]" in msg
    assert "measure_deflection.py" in msg


def test_unknown_side(store_path):
    with pytest.raises(GravityCompensationError, match="side 'middle'"):
        resolve_deflection_calibration(store_path, "middle", 25.0)


def test_missing_file(tmp_path):
    with pytest.raises(GravityCompensationError, match="does not exist"):
        resolve_deflection_calibration(
            str(tmp_path / "nope.json"), "left", 25.0
        )


def test_malformed_stiffness_rejected(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(
        json.dumps(
            {
                "left": {
                    "25": {"offset_stiffness": [1.0], "deflection_table": {}}
                }
            }
        )
    )
    with pytest.raises(GravityCompensationError, match="6 offset_stiffness"):
        resolve_deflection_calibration(str(path), "left", 25.0)


def test_shipped_store_covers_both_arms_at_25_and_40():
    shipped = "/opt/roboorchard/calibration/deflection_calibrations.json"
    import os

    if not os.path.exists(shipped):
        pytest.skip("shipped calibration store not mounted")
    for side in ("left", "right"):
        for kp in (25.0, 40.0):
            stiffness, table_json = resolve_deflection_calibration(
                shipped, side, kp
            )
            assert len(stiffness) == 6
            assert json.loads(table_json)["2"]


# --- v2 store sections (friction / gravity) ---------------------------

V2_STORE = dict(
    STORE,
    store_version=2,
    friction={
        "left": {
            "scale": [0.0, 0.55, 0.18, 0.0, 0.0, 0.0],
            "taper_velocity": 4.0,
            "measured": "2026-07-08",
        },
        "right": {"scale": [0.0, 0.5, 0.2, 0.0, 0.0, 0.0]},
    },
    gravity={
        "left": {"scale_per_joint": [1.0, 1.09, 1.06, 0.88, 1.0, 1.0]},
    },
)


@pytest.fixture
def v2_store_path(tmp_path):
    # distinct filename: some tests use both fixtures in one tmp_path
    path = tmp_path / "deflection_calibrations_v2.json"
    path.write_text(json.dumps(V2_STORE))
    return str(path)


def test_v2_sections_do_not_break_deflection_lookup(v2_store_path):
    stiffness, _ = resolve_deflection_calibration(v2_store_path, "left", 25.0)
    assert stiffness == [0.0, 232.0, 291.0, 51.0, 0.0, 0.0]
    with pytest.raises(GravityCompensationError, match="side 'middle'"):
        resolve_deflection_calibration(v2_store_path, "middle", 25.0)


def test_friction_lookup(v2_store_path):
    entry = resolve_friction_calibration(v2_store_path, "left")
    assert entry == {
        "scale": [0.0, 0.55, 0.18, 0.0, 0.0, 0.0],
        "taper_velocity": 4.0,
    }


def test_friction_taper_optional(v2_store_path):
    entry = resolve_friction_calibration(v2_store_path, "right")
    assert entry["scale"][1] == 0.5
    assert entry["taper_velocity"] is None


def test_friction_missing_section_returns_none(store_path):
    # pre-v2 store: silent fallback to parameter values, not an error
    assert resolve_friction_calibration(store_path, "left") is None


def test_friction_missing_side_returns_none(v2_store_path):
    store = dict(V2_STORE, friction={"left": V2_STORE["friction"]["left"]})
    import pathlib

    path = pathlib.Path(v2_store_path)
    path.write_text(json.dumps(store))
    assert resolve_friction_calibration(str(path), "right") is None


def test_friction_missing_file_raises(tmp_path):
    with pytest.raises(GravityCompensationError, match="does not exist"):
        resolve_friction_calibration(str(tmp_path / "nope.json"), "left")


@pytest.mark.parametrize(
    "entry",
    [
        {"scale": [0.0, 0.55, 0.18]},  # wrong length
        {"scale": [0.0, 2.5, 0.0, 0.0, 0.0, 0.0]},  # out of range
        {"scale": [0.0, -0.1, 0.0, 0.0, 0.0, 0.0]},  # negative
        {"taper_velocity": 4.0},  # no scale at all
        {"scale": [0.0] * 6, "taper_velocity": -1.0},  # bad taper
    ],
)
def test_friction_malformed_entry_raises(tmp_path, entry):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps({"friction": {"left": entry}}))
    with pytest.raises(GravityCompensationError):
        resolve_friction_calibration(str(path), "left")


def test_gravity_scale_lookup(v2_store_path):
    scale = resolve_gravity_scale_calibration(v2_store_path, "left")
    assert scale == [1.0, 1.09, 1.06, 0.88, 1.0, 1.0]


def test_gravity_scale_missing_returns_none(store_path, v2_store_path):
    assert resolve_gravity_scale_calibration(store_path, "left") is None
    assert resolve_gravity_scale_calibration(v2_store_path, "right") is None


def test_gravity_scale_malformed_raises(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(
        json.dumps({"gravity": {"left": {"scale_per_joint": [3.0] * 6}}})
    )
    with pytest.raises(GravityCompensationError, match="scale_per_joint"):
        resolve_gravity_scale_calibration(str(path), "left")
