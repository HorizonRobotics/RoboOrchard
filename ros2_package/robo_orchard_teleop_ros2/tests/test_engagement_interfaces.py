# Project RoboOrchard
#
# Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
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

from pathlib import Path
from xml.etree import ElementTree

INTERFACE_ROOT = (
    Path(__file__).resolve().parents[2] / "robo_orchard_teleop_msg_ros2"
)


def _definition(path: str) -> list[str]:
    return [
        line.strip()
        for line in (INTERFACE_ROOT / path).read_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


def test_match_status_is_source_neutral():
    assert _definition("msg/TeleopMatchState.msg") == [
        "std_msgs/Header header",
        "string candidate_topic",
        "uint64 loss_id",
        "bool ready",
        "bool matched",
        "string reason",
    ]


def test_recapture_acknowledgement_includes_candidate_cutover():
    assert _definition("srv/RecaptureBaseline.srv") == [
        "---",
        "bool success",
        "string reason",
        "string candidate_topic",
        "builtin_interfaces/Time stamp_barrier",
    ]


def test_cutover_interface_build_dependencies():
    manifest = ElementTree.parse(INTERFACE_ROOT / "package.xml").getroot()
    dependencies = {element.text for element in manifest.findall("depend")}
    assert {"builtin_interfaces", "std_msgs"} <= dependencies
    assert manifest.findtext("exec_depend") == "rosidl_default_runtime"

    cmake = (INTERFACE_ROOT / "CMakeLists.txt").read_text()
    assert '"srv/RecaptureBaseline.srv"' in cmake
    assert "find_package(builtin_interfaces REQUIRED)" in cmake
    assert "ament_export_dependencies(rosidl_default_runtime)" in cmake
