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

import os
import subprocess


def test_required_stream_failure_exits_and_cleans_up() -> None:
    """Exercise the node process using only the injected fake SDK."""
    environment = dict(os.environ, WUJI_HAND2_TEST_MODE="terminal")
    result = subprocess.run(
        [os.environ["WUJI_HAND2_TEST_NODE"]],
        env=environment,
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    marker = "WUJI_HAND2_TEST_TERMINAL_CALLS="
    assert marker in output
    calls = output.split(marker, maxsplit=1)[1].splitlines()[0].split(",")
    cleanup = [
        calls.index(operation)
        for operation in (
            "sub_close",
            "close_joint_command_publisher",
            "disable",
            "disconnect",
            "release",
            "shutdown",
        )
    ]
    assert cleanup == sorted(cleanup), calls
    assert "enable" not in calls
