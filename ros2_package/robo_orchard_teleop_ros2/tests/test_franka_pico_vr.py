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
import sys
import textwrap
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_franka_reset_client_flow_in_subprocess():
    script = textwrap.dedent(
        """
        import sys
        import types
        from types import SimpleNamespace

        import test_pico_teleop
        import test_pico_vr_node

        franka_msgs = types.ModuleType("franka_msgs")
        franka_msgs.msg = types.ModuleType("franka_msgs.msg")
        franka_msgs.msg.FrankaRobotState = type("FrankaRobotState", (), {})
        sys.modules["franka_msgs"] = franka_msgs
        sys.modules["franka_msgs.msg"] = franka_msgs.msg
        sys.modules["xacro"] = types.ModuleType("xacro")

        from robo_orchard_teleop_ros2.robot.franka.pico_vr import (
            FrankaPicoVRTeleOpNode,
        )
        from robo_orchard_teleop_ros2.bridge.pico.teleop import Action


        class Logger:
            def __init__(self):
                self.info_messages = []
                self.warning_messages = []
                self.error_messages = []

            def info(self, message):
                self.info_messages.append(message)

            def warning(self, message):
                self.warning_messages.append(message)

            def error(self, message):
                self.error_messages.append(message)


        class Future:
            def __init__(self, response=None):
                self.response = response
                self.callback = None

            def add_done_callback(self, callback):
                self.callback = callback

            def result(self):
                return self.response


        class ResetClient:
            def __init__(self):
                self.requests = []
                self.future = Future()
                self.ready = True

            def service_is_ready(self):
                return self.ready

            def call_async(self, request):
                self.requests.append(request)
                return self.future


        logger = Logger()
        client = ResetClient()
        finish_reset_calls = []
        joint_state_updates = []
        teleop = SimpleNamespace(
            begin_reset=lambda: None,
            finish_reset=lambda: finish_reset_calls.append(True),
            update_robot_joint_state=joint_state_updates.append,
        )
        node = SimpleNamespace(
            _reset_future=None,
            _manager_resetting=False,
            _reset_generation=0,
            _destroying=False,
            reset_client=client,
            reset_service_name="/robot/control/reset",
            get_logger=lambda: logger,
            _reset_result=lambda generation, future: None,
            teleop=teleop,
            source_type="right",
            _last_input_times={},
            _check_input_freshness=lambda now: True,
            current_joint_positions=None,
            _finish_reset_state=lambda: (
                FrankaPicoVRTeleOpNode._finish_reset_state(node)
            ),
        )

        FrankaPicoVRTeleOpNode._request_reset(node)

        assert len(client.requests) == 1
        assert node._reset_future is client.future
        assert client.future.callback.func is node._reset_result
        assert client.future.callback.args == (1,)

        node.current_joint_positions = [0.1] * 7
        response = SimpleNamespace(success=True, message="reset complete")

        FrankaPicoVRTeleOpNode._reset_result(node, 1, Future(response))

        assert finish_reset_calls == [True]
        assert joint_state_updates == [[0.1] * 7]
        assert node._reset_future is None
        assert logger.info_messages[-1] == (
            "Franka reset completed successfully."
        )

        client.ready = False
        node.current_joint_positions = None
        FrankaPicoVRTeleOpNode._request_reset(node)

        assert len(client.requests) == 1
        assert finish_reset_calls == [True, True]
        assert logger.warning_messages[-1] == (
            "Franka reset service '/robot/control/reset' is unavailable."
        )

        reset_requests = []
        node.teleop.update_vr_state = lambda message: Action.RESET
        node._request_reset = lambda: reset_requests.append(True)

        FrankaPicoVRTeleOpNode._vr_state_callback(
            node, SimpleNamespace(
                right_controller=SimpleNamespace(status=1),
            ),
        )

        assert reset_requests == [True]
        assert logger.info_messages[-1] == "Reset gesture received."
        """
    )

    environment = os.environ.copy()
    for name in tuple(environment):
        if name.startswith("COV_CORE_") or name == "COVERAGE_PROCESS_START":
            environment.pop(name)
    python_path = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(PACKAGE_ROOT), str(PACKAGE_ROOT / "tests")]
    )
    if python_path:
        environment["PYTHONPATH"] += os.pathsep + python_path

    subprocess.run(
        [sys.executable, "-c", script],
        check=True,
        env=environment,
    )
