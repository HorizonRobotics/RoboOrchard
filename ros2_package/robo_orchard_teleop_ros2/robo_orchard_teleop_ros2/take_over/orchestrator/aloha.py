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

import threading

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, ParameterDescriptor
from std_srvs.srv import Trigger

# Per-call hang guard for a downstream service call. Normal completion is
# sub-second; the auto flow's worst case is 3 serial calls = 30 s.
_DOWNSTREAM_CALL_TIMEOUT_S = 10.0

# Bounded wait for a downstream service to be discovered, absorbing the
# ROS2 discovery race right after launch. Returns immediately once the
# service is already available, so steady-state calls pay no cost.
_SERVICE_WAIT_TIMEOUT_S = 2.0


class AlohaOrchestratorNode(Node):
    """Per-side orchestrator for the Aloha master-slave teleop stack.

    Owns two ``std_srvs/srv/Trigger`` services, ``auto`` and ``takeover``.
    ``auto`` enables the configured arms before switching the muxer, closing
    the TEACH-mode race where the muxer forwards commands to arms still in
    TEACH_MODE. One instance per side; the node has no concept of left/right
    and works purely from its three service-name parameters.

    STOP is out of scope (Path A): no stop service here -- the frontend Stop
    button calls the muxer directly.
    """

    def __init__(self):
        super().__init__("aloha_orchestrator_node")

        # --- Parameters ---
        self.declare_parameter(
            "enable_services",
            [""],
            ParameterDescriptor(
                description="Fully-qualified names of every enable_ctrl service to enable on this side (slave and master arms in the Aloha deployment)."  # noqa: E501
            ),
        )
        self.declare_parameter(
            "muxer_release_service",
            "",
            ParameterDescriptor(
                description="Fully-qualified name of the muxer's release_control service."  # noqa: E501
            ),
        )
        self.declare_parameter(
            "muxer_takeover_service",
            "",
            ParameterDescriptor(
                description="Fully-qualified name of the muxer's trigger_takeover service."  # noqa: E501
            ),
        )

        # Get parameter values
        self._enable_services = self.get_parameter("enable_services").value
        self._muxer_release_service = self.get_parameter(
            "muxer_release_service"
        ).value
        self._muxer_takeover_service = self.get_parameter(
            "muxer_takeover_service"
        ).value

        # --- Startup parameter validation (fail fast) ---
        # Defaults are empty (no sensible per-side default); a missing launch
        # key silently falls back to them, so fail at construction.
        if not self._enable_services:
            self.get_logger().error(
                "Required parameter `enable_services` is empty. "
                "It must be a non-empty list of enable_ctrl service names."
            )
            raise ValueError("Required parameter `enable_services` is empty.")
        for service_name in self._enable_services:
            if not service_name:
                self.get_logger().error(
                    "Parameter `enable_services` contains an empty service "
                    "name. Every entry must be a non-empty service name."
                )
                raise ValueError(
                    "Parameter `enable_services` contains an empty service "
                    "name."
                )
        for param_name, param_value in (
            ("muxer_release_service", self._muxer_release_service),
            ("muxer_takeover_service", self._muxer_takeover_service),
        ):
            if not param_value:
                self.get_logger().error(
                    f"Required parameter `{param_name}` is empty. "
                    "It must be set to a non-empty service name."
                )
                raise ValueError(
                    f"Required parameter `{param_name}` is empty."
                )

        # --- Callback groups (executor strategy, see plan Section 6.6) ---
        # Servers are mutually exclusive (one mode switch at a time); clients
        # use a separate group so a downstream response is handled while a
        # server callback blocks waiting on it.
        self._server_cb_group = MutuallyExclusiveCallbackGroup()
        self._client_cb_group = ReentrantCallbackGroup()

        # --- Service clients ---
        # One Trigger client per enable_services entry, in order, plus the two
        # muxer clients. Validation above guarantees every name is non-empty.
        self._enable_clients = [
            self.create_client(
                Trigger,
                service_name,
                callback_group=self._client_cb_group,
            )
            for service_name in self._enable_services
        ]
        self._release_client = self.create_client(
            Trigger,
            self._muxer_release_service,
            callback_group=self._client_cb_group,
        )
        self._takeover_client = self.create_client(
            Trigger,
            self._muxer_takeover_service,
            callback_group=self._client_cb_group,
        )

        # --- Service servers ---
        self._auto_service = self.create_service(
            Trigger,
            "auto",
            self._auto_service_callback,
            callback_group=self._server_cb_group,
        )
        self._takeover_service = self.create_service(
            Trigger,
            "takeover",
            self._takeover_service_callback,
            callback_group=self._server_cb_group,
        )

        self.get_logger().info(
            "AlohaOrchestratorNode initialized. "
            f"enable_services={self._enable_services}, "
            f"muxer_release_service={self._muxer_release_service}, "
            f"muxer_takeover_service={self._muxer_takeover_service}"
        )

    def _call_service_sync(
        self, client, service_name: str
    ) -> Trigger.Response:
        """Call a downstream Trigger service synchronously.

        Waits up to ``_SERVICE_WAIT_TIMEOUT_S`` for the service to be
        discovered, absorbing the post-launch discovery race. Then waits via
        ``call_async`` + ``add_done_callback`` + ``threading.Event`` rather
        than ``spin_until_future_complete``, which would re-enter the executor
        from inside a service callback. On timeout the abandoned request is
        dropped from the client. Returns ``success=False`` on an unavailable
        service, a timeout, or an exception.
        """
        if not client.wait_for_service(timeout_sec=_SERVICE_WAIT_TIMEOUT_S):
            self.get_logger().error(
                f"Service `{service_name}` is not available."
            )
            response = Trigger.Response()
            response.success = False
            response.message = f"Service `{service_name}` is not available."
            return response

        try:
            future = client.call_async(Trigger.Request())
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            if not event.wait(_DOWNSTREAM_CALL_TIMEOUT_S):
                # Drop the abandoned request so a permanently hung
                # downstream service does not leak a pending-request slot.
                client.remove_pending_request(future)
                self.get_logger().error(
                    f"Service `{service_name}` call timed out after "
                    f"{_DOWNSTREAM_CALL_TIMEOUT_S} s."
                )
                response = Trigger.Response()
                response.success = False
                response.message = (
                    f"Service `{service_name}` call timed out after "
                    f"{_DOWNSTREAM_CALL_TIMEOUT_S} s."
                )
                return response
            return future.result()
        except Exception as e:
            self.get_logger().error(
                f"Service `{service_name}` call failed: {e}"
            )
            response = Trigger.Response()
            response.success = False
            response.message = f"Service `{service_name}` call failed: {e}"
            return response

    def _auto_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Enable every configured arm, then release muxer to AUTONOMOUS."""
        # Serially enable every arm; the first failure short-circuits before
        # the muxer is switched.
        for service_name, client in zip(
            self._enable_services, self._enable_clients, strict=True
        ):
            enable_result = self._call_service_sync(client, service_name)
            if not enable_result.success:
                self.get_logger().warn(
                    f"Auto: enable_ctrl on `{service_name}` failed: "
                    f"{enable_result.message}"
                )
                response.success = False
                response.message = (
                    f"enable_ctrl on `{service_name}` failed: "
                    f"{enable_result.message}"
                )
                return response

        release_result = self._call_service_sync(
            self._release_client, self._muxer_release_service
        )

        # Propagate a failed muxer call; arms stay enabled, no rollback.
        if not release_result.success:
            self.get_logger().warn(
                f"Auto: muxer release_control failed: {release_result.message}"
            )
            response.success = False
            response.message = release_result.message
            return response

        response.success = True
        response.message = "Auto mode activated."
        self.get_logger().info(response.message)
        return response

    def _takeover_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Trigger takeover (OVERRIDE) on the muxer; enables no arm.

        ``enable_ctrl`` switches an arm to CAN_MODE, but takeover needs the
        master arm in TEACH_MODE -- physical-button-only, software cannot set
        it -- and the slave is already controllable. So the callback only logs
        a reminder, then calls the muxer.
        """
        # Static reminder; the orchestrator queries no arm state.
        self.get_logger().info(
            "Takeover: use the hardware teach button on the master arm to "
            "put it into teach mode. The software/orchestrator cannot enter "
            "teach mode -- it is physical-button-only."
        )

        takeover_result = self._call_service_sync(
            self._takeover_client, self._muxer_takeover_service
        )

        # Propagate a failed muxer call, same as the auto flow.
        if not takeover_result.success:
            self.get_logger().warn(
                "Takeover: muxer trigger_takeover failed: "
                f"{takeover_result.message}"
            )
            response.success = False
            response.message = takeover_result.message
            return response

        response.success = True
        response.message = "Takeover mode activated."
        self.get_logger().info(response.message)
        return response


def main(args=None):
    """Initialize and spin the node.

    Uses MultiThreadedExecutor (not the muxer's single-threaded spin): the
    service callbacks make synchronous downstream calls. See plan Section 6.6.
    """
    rclpy.init(args=args)
    node = AlohaOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
