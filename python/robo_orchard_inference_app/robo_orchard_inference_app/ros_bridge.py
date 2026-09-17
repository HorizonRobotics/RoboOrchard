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

import atexit
import threading
import time
from copy import deepcopy
from functools import partial
from pathlib import Path
from typing import Callable, Literal

import roslibpy

from robo_orchard_inference_app.config import ROSBridgeCfg
from robo_orchard_inference_app.logger import Logger
from robo_orchard_inference_app.state import InferenceState


class RosServiceHelper:
    """Encapsulates ROS service requests and runtime status subscriptions."""

    def __init__(
        self,
        ros_client: roslibpy.Ros,
        ros_bridge_cfg: ROSBridgeCfg,
        inference_state: InferenceState,
        logger: Logger,
    ):
        """Initializes the RosServiceHelper.

        Args:
            ros_client: The active roslibpy.Ros client connection.
            ros_bridge_cfg: Configuration for ROS bridge services.
            inference_state: The shared inference state object to update.
            logger: Logger for logging
        """
        self.ros_client = ros_client
        self.cfg = ros_bridge_cfg
        self.state = inference_state
        self.logger = logger
        self._synced_tf_fingerprint: tuple[str, frozenset] | None = None
        self._status_lock = threading.Lock()
        self._status_messages: dict[str, tuple[float, dict]] = {}
        self._status_topics: list[roslibpy.Topic] = []
        self._recorder_stop_status: dict | None = None
        atexit.register(self.cleanup)

    def start_status_monitor(self) -> None:
        """Subscribe once; callbacks cache snapshots without updating UI state.

        Start and cleanup are owned by the UI thread. Transport callbacks
        only access the cache under its lock.
        """
        if self._status_topics:
            return
        topics = [
            (key, roslibpy.Topic(self.ros_client, name, message_type))
            for key, name, message_type in (
                (
                    "inference",
                    self.cfg.inference_status_topic,
                    "robo_orchard_deploy_msg_ros2/msg/InferenceStatus",
                ),
                (
                    "recorder",
                    f"{self.cfg.recorder_name}/status",
                    "robo_orchard_data_msg_ros2/msg/RecorderStatus",
                ),
                (
                    "control",
                    self.cfg.control_status_topic,
                    "robo_orchard_teleop_msg_ros2/msg/ControlMode",
                ),
            )
        ]
        self.ros_client.on("close", self._invalidate_status)
        try:
            for key, topic in topics:
                self._status_topics.append(topic)
                topic.subscribe(partial(self._receive_status, key))
        except Exception:
            self._stop_status_monitor()
            raise

    def _receive_status(self, key: str, message: dict) -> None:
        with self._status_lock:
            if not self._status_topics or not self.ros_client.is_connected:
                return
            message = deepcopy(message)
            self._status_messages[key] = (time.monotonic(), message)
            stopped = self._recorder_stop_status
            if (
                key == "recorder"
                and stopped is not None
                and stopped.get("data") is None
                and message.get("session_id") == stopped["session_id"]
                and message.get("destination") == stopped["destination"]
                and message.get("data") in {"completed", "idle", "failed"}
            ):
                self._recorder_stop_status = message

    def recorder_stop_result(
        self, session_id: str, destination: str
    ) -> dict | None:
        """Consume a matching Stop terminal result, not a live status.

        Received terminal evidence survives later sessions and disconnection
        until the App consumes it. Only one explicit Stop is tracked.
        """
        with self._status_lock:
            result = self._recorder_stop_status
            if (
                result is None
                or result.get("data") is None
                or result["session_id"] != session_id
                or result["destination"] != destination
            ):
                return None
            self._recorder_stop_status = None
            return deepcopy(result)

    def _invalidate_status(self, *args) -> None:
        with self._status_lock:
            self._status_messages.clear()

    def status_snapshot(self, key: str) -> dict | None:
        """Return a copy of a fresh snapshot, or None when unavailable.

        Freshness uses local monotonic receive time, not the ROS timestamp.
        """
        with self._status_lock:
            if not self.ros_client.is_connected:
                self._status_messages.clear()
                return None
            received = self._status_messages.get(key)
            if received is None:
                return None
            timestamp, message = received
            if time.monotonic() - timestamp > self.cfg.status_timeout_s:
                return None
            return deepcopy(message)

    def refresh_runtime_state(self) -> None:
        """Project control and inference snapshots into the UI-owned model."""
        control = self.status_snapshot("control")
        mode = control.get("data") if control is not None else None
        self.state.control_mode = (
            mode
            if isinstance(mode, str)
            and mode in {"auto", "takeover", "stop", "resetting"}
            else None
        )
        inference = self.status_snapshot("inference")
        value = inference.get("data") if inference is not None else None
        self.state.is_inference_service_running = (
            {"enabled": True, "disabled": False}.get(value)
            if isinstance(value, str)
            else None
        )

    def _stop_status_monitor(self) -> None:
        with self._status_lock:
            topics = self._status_topics
            self._status_topics = []
            self._status_messages.clear()
            self._recorder_stop_status = None
        if topics:
            self.ros_client.off("close", self._invalidate_status)
        error: Exception | None = None
        for topic in topics:
            try:
                topic.unsubscribe()
            except Exception as exc:
                if error is None:
                    error = exc
        if error is not None:
            raise error

    def _check_client_connected(self) -> bool:
        """Checks if the ROS client is connected.

        Returns:
            True if connected, False otherwise.
        """
        if not self.ros_client or not self.ros_client.is_connected:
            self.logger.error("🚨 ROS is not connected!")
            return False
        return True

    def _call_services(
        self,
        service_names: str | list[str],
        success_msg: str,
        success_callback: Callable[[], None] | None = None,
        timeout: float = 5.0,
        service_type: str = "std_srvs/srv/Trigger",
        request_data: dict | None = None,
    ) -> bool:
        """Generic helper to call a list of ROS services.

        Args:
            service_names: A list of ROS service names to call.
            success_msg: Message to display upon successful calls.
            success_callback: A function to call after all services succeed.
            timeout: Timeout in seconds for each service call.
            service_type: Service type.
            request_data: Service request data.

        Returns:
            True if all service calls were successful, False otherwise.
        """
        if not self._check_client_connected():
            return False

        if isinstance(service_names, str):
            service_names = [service_names]

        available_services: list[str] = self.ros_client.get_services()

        for service_name in service_names:
            if service_name not in available_services:
                self.logger.error(f"Service {service_name} not found!")
                return False
            if not self._call_service(
                service_name=service_name,
                timeout=timeout,
                service_type=service_type,
                request_data=request_data,
            ):
                return False

        self.logger.info(success_msg)
        if success_callback:
            success_callback()
        return True

    def _call_service(
        self,
        service_name: str,
        timeout: float,
        service_type: str,
        request_data: dict | None,
    ) -> bool:
        success, error_msg = self._call_service_result(
            service_name=service_name,
            timeout=timeout,
            service_type=service_type,
            request_data=request_data,
        )
        if error_msg:
            self.logger.error(error_msg)
        return success is True

    def _call_service_result(
        self,
        service_name: str,
        timeout: float,
        service_type: str,
        request_data: dict | None,
    ) -> tuple[bool | None, str | None]:
        """Return acceptance, rejection, or an unknown result with details."""
        try:
            service = roslibpy.Service(
                self.ros_client, service_name, service_type
            )
            request = roslibpy.ServiceRequest(request_data)
        except Exception as error:
            return False, f"Cannot prepare service {service_name}: {error}"
        try:
            result = service.call(request, timeout=timeout)
            if result.get("success") is False:
                msg = result.get("message", "No message provided.")
                return False, f"Service {service_name} failed: {msg}"
            if result.get("success") is not True:
                return None, f"Invalid response from service: {service_name}"
        except roslibpy.core.RosTimeoutError:
            return None, f"Timeout calling service: {service_name}"
        except Exception as error:
            return None, f"Error calling {service_name}: {error}"
        return True, None

    def _set_param(
        self,
        node_name: str,
        request_data: dict | None = None,
        timeout: float = 5.0,
    ) -> bool:
        """Generic helper to set a ROS parameter on a list of nodes.

        Args:
            node_name: The ROS node name to set the parameter on.
            request_data: Parameter name and value to set.
            timeout: Timeout in seconds for the parameter set service call.

        Returns:
            bool: True if the parameter was set successfully, False otherwise.
        """
        if not self._check_client_connected():
            return False

        available_services: list[str] = self.ros_client.get_services()
        service_type = "rcl_interfaces/srv/SetParameters"
        set_param_service = f"{node_name}/set_parameters"
        if set_param_service not in available_services:
            self.logger.error(
                f"Parameter service {set_param_service} not found!"
            )
            return False
        try:
            service = roslibpy.Service(
                self.ros_client, set_param_service, service_type
            )
            request = roslibpy.ServiceRequest(request_data)
            result = service.call(request, timeout=timeout)
            if "results" in result and result["results"][0]["successful"]:
                return True
            else:
                self.logger.error("Failed to set parameter.")
                return False
        except roslibpy.core.RosTimeoutError:
            self.logger.error(
                f"Timeout calling set parameter service: {set_param_service}"
            )
            return False
        except Exception as e:
            self.logger.error(f"Error calling {set_param_service}: {e}")
            return False

    def get_node_names(self) -> list[str]:
        if not self._check_client_connected():
            return []

        try:
            service = roslibpy.Service(
                self.ros_client, "/rosapi/nodes", "rosapi_msgs/srv/Nodes"
            )
            request = roslibpy.ServiceRequest({})
            result = service.call(request, timeout=5.0)
            return [
                node
                for node in result.get("nodes", [])
                if isinstance(node, str)
            ]
        except Exception as e:
            self.logger.error(f"Error calling /rosapi/nodes: {e}")
            return []

    def get_tf_publisher_startup_id(
        self, node_name: str = "/static_tf_publisher"
    ) -> str | None:
        if not self._check_client_connected():
            return None

        get_param_service = f"{node_name}/get_parameters"
        try:
            service = roslibpy.Service(
                self.ros_client,
                get_param_service,
                "rcl_interfaces/srv/GetParameters",
            )
            request = roslibpy.ServiceRequest({"names": ["startup_id"]})
            result = service.call(request, timeout=3.0)
            values = result.get("values", [])
            if not values:
                return None
            value = values[0]
            string_value = value.get("string_value", "")
            if not string_value:
                return None
            return string_value
        except roslibpy.core.RosTimeoutError:
            return None
        except Exception as e:
            self.logger.error(
                f"Error calling {get_param_service} for startup_id: {e}"
            )
            return None

    def invalidate_static_transform_cache(self) -> None:
        self._synced_tf_fingerprint = None

    def _get_tf_directory_fingerprint(
        self, directory: str
    ) -> tuple[str, frozenset] | None:
        try:
            files = frozenset(
                (f.name, f.stat().st_mtime_ns, f.stat().st_size)
                for f in Path(directory).glob("*.json")
            )
        except OSError:
            return None
        return (directory, files)

    def sync_static_transforms(self, episode_meta) -> bool:
        directory = episode_meta.tf_directory
        if not directory:
            return True

        if not self.cfg.static_transform_service_name:
            self.logger.error(
                "static_transform_service_name is not configured; "
                "cannot sync static transforms."
            )
            return False

        fingerprint = self._get_tf_directory_fingerprint(directory)
        if fingerprint is None:
            return False
        if fingerprint == self._synced_tf_fingerprint:
            return True

        success = self._call_services(
            service_names=self.cfg.static_transform_service_name,
            success_msg="Static transforms loaded successfully!",
            service_type=(
                "robo_orchard_data_msg_ros2/srv/SetStaticTransforms"
            ),
            request_data={"directory": directory},
        )
        if success:
            self._synced_tf_fingerprint = fingerprint
        return success

    def reset_arm(self) -> bool:
        """Request the Control Manager-owned reset sequence."""
        return self._call_services(
            service_names=self.cfg.reset_service_name,
            success_msg="Robot reset completed successfully!",
            timeout=self.cfg.reset_timeout_s,
        )

    def enable_inference(self, episode_meta) -> bool:
        """Set instruction then sends a request to enable the inference service."""  # noqa: E501
        request_data = dict(parameters=[])
        param_value = episode_meta.instruction
        request_data["parameters"] = [
            {
                "name": "instruction",
                "value": {"type": 4, "string_value": param_value},
            }
        ]
        active_nodes = self.ros_client.get_nodes()
        candidate_nodes = self.cfg.inference_node_candidates
        if not candidate_nodes:
            self.logger.error("No inference node candidates configured!")
            return False

        matching_nodes = [
            node for node in candidate_nodes if node in active_nodes
        ]
        inference_node = None
        if len(matching_nodes) == 0:
            self.logger.error(
                "No matching inference nodes found among candidates!"
            )
            return False
        elif len(matching_nodes) > 1:
            self.logger.error(
                "Multiple matching inference nodes found! Please check!"
            )
            return False
        else:
            inference_node = matching_nodes[0]

        if not self._set_param(
            node_name=inference_node,
            request_data=request_data,
        ):
            self.logger.error("Failed to set inference params.")
            return False
        else:
            self.logger.info(
                f"Instruction parameter set successfully: {param_value}"
            )

        return self._call_services(
            service_names=self.cfg.enable_inference_service_name,
            success_msg="Inference service enabled!",
        )

    def disable_inference(self) -> bool:
        """Sends a request to disable the inference service."""
        return self._call_services(
            service_names=self.cfg.disable_inference_service_name,
            success_msg="Inference service disabled!",
        )

    def is_inference_node_active(self) -> bool:
        """Return True if a configured inference node is currently running.

        Disabling inference is a precondition for an arm reset only when an
        inference node exists; with none launched there is nothing that
        could contend with the reset.
        """
        active_nodes = self.get_node_names()
        return any(
            node in active_nodes for node in self.cfg.inference_node_candidates
        )

    def set_control_mode(
        self, mode: Literal["auto", "takeover", "stop"]
    ) -> bool:
        """Sets the robot's control mode."""
        service_map = {
            "auto": self.cfg.auto_service_name,
            "takeover": self.cfg.takeover_service_name,
            "stop": self.cfg.stop_service_name,
        }
        message_map = {
            "auto": "/Auto command sent successfully!",
            "takeover": "/TakeOver command sent successfully!",
            "stop": "/Stop command sent successfully!",
        }
        return self._call_services(
            service_names=service_map[mode],
            success_msg=message_map[mode],
            timeout=30.0,
        )

    def start_recording(self, uri: str) -> bool | None:
        """Request a recording session without inferring node state.

        Args:
            uri: Absolute recording destination associated with the request.

        Returns:
            True for an accepted request, False for rejection or failure before
            dispatch, and None when a dispatched request's outcome is unknown.
            A timeout does not cancel recording; use matching status to confirm
            the session before retrying.
        """
        if not self._check_client_connected():
            return False
        service_name = f"{self.cfg.recorder_name}/start_recording"
        try:
            service_available = service_name in self.ros_client.get_services()
        except Exception as error:
            self.logger.error(f"Recorder Start discovery failed: {error}")
            return False
        if not service_available:
            self.logger.error(f"Service {service_name} not found!")
            return False
        with self._status_lock:
            self._status_messages.pop("recorder", None)
            self._recorder_stop_status = None
        success, error_msg = self._call_service_result(
            service_name=service_name,
            timeout=5.0,
            service_type="robo_orchard_data_msg_ros2/srv/StartRecording",
            request_data=dict(destination=uri),
        )
        if error_msg:
            self.logger.error(error_msg)
        if success is True:
            self.logger.info("Recording session initialized!")
        return success

    def stop_recording(
        self, *, session: tuple[str, str] | None = None
    ) -> bool | None:
        """Request Stop, optionally retaining its session's terminal status.

        Args:
            session: This App's session ID and absolute destination to track.

        Returns:
            True for acceptance, False for rejection or failure before
            dispatch, and None for an unknown dispatched result.
        """
        if not self._check_client_connected():
            return False
        with self._status_lock:
            if session is not None:
                session_id, destination = session
                tracked = self._recorder_stop_status
                if (
                    tracked is None
                    or tracked["session_id"] != session_id
                    or tracked["destination"] != destination
                ):
                    self._recorder_stop_status = {
                        "session_id": session_id,
                        "destination": destination,
                    }
                received = self._status_messages.get("recorder")
                if (
                    received is not None
                    and received[1].get("session_id") == session_id
                    and received[1].get("destination") == destination
                    and received[1].get("data")
                    in {"completed", "idle", "failed"}
                    and self._recorder_stop_status.get("data") is None
                ):
                    self._recorder_stop_status = received[1]
            self._status_messages.pop("recorder", None)
        service_name = f"{self.cfg.recorder_name}/stop_recording"
        try:
            service_available = service_name in self.ros_client.get_services()
        except Exception as error:
            self.logger.error(f"Recorder Stop discovery failed: {error}")
            return False
        if not service_available:
            self.logger.error(f"Service {service_name} not found!")
            return False
        success, error_msg = self._call_service_result(
            service_name=service_name,
            timeout=5.0,
            service_type="std_srvs/srv/Trigger",
            request_data={},
        )
        if error_msg:
            self.logger.error(error_msg)
        if success is True:
            self.logger.info("Recording Stopped!")
        return success

    def cleanup(self) -> None:
        """Release client subscriptions without stopping the recorder."""
        self._stop_status_monitor()

    def record_handeye_calib_pose(self) -> bool:
        """Sends a request to record the current hand-eye calibration pose."""
        if self.cfg.record_handeye_calib_service_name is None:
            self.logger.error(
                "record_handeye_calib_service_name is not configured!"
            )
            return False
        return self._call_services(
            service_names=self.cfg.record_handeye_calib_service_name,
            success_msg="Hand-eye calibration pose recorded!",
        )

    def save_and_compute_handeye_calib(self) -> bool:
        """Sends a request to save and compute the hand-eye calibration."""
        if self.cfg.save_handeye_calib_service_name is None:
            self.logger.error(
                "save_handeye_calib_service_name is not configured!"
            )
            return False
        return self._call_services(
            service_names=self.cfg.save_handeye_calib_service_name,
            success_msg="Hand-eye calibration saved and computed!",
        )
