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
from typing import Callable, Literal

import roslibpy

from robo_orchard_inference_app.config import ROSBridgeCfg
from robo_orchard_inference_app.logger import Logger
from robo_orchard_inference_app.state import InferenceState


class RosServiceHelper:
    """Encapsulates all interactions with ROS services."""

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
        self._is_recording = False
        atexit.register(self.cleanup)

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
            try:
                service = roslibpy.Service(
                    self.ros_client, service_name, service_type
                )
                request = roslibpy.ServiceRequest(request_data)
                result = service.call(request, timeout=timeout)
                if not result.get("success", False):
                    msg = result.get("message", "No message provided.")
                    self.logger.error(f"Service {service_name} failed: {msg}")
                    return False
            except roslibpy.core.RosTimeoutError:
                self.logger.error(f"Timeout calling service: {service_name}")
                return False
            except Exception as e:
                self.logger.error(f"Error calling {service_name}: {e}")
                return False

        self.logger.info(success_msg)
        if success_callback:
            success_callback()
        return True

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

    def enable_arm(self) -> bool:
        """Sends a request to enable the robot arm."""
        return self._call_services(
            service_names=self.cfg.enable_arm_service_name,
            success_msg="/EnableArm command sent successfully!",
            success_callback=lambda: setattr(
                self.state, "arm_ctrl_status", "enabled"
            ),
            timeout=25.0,
        )

    def disable_arm(self) -> bool:
        """Sends a request to disable the robot arm."""
        return self._call_services(
            service_names=self.cfg.disable_arm_service_name,
            success_msg="/DisableArm command sent successfully!",
            success_callback=lambda: setattr(
                self.state, "arm_ctrl_status", "disabled"
            ),
            timeout=25.0,
        )

    def reset_arm(self) -> bool:
        """Sends a request to reset the robot arm controllers to zero."""
        return self._call_services(
            service_names=self.cfg.reset_arm_service_name,
            success_msg="Robot arm controllers reset successfully!",
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
            success_callback=lambda: setattr(
                self.state, "is_inference_service_running", True
            ),
        )

    def disable_inference(self) -> bool:
        """Sends a request to disable the inference service."""
        return self._call_services(
            service_names=self.cfg.disable_inference_service_name,
            success_msg="Inference service disabled!",
            success_callback=lambda: setattr(
                self.state, "is_inference_service_running", False
            ),
        )

    def set_control_mode(
        self, mode: Literal["auto", "takeover", "stop"]
    ) -> bool:
        """Sets the robot's control mode."""
        service_map = {
            "auto": self.cfg.release_service_name,
            "takeover": self.cfg.takeover_service_name,
            "stop": self.cfg.stop_service_name,
        }
        message_map = {
            "auto": "/Release command sent successfully!",
            "takeover": "/TakeOver command sent successfully!",
            "stop": "/Stop command sent successfully!",
        }
        return self._call_services(
            service_names=service_map[mode],
            success_msg=message_map[mode],
            success_callback=lambda: setattr(self.state, "control_mode", mode),
        )

    def start_recording(self, uri: str) -> bool:
        flag = self._call_services(
            service_names=[f"{self.cfg.recorder_name}/start_recording"],
            success_msg="Recording Started!",
            timeout=5.0,
            service_type="robo_orchard_data_msg_ros2/srv/StartRecording",
            request_data=dict(destination=uri),
        )
        if flag:
            self._is_recording = True
        return flag

    def stop_recording(self) -> bool:
        flag = self._call_services(
            service_names=[f"{self.cfg.recorder_name}/stop_recording"],
            success_msg="Recording Stopped!",
            timeout=5.0,
        )
        if flag:
            self._is_recording = False
        return flag

    def cleanup(self):
        if self._is_recording:
            try:
                self.stop_recording()
            except:  # noqa: E722
                pass

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
