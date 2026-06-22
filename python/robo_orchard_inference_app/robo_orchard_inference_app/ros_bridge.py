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
import hashlib
import os
from typing import Callable, Literal

import roslibpy

from robo_orchard_inference_app.config import ROSBridgeCfg
from robo_orchard_inference_app.logger import Logger
from robo_orchard_inference_app.state import InferenceState

MIT_PARAM_NAMES = (
    "enable_mit_ctrl",
    "mit_kp",
    "mit_kd",
    "mit_vel_ref",
    "mit_torque_ref",
    "mit_velocity_feedforward",
    "mit_velocity_feedforward_source",
    "mit_gravity_compensation_enabled",
    "mit_gravity_compensation_urdf_path",
    "mit_gravity_compensation_scale",
    "mit_gravity_compensation_scale_per_joint",
    "mit_gravity_compensation_max_t_ref",
)


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
        self._last_requested_mit_params: (
            dict[str, bool | float | str] | None
        ) = None
        self._last_static_transform_fingerprint = None
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

    def get_node_names(self) -> list[str]:
        if not self._check_client_connected():
            return []
        try:
            return list(self.ros_client.get_nodes())
        except Exception as e:
            self.logger.error(f"Error getting ROS node names: {e}")
            return []

    @staticmethod
    def _static_transform_fingerprint(directory: str):
        abs_directory = os.path.abspath(directory)
        entries = []
        try:
            for name in sorted(os.listdir(abs_directory)):
                if not name.endswith(".json"):
                    continue
                path = os.path.join(abs_directory, name)
                if not os.path.isfile(path):
                    continue
                stat = os.stat(path)
                with open(path, "rb") as f:
                    digest = hashlib.sha256(f.read()).hexdigest()
                entries.append((name, stat.st_size, stat.st_mtime_ns, digest))
        except OSError:
            return (abs_directory, None)
        return (abs_directory, tuple(entries))

    def invalidate_static_transform_cache(self) -> None:
        self._last_static_transform_fingerprint = None

    def sync_static_transforms(self, episode_meta) -> bool:
        directory = getattr(episode_meta, "tf_directory", "")
        if not directory:
            return True

        service_name = self.cfg.static_transform_service_name
        if not service_name:
            self.logger.error("static_transform_service_name is not configured")
            return False

        fingerprint = self._static_transform_fingerprint(directory)
        if fingerprint == self._last_static_transform_fingerprint:
            return True

        ok = self._call_services(
            service_names=service_name,
            success_msg="Static transforms updated successfully!",
            timeout=5.0,
            service_type="robo_orchard_data_msg_ros2/srv/SetStaticTransforms",
            request_data={"directory": directory},
        )
        if ok:
            self._last_static_transform_fingerprint = fingerprint
        return ok

    def get_tf_publisher_startup_id(self) -> str | None:
        params, error = self._get_params(
            "/static_tf_publisher", param_names=("startup_id",)
        )
        if error:
            return None
        startup_id = params.get("startup_id")
        return startup_id if isinstance(startup_id, str) else None

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
            results = result.get("results", [])
            if results and all(
                item.get("successful", False) for item in results
            ):
                return True

            reasons = [item.get("reason", "") for item in results]
            reason = "; ".join(reason for reason in reasons if reason)
            self.logger.error(f"Failed to set parameter. {reason}".strip())
            return False
        except roslibpy.core.RosTimeoutError:
            self.logger.error(
                f"Timeout calling set parameter service: {set_param_service}"
            )
            return False
        except Exception as e:
            self.logger.error(f"Error calling {set_param_service}: {e}")
            return False

    @staticmethod
    def _python_value_to_parameter_value(
        value: bool | float | str | list
    ) -> dict:
        if isinstance(value, bool):
            return {"type": 1, "bool_value": value}
        if isinstance(value, str):
            return {"type": 4, "string_value": value}
        if isinstance(value, (list, tuple)):
            return {
                "type": 8,
                "double_array_value": [float(v) for v in value],
            }
        return {"type": 3, "double_value": float(value)}

    def _set_params(
        self, node_name: str, params: dict[str, bool | float | str | list]
    ) -> bool:
        request_data = {
            "parameters": [
                {
                    "name": name,
                    "value": self._python_value_to_parameter_value(value),
                }
                for name, value in params.items()
            ]
        }
        return self._set_param(node_name=node_name, request_data=request_data)

    @staticmethod
    def _parameter_value_to_python(value: dict) -> object:
        value_type = value.get("type")
        if value_type == 1:
            return value.get("bool_value", False)
        if value_type == 2:
            return value.get("integer_value", 0)
        if value_type == 3:
            return value.get("double_value", 0.0)
        if value_type == 4:
            return value.get("string_value", "")
        if value_type == 5:
            return value.get("byte_array_value", [])
        if value_type == 6:
            return value.get("bool_array_value", [])
        if value_type == 7:
            return value.get("integer_array_value", [])
        if value_type == 8:
            return value.get("double_array_value", [])
        if value_type == 9:
            return value.get("string_array_value", [])
        return None

    def _get_params(
        self,
        node_name: str,
        param_names: tuple[str, ...] = MIT_PARAM_NAMES,
        timeout: float = 5.0,
    ) -> tuple[dict[str, object], str | None]:
        if not self._check_client_connected():
            return {}, "ROS is not connected"

        available_services: list[str] = self.ros_client.get_services()
        service_type = "rcl_interfaces/srv/GetParameters"
        get_param_service = f"{node_name}/get_parameters"
        if get_param_service not in available_services:
            return {}, f"Parameter service {get_param_service} not found"

        try:
            service = roslibpy.Service(
                self.ros_client, get_param_service, service_type
            )
            request = roslibpy.ServiceRequest({"names": list(param_names)})
            result = service.call(request, timeout=timeout)
        except roslibpy.core.RosTimeoutError:
            return {}, f"Timeout calling parameter service {get_param_service}"
        except Exception as e:
            return {}, f"Error calling {get_param_service}: {e}"

        values = result.get("values", [])
        if len(values) != len(param_names):
            return {}, (
                f"Expected {len(param_names)} parameter values from "
                f"{get_param_service}, got {len(values)}"
            )

        params = {
            name: self._parameter_value_to_python(value)
            for name, value in zip(param_names, values, strict=True)
        }
        return params, None

    def get_mit_params_snapshot(self) -> dict[str, object]:
        mit_cfg = self.cfg.mit_control
        snapshot: dict[str, object] = {
            "param_names": list(MIT_PARAM_NAMES),
            "requested": self._last_requested_mit_params,
            "master": [],
            "follower": [],
        }

        for role, node_names in (
            ("master", mit_cfg.master_param_node_names),
            ("follower", mit_cfg.follower_param_node_names),
        ):
            entries = snapshot[role]
            assert isinstance(entries, list)
            for node_name in node_names:
                params, error = self._get_params(node_name)
                entry = {
                    "node_name": node_name,
                    "status": "error" if error else "confirmed",
                    "params": params,
                }
                if error:
                    entry["error"] = error
                entries.append(entry)

        return snapshot

    def set_mit_params(
        self,
        master_enabled: bool,
        master_kp: float,
        master_kd: float,
        master_vel_ref: float,
        master_torque_ref: float,
        follower_enabled: bool,
        follower_kp: float,
        follower_kd: float,
        follower_vel_ref: float,
        follower_torque_ref: float,
        follower_velocity_feedforward: bool | None = None,
        follower_gravity_compensation_alpha: float | None = None,
        follower_gravity_compensation_enabled: bool | None = None,
        follower_gravity_compensation_urdf_path: str | None = None,
        follower_gravity_compensation_scale: float | None = None,
        follower_gravity_compensation_scale_per_joint: (
            list[float] | None
        ) = None,
        follower_gravity_compensation_max_t_ref: float | None = None,
    ) -> bool:
        mit_cfg = self.cfg.mit_control
        requested_params: list[
            tuple[str, dict[str, bool | float | str | list]]
        ] = []
        for node_name in mit_cfg.master_param_node_names:
            requested_params.append(
                (
                    node_name,
                    {
                        "enable_mit_ctrl": master_enabled,
                        "mit_kp": master_kp,
                        "mit_kd": master_kd,
                        "mit_vel_ref": master_vel_ref,
                        "mit_torque_ref": master_torque_ref,
                    },
                )
            )
        follower_params: dict[str, bool | float | str | list] = {
            "enable_mit_ctrl": follower_enabled,
            "mit_kp": follower_kp,
            "mit_kd": follower_kd,
            "mit_vel_ref": follower_vel_ref,
            "mit_torque_ref": follower_torque_ref,
        }
        if follower_velocity_feedforward is not None:
            follower_params["mit_velocity_feedforward"] = bool(
                follower_velocity_feedforward
            )
        if follower_gravity_compensation_alpha is not None:
            follower_gravity_compensation_alpha = float(
                follower_gravity_compensation_alpha
            )
            # Gravity compensation is always enabled; alpha == 0 is the
            # no-op (the scale multiplies the computed torque to zero), so
            # there is no separate enable/scale state to keep in sync.
            follower_gravity_compensation_enabled = True
            follower_gravity_compensation_scale = (
                follower_gravity_compensation_alpha
            )
        if follower_gravity_compensation_enabled is not None:
            follower_params[
                "mit_gravity_compensation_enabled"
            ] = follower_gravity_compensation_enabled
        if follower_gravity_compensation_urdf_path is not None:
            follower_params["mit_gravity_compensation_urdf_path"] = (
                follower_gravity_compensation_urdf_path
            )
        if follower_gravity_compensation_scale is not None:
            follower_params["mit_gravity_compensation_scale"] = (
                follower_gravity_compensation_scale
            )
        if follower_gravity_compensation_scale_per_joint is not None:
            follower_params["mit_gravity_compensation_scale_per_joint"] = [
                float(v)
                for v in follower_gravity_compensation_scale_per_joint
            ]
        if follower_gravity_compensation_max_t_ref is not None:
            follower_params["mit_gravity_compensation_max_t_ref"] = (
                follower_gravity_compensation_max_t_ref
            )
        for node_name in mit_cfg.follower_param_node_names:
            requested_params.append((node_name, dict(follower_params)))

        if not requested_params:
            self.logger.warning("No MIT parameter nodes are configured.")
            return True

        for node_name, params in requested_params:
            if not self._set_params(node_name, params):
                return False

        self._last_requested_mit_params = {
            "master_enabled": master_enabled,
            "master_kp": master_kp,
            "master_kd": master_kd,
            "master_vel_ref": master_vel_ref,
            "master_torque_ref": master_torque_ref,
            "follower_enabled": follower_enabled,
            "follower_kp": follower_kp,
            "follower_kd": follower_kd,
            "follower_vel_ref": follower_vel_ref,
            "follower_torque_ref": follower_torque_ref,
        }
        if follower_gravity_compensation_alpha is not None:
            self._last_requested_mit_params[
                "follower_gravity_compensation_alpha"
            ] = follower_gravity_compensation_alpha
        if follower_gravity_compensation_enabled is not None:
            self._last_requested_mit_params[
                "follower_gravity_compensation_enabled"
            ] = follower_gravity_compensation_enabled
        if follower_gravity_compensation_urdf_path is not None:
            self._last_requested_mit_params[
                "follower_gravity_compensation_urdf_path"
            ] = follower_gravity_compensation_urdf_path
        if follower_gravity_compensation_scale is not None:
            self._last_requested_mit_params[
                "follower_gravity_compensation_scale"
            ] = follower_gravity_compensation_scale
        if follower_gravity_compensation_scale_per_joint is not None:
            self._last_requested_mit_params[
                "follower_gravity_compensation_scale_per_joint"
            ] = list(follower_gravity_compensation_scale_per_joint)
        if follower_gravity_compensation_max_t_ref is not None:
            self._last_requested_mit_params[
                "follower_gravity_compensation_max_t_ref"
            ] = follower_gravity_compensation_max_t_ref

        self.logger.info(
            "MIT params set: "
            f"master enabled={master_enabled}, "
            f"master kp={master_kp}, master kd={master_kd}, "
            f"master vel_ref={master_vel_ref}, "
            f"master torque_ref={master_torque_ref}, "
            f"follower enabled={follower_enabled}, "
            f"follower kp={follower_kp}, follower kd={follower_kd}, "
            f"follower vel_ref={follower_vel_ref}, "
            f"follower torque_ref={follower_torque_ref}, "
            "follower gravity_compensation_alpha="
            f"{follower_gravity_compensation_alpha}, "
            "follower gravity_compensation_enabled="
            f"{follower_gravity_compensation_enabled}, "
            "follower gravity_compensation_urdf_path="
            f"{follower_gravity_compensation_urdf_path}, "
            "follower gravity_compensation_scale="
            f"{follower_gravity_compensation_scale}, "
            "follower gravity_compensation_scale_per_joint="
            f"{follower_gravity_compensation_scale_per_joint}, "
            "follower gravity_compensation_max_t_ref="
            f"{follower_gravity_compensation_max_t_ref}"
        )
        return True

    def set_follower_gravity_compensation_alpha(
        self, alpha: float
    ) -> bool:
        alpha = float(alpha)
        # Gravity compensation stays enabled; alpha == 0 yields zero torque.
        enabled = True
        mit_cfg = self.cfg.mit_control
        if not mit_cfg.follower_param_node_names:
            return True
        params: dict[str, bool | float | str | list] = {
            "mit_gravity_compensation_enabled": enabled,
            "mit_gravity_compensation_scale": alpha,
        }
        # Apply the per-joint scale alongside, so the firmware's ~4x execution
        # on joints 1-3 is corrected whenever gravity comp is toggled on.
        per_joint = list(
            mit_cfg.default_follower_gravity_compensation_scale_per_joint
        )
        if per_joint:
            params["mit_gravity_compensation_scale_per_joint"] = [
                float(v) for v in per_joint
            ]
        for node_name in mit_cfg.follower_param_node_names:
            if not self._set_params(node_name, params):
                return False
        self.logger.info(
            "Follower gravity compensation "
            f"alpha={alpha}, enabled={enabled}, "
            f"scale_per_joint={per_joint}"
        )
        return True

    def set_follower_gravity_compensation(self, enabled: bool) -> bool:
        return self.set_follower_gravity_compensation_alpha(
            1.0 if enabled else 0.0
        )

    def set_follower_gravity_record_dir(
        self, directory: str | None
    ) -> bool:
        """Point per-timestep gravity recording at an episode ``directory``.

        Writes one ``gravity_<node>.csv`` per follower node into
        ``directory``. Passing ``None``/empty disables recording on the
        follower nodes.
        """
        mit_cfg = self.cfg.mit_control
        if not mit_cfg.follower_param_node_names:
            return True
        ok = True
        for node_name in mit_cfg.follower_param_node_names:
            if directory:
                label = node_name.strip("/").split("/")[-1] or "follower"
                path = os.path.join(directory, f"gravity_{label}.csv")
            else:
                path = ""
            if not self._set_params(
                node_name,
                {"mit_gravity_compensation_record_path": path},
            ):
                ok = False
        self.logger.info(
            f"Follower gravity recording dir set to {directory!r}"
        )
        return ok

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

    def _push_reset_position(self, reset_position: list[float]) -> bool:
        """Pushes the given reset pose to the follower controllers."""
        if len(reset_position) != 7:
            self.logger.error(
                "reset_position must have 7 joint values, got "
                f"{len(reset_position)}."
            )
            return False
        request_data = {
            "parameters": [
                {
                    "name": "reset_position",
                    "value": {
                        "type": 8,
                        "double_array_value": [
                            float(v) for v in reset_position
                        ],
                    },
                }
            ]
        }
        ok = True
        for node_name in self.cfg.mit_control.follower_param_node_names:
            if not self._set_param(
                node_name=node_name, request_data=request_data
            ):
                ok = False
        return ok

    def reset_arm(self, position: list[float] | None = None) -> bool:
        """Sends a request to reset the robot arm controllers.

        Args:
            position: Optional reset pose (7 joint values) overriding the
                configured standard reset pose for this call only. The pose
                is pushed to the controllers on every reset, so one caller's
                override never leaks into the next reset.
        """
        if position:
            reset_position = list(position)
        elif self.cfg.reset_position:
            reset_position = list(self.cfg.reset_position)
        else:
            reset_position = [0.0] * 7
        if not self._push_reset_position(reset_position):
            return False
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
        self,
        mode: Literal["auto", "takeover", "stop"],
        mit_params: dict[str, bool | float] | None = None,
    ) -> bool:
        """Sets the robot's control mode."""
        if mode == "takeover" and mit_params is not None:
            if not self.set_mit_params(**mit_params):
                return False

        service_map = {
            "auto": self.cfg.release_service_name,
            "takeover": self.cfg.takeover_service_name,
            "stop": self.cfg.stop_service_name,
        }
        message_map = {
            "auto": "/Release command sent successfully!",
            "takeover": "/TakeOver command sent! Use the hardware teach button to enter teach mode.",  # noqa: E501
            "stop": "/Stop command sent successfully!",
        }
        return self._call_services(
            service_names=service_map[mode],
            success_msg=message_map[mode],
            success_callback=lambda: setattr(self.state, "control_mode", mode),
            timeout=30.0,
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
