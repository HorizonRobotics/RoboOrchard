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
import os
import subprocess
from datetime import datetime

import psutil
import roslibpy
import streamlit as st


def time_str_now() -> str:
    """Generates a timestamp string in 'YYYY_MM_DD-HH_MM_SS' format.

    Returns:
        str: Current timestamp in the format 'YYYY_MM_DD-HH_MM_SS'
    """
    return datetime.now().strftime("%Y_%m_%d-%H_%M_%S")


def remove_path(path: str) -> None:
    """Removes a file or directory at the specified path.

    Args:
        path (str): Path to the file or directory to be removed

    Returns:
        None

    Notes:
        - If the path doesn't exist, the function returns silently
        - Uses shutil.rmtree for directories and os.remove for files
    """
    if not os.path.exists(path):
        return
    if os.path.isdir(path):
        import shutil

        shutil.rmtree(path)
    else:
        os.remove(path)


def check_process(process: subprocess.Popen, min_live_time: float = 5):
    """Checks if a process is running successfully for a minimum time.

    Args:
        process (subprocess.Popen): The process to check.
        min_live_time (float): Minimum time in seconds to wait before
            checking (default: 5).

    Returns:
        bool: True if process is running or completed successfully.

    Raises:
        subprocess.CalledProcessError: If process exits with non-zero
        return code.
    """
    try:
        r_code = process.wait(min_live_time)
        if r_code == 0:
            return True
        else:
            out_msgs, err_msgs = process.communicate()
            process.terminate()
            raise subprocess.CalledProcessError(
                returncode=r_code,
                cmd=[],
                output=out_msgs,
                stderr=err_msgs,
            )
    except subprocess.TimeoutExpired:
        return True


def start_process(
    *args,
    stop_signal: int = psutil.signal.SIGINT,
    min_live_time: float = 5,
    redirect: bool = True,
    **kwargs,
) -> subprocess.Popen:
    """Starts a subprocess.

    Args:
        *args: Variable length argument list passed to subprocess.Popen
        stop_signal (int): Signal to use when stopping the
            process (default: SIGINT)
        min_live_time (float): Minimum time in seconds to verify
            process stability (default: 5)
        redirect (bool): Whether to redirect stdout and stderr to
            pipes (default: True)
        **kwargs: Additional keyword arguments passed to subprocess.Popen

    Returns:
        subprocess.Popen: The started process object

    Notes:
        - Registers process cleanup with atexit
        - Sets process group ID for proper signal handling
    """

    if redirect:
        kwargs.update(
            {
                "stdout": subprocess.PIPE,
                "stderr": subprocess.PIPE,
            }
        )

    process = subprocess.Popen(*args, preexec_fn=os.setsid, **kwargs)
    atexit.register(stop_process, pid=process.pid, signal=stop_signal)
    if min_live_time <= 0:
        return process
    else:
        check_process(process, min_live_time)
        return process


def stop_process(
    pid: int | subprocess.Popen,
    signal: int = psutil.signal.SIGINT,
    timeout: float = 60,
    force: bool = True,
) -> bool:
    """Stops a process or process group with optional force killing.

    Args:
        pid (int | subprocess.Popen): Process ID or Popen object to stop
        signal (int): Signal to send to the process (default: SIGINT)
        timeout (float): Maximum time in seconds to wait for process
            termination (default: 60)
        force (bool): Whether to force kill with SIGKILL if normal
            termination fails (default: True)

    Returns:
        bool: True if process was stopped successfully

    Raises:
        Exception: If process termination fails and force is False
    """

    if isinstance(pid, subprocess.Popen):
        pid = pid.pid

    if not psutil.pid_exists(pid):
        return True

    gid = os.getpgid(pid)

    try:
        proc = psutil.Process(pid)
        os.killpg(gid, signal)
        proc.wait(timeout=timeout)
        return True
    except psutil.NoSuchProcess:
        return True
    except Exception as e:  # noqa
        if force:
            os.killpg(gid, psutil.signal.SIGKILL)
            return True
        raise


class RosBridgeServerConnectedError(Exception):
    pass


def terminate_ros_bridge_client(client: roslibpy.Ros):
    if client.is_connected:
        client.terminate()


@st.cache_resource
def create_ros_bridge_client(
    host: str = "localhost", port: int = 9090
) -> roslibpy.Ros:
    client = roslibpy.Ros(host=host, port=port)
    client.run(timeout=10)

    if not client.is_connected:
        raise RosBridgeServerConnectedError

    atexit.register(terminate_ros_bridge_client, client=client)

    return client
