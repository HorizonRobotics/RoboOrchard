#!/bin/bash
set -e

if [ -n "${REPLAY_TIME_S:-}" ]; then
    echo "REPLAY_TIME_S is ignored;" \
        "set replay_time_s in the Control Manager config." >&2
fi

URDF_PATH=${PIPER_URDF_PATH:-${1:-}}
if [ -z "$URDF_PATH" ]; then
    echo "PIPER_URDF_PATH is required, or pass the URDF path as the first argument." >&2
    exit 2
fi
if [ -z "${CHASSIS_URL:-}" ] || [ -z "${LIFT_URL:-}" ]; then
    echo "CHASSIS_URL and LIFT_URL are required with Hexfellow." >&2
    exit 2
fi

if [ -z "${PIPER_URDF_PATH:-}" ] && [ "$#" -gt 0 ]; then
    shift
fi

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

TELEOP_SOURCE=pico WITH_HEXFELLOW=true \
    python3 "$SCRIPT_DIR/gen_control_manager_config.py"

ros2 launch robo_orchard_teleop_ros2 \
    hexfellow_piper_pico_teleop_compat.launch.py \
    urdf_path:="$URDF_PATH" \
    chassis_url:="$CHASSIS_URL" \
    lift_url:="$LIFT_URL" \
    control_manager_config_file:="$SCRIPT_DIR/control_manager.json" \
    "$@"
