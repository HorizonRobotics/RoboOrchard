set -ex

if [ -n "${REPLAY_TIME_S:-}" ]; then
    echo "REPLAY_TIME_S is ignored;" \
        "set replay_time_s in the Control Manager config." >&2
fi

URDF_PATH=${PIPER_URDF_PATH:-${1:-}}
if [ -z "$URDF_PATH" ]; then
    echo "PIPER_URDF_PATH is required, or pass the URDF path as the first argument."
    exit 2
fi

if [ -z "${PIPER_URDF_PATH:-}" ] && [ "$#" -gt 0 ]; then
    shift
fi

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

TELEOP_SOURCE=pico python3 "$SCRIPT_DIR/gen_control_manager_config.py"

ros2 launch robo_orchard_teleop_ros2 piper_pico_dagger_compat.launch.py \
    urdf_path:="$URDF_PATH" \
    control_manager_config_file:="$SCRIPT_DIR/control_manager.json" \
    "$@"
