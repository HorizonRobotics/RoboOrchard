set -ex

if [ -n "${REPLAY_TIME_S:-}" ]; then
    echo "REPLAY_TIME_S is ignored;" \
        "set replay_time_s in the Control Manager config." >&2
fi

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

TELEOP_SOURCE=aloha python3 "$SCRIPT_DIR/gen_control_manager_config.py"

ros2 launch robo_orchard_teleop_ros2 piper_dagger_compat.launch.py \
    control_manager_config_file:="$SCRIPT_DIR/control_manager.json" \
    "$@"
