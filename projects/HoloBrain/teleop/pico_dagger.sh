set -ex

URDF_PATH=${PIPER_URDF_PATH:-${1:-}}
if [ -z "$URDF_PATH" ]; then
    echo "PIPER_URDF_PATH is required, or pass the URDF path as the first argument."
    exit 2
fi

if [ -z "${PIPER_URDF_PATH:-}" ] && [ "$#" -gt 0 ]; then
    shift
fi

ros2 launch robo_orchard_teleop_ros2 piper_pico_dagger_compat.launch.py \
    replay_time_s:="${REPLAY_TIME_S:-0.0}" \
    urdf_path:="$URDF_PATH" \
    "$@"
