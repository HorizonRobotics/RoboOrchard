set -ex

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

python3 $SCRIPT_DIR/gen_data_recorder_config.py

ros2 run robo_orchard_data_ros2 mcap_recorder_service \
    --ros-args \
    -p config_file:="$SCRIPT_DIR/data_recorder.json"
