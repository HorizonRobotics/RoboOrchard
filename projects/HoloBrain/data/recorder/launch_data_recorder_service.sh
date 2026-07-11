set -ex

# Kill stale instance so re-running script will restart
pkill -TERM -f "[m]cap_recorder_service" || true
sleep 1
pkill -KILL -f "[m]cap_recorder_service" || true

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

python3 $SCRIPT_DIR/gen_data_recorder_config.py

ros2 run robo_orchard_data_ros2 mcap_recorder_service \
    --ros-args \
    -p config_file:="$SCRIPT_DIR/data_recorder.json"
