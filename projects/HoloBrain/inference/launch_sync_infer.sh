set -ex

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

python3 $SCRIPT_DIR/gen_sync_config.py

ros2 launch robo_orchard_deploy_ros2 launch_agilex_sync.launch.py \
    config_file:="$SCRIPT_DIR/sync_inference.json"
