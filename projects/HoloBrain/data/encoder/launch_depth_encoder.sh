set -ex

# Kill stale instance so re-running script will restart
pkill -TERM -f "[i]mage_encoder.*depth_encoding" || true
sleep 1
pkill -KILL -f "[i]mage_encoder.*depth_encoding" || true

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

python3 $SCRIPT_DIR/gen_depth_encoding_config.py

ros2 run robo_orchard_data_ros2 image_encoder \
    --ros-args \
    -p config_file:="$SCRIPT_DIR/depth_encoding.json"
