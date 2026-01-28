set -ex

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

python3 $SCRIPT_DIR/gen_image_encoding_config.py

ros2 run robo_orchard_data_ros2 image_encoder \
    --ros-args \
    -p config_file:="$SCRIPT_DIR/image_encoding.json"
