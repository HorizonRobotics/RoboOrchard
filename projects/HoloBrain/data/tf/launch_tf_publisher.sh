set -ex

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

python3 $SCRIPT_DIR/gen_tf_config.py

ros2 run robo_orchard_data_ros2 tf_publisher \
    --ros-args \
    -p config_file:="$SCRIPT_DIR/tf_publisher.json"
