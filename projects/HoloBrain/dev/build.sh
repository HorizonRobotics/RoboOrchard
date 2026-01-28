set -e

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

docker build -t horizonrobotics/holobrain:ubuntu22.04-py3.10-ros-humble-torch2.8.0 \
    $SCRIPT_DIR/ \
    --network host \
    --build-arg ROS_DISTRO=humble
