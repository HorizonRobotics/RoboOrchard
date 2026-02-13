set -ex

DOCKER_NAME=${1:-holobrain}

echo ">>> Launching container: $DOCKER_NAME"

docker run -itd \
    --gpus all \
    --name "$DOCKER_NAME" \
    --user root \
    --shm-size=128g \
    --ipc host \
    --network host \
    --pid host \
    --rm \
    --privileged \
    -e USER=$USER \
    -e DOCKER_USER=$USER \
    -e PYTHONUNBUFFERED=1 \
    -e ROS_DOMAIN_ID=2 \
    -e ROS2_INSTALL_PATH=/opt/ros/humble/ \
    -e no_proxy="localhost,127.0.0.1" \
    -w /home/users/$USER \
    -v /dev:/dev \
    -v ~/data:/data:rw \
    -v ~/workspace/docker_env:/home/users/$USER:rw \
    horizonrobotics/holobrain:v0-ubuntu22.04-py3.10-ros-humble-torch2.8.0
