set -e

SESSION_NAME="holobrain"
DOCKER_CONTAINER_NAME="holobrain"

if [ "$(docker ps -aq -f name=^/${DOCKER_CONTAINER_NAME}$)" ]; then
    echo "Try to stoping container..."
    docker stop $DOCKER_CONTAINER_NAME >/dev/null 2>&1
fi

tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? -eq 0 ]; then
    echo "Try to closing tumxp workflow..."
    tmux kill-session -t $SESSION_NAME
fi
