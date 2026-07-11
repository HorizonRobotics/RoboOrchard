set -ex

# Kill stale instance so re-running script will restart
pkill -TERM -f "[r]obo-orchard inference-app launch" || true
sleep 1
pkill -KILL -f "[r]obo-orchard inference-app launch" || true

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

python3 $SCRIPT_DIR/gen_inference_app_launch_config.py
python3 $SCRIPT_DIR/gen_inference_app_task_config.py

robo-orchard inference-app launch \
    --server.address localhost \
    -- \
    --launch-config $SCRIPT_DIR/inference_app_launch_cfg.json \
    --task-config $SCRIPT_DIR/inference_app_task_cfg.json \
    "$@"
