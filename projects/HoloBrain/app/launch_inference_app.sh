set -ex

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")

python3 $SCRIPT_DIR/gen_inference_app_launch_config.py

robo-orchard inference-app launch \
    --server.address localhost \
    -- \
    --launch-config $SCRIPT_DIR/inference_app_launch_cfg.json \
    $1
