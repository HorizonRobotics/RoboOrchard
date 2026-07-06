#!/usr/bin/env bash
# Restart the HoloBrain data/robot/app panes with gravity compensation.
#
# NOTE: since the calibrated deflection-compensation defaults moved into
# piper_dagger_compat.launch.py (and the app defaults into the inference
# app's config.py), a plain
#
#     robot-lab-nexus launch --no-pull --mount ~/data:/data \
#         --mount ~/roboorchard-work/roboorchard-dev:/opt/roboorchard \
#         --mode inference --record
#
# already brings the stack up fully configured. Use this script only to
# RESTART panes mid-session (e.g. after rebuilding the ros2 package or
# editing controller code) without tearing down the tmux session.
#
# Optional: export PIPER_GRAVITY_URDF_PATH to use a non-default gravity
# URDF (flows into the controllers via generated/teleop/dagger.sh).
set -euo pipefail

if ! docker ps --format "{{.Names}}" | grep -qx holobrain; then
  echo "holobrain container is not running. Start it first with robot-lab-nexus launch."
  exit 1
fi

GRAVITY_URDF_PATH="${PIPER_GRAVITY_URDF_PATH:-/data/holobrain/urdf/piper_x_description_dualarm_v2.urdf}"
DOCKER_BASE="docker exec -it -e TERM=xterm-256color -e NEXUS_GENERATED=/opt/robot-lab-nexus/generated -e NEXUS_CACHE_DIR=/opt/robot-lab-nexus/cache -e NEXUS_CAMERA_MODEL=d405 -e NEXUS_CAMERA_PROFILE=/opt/robot-lab-nexus/generated/camera_profile.json -e LEFT_WRIST_CAMERA_SERIAL_NO=335122270662 -e RIGHT_WRIST_CAMERA_SERIAL_NO=353322271383 -e ENV_CENTER_CAMERA_SERIAL_NO=246322301143 -e PIPER_GRAVITY_URDF_PATH=$GRAVITY_URDF_PATH holobrain /bin/bash -lc"
SOURCE_ALL='source /opt/ros/humble/setup.bash && source /opt/foxglove_bridge/install/setup.bash && source /opt/aruco_ros/install/setup.bash && source /opt/roboorchard/ros2_package/install/setup.bash && source /opt/robot-venv/bin/activate && cd /opt/robot-lab-nexus/generated'
DATA_IMAGE_CMD="$DOCKER_BASE '$SOURCE_ALL && bash data/encoder/launch_image_encoder.sh'"
DATA_DEPTH_CMD="$DOCKER_BASE '$SOURCE_ALL && bash data/encoder/launch_depth_encoder.sh'"
DATA_TF_CMD="$DOCKER_BASE '$SOURCE_ALL && bash data/tf/launch_tf_publisher.sh'"
DATA_RECORDER_CMD="$DOCKER_BASE '$SOURCE_ALL && bash data/recorder/launch_data_recorder_service.sh'"
ROBOT_CMD="$DOCKER_BASE '$SOURCE_ALL && bash rename-can.sh && bash teleop/dagger.sh'"
APP_CMD="$DOCKER_BASE '$SOURCE_ALL && bash app/launch_inference_app.sh'"

# Gravity compensation needs pinocchio (dagger.sh also self-checks this;
# kept here so a restart surfaces install errors early). pin>=3/numpy 2.x
# break the image's CV bridge path - keep these exact versions.
echo "Ensuring NumPy-1-compatible Pinocchio in /opt/robot-venv..."
docker exec holobrain bash -lc '/opt/robot-venv/bin/python3 -c "import pinocchio" 2>/dev/null || /opt/robot-venv/bin/pip install "numpy==1.26.4" "pin==2.7.0"'

echo "Stopping stale data and Piper processes..."
docker exec holobrain bash -lc 'set +e; pkill -TERM -f "[i]mage_encoder|[m]cap_recorder_service|[t]f_publisher|[s]ingle_ctrl|[t]ake_over|[a]loha_orchestrator|[p]iper_dagger_compat.launch.py"; sleep 2; pkill -KILL -f "[i]mage_encoder|[m]cap_recorder_service|[t]f_publisher|[s]ingle_ctrl|[t]ake_over|[a]loha_orchestrator|[p]iper_dagger_compat.launch.py"; true'

echo "Restarting data-service panes..."
if tmux list-windows -t holobrain -F '#{window_name}' | grep -Fxq 'data service'; then
  tmux kill-window -t 'holobrain:data service'
fi
tmux new-window -t holobrain:2 -n 'data service' "$DATA_IMAGE_CMD"
tmux split-window -t 'holobrain:data service.0' "$DATA_DEPTH_CMD"
tmux split-window -t 'holobrain:data service.0' "$DATA_TF_CMD"
tmux split-window -t 'holobrain:data service.0' "$DATA_RECORDER_CMD"
tmux select-layout -t 'holobrain:data service' tiled

sleep 5

echo "Starting robot-control pane..."
if tmux list-windows -t holobrain -F '#{window_name}' | grep -Fxq 'robot control'; then
  tmux respawn-pane -k -t 'holobrain:robot control.0' "$ROBOT_CMD"
else
  tmux new-window -t holobrain:3 -n 'robot control' "$ROBOT_CMD"
fi

echo "Waiting for robot controller nodes..."
for i in $(seq 1 40); do
  if docker exec holobrain bash -lc 'source /opt/ros/humble/setup.bash; source /opt/roboorchard/ros2_package/install/setup.bash; ros2 node list | grep -q /robot/left_master/robot_left_master_controller && ros2 node list | grep -q /robot/left/robot_left_controller && ros2 node list | grep -q /robot/right/robot_right_controller'; then
    break
  fi
  if [ "$i" = 40 ]; then
    echo "Timed out waiting for controller nodes. Current robot-control pane output:" >&2
    tmux capture-pane -p -t 'holobrain:robot control.0' >&2 || true
    exit 1
  fi
  sleep 1
done

echo "Verifying follower gravity-compensation configuration..."
docker exec holobrain bash -lc 'source /opt/ros/humble/setup.bash; source /opt/roboorchard/ros2_package/install/setup.bash; ok=1; for n in /robot/left/robot_left_controller /robot/right/robot_right_controller; do enabled=$(ros2 param get "$n" mit_gravity_compensation_enabled); stiff=$(ros2 param get "$n" mit_gravity_compensation_offset_stiffness); table=$(ros2 param get "$n" mit_gravity_compensation_deflection_table); echo "$n: $enabled | stiffness ${stiff#Double values are: } | table ${#table} chars"; case "$table" in *\"2\"*) ;; *) echo "  MISSING deflection table!"; ok=0;; esac; done; [ "$ok" = 1 ]'

echo "Waiting for color compressed image publishers..."
for i in $(seq 1 40); do
  if docker exec holobrain bash -lc 'source /opt/ros/humble/setup.bash; source /opt/roboorchard/ros2_package/install/setup.bash; ros2 topic info /left_camera/color/image_raw/compressed_data 2>/dev/null | grep -q "Publisher count: 1" && ros2 topic info /middle_camera/color/image_raw/compressed_data 2>/dev/null | grep -q "Publisher count: 1" && ros2 topic info /right_camera/color/image_raw/compressed_data 2>/dev/null | grep -q "Publisher count: 1"'; then
    break
  fi
  if [ "$i" = 40 ]; then
    echo "Timed out waiting for compressed color publishers. Image encoder pane:" >&2
    tmux capture-pane -p -t 'holobrain:data service.0' >&2 || true
    exit 1
  fi
  sleep 1
done

echo "Stopping stale inference-app processes..."
docker exec holobrain bash -lc 'set +e; pkill -TERM -f "[r]obo-orchard inference-app launch|[l]aunch_inference_app.sh"; sleep 2; pkill -KILL -f "[r]obo-orchard inference-app launch|[l]aunch_inference_app.sh"; true'
if tmux list-windows -t holobrain -F '#{window_name}' | grep -Fxq 'inference'; then
  tmux kill-window -t 'holobrain:inference'
fi

echo "Restarting app pane..."
if tmux list-windows -t holobrain -F '#{window_name}' | grep -Fxq 'app'; then
  tmux respawn-pane -k -t 'holobrain:app.0' "$APP_CMD"
else
  tmux new-window -t holobrain:4 -n app "$APP_CMD"
fi

echo "Done. Controllers launched with calibrated gravity compensation enabled (deflection tables from piper_dagger_compat.launch.py, follower kp=25)."
