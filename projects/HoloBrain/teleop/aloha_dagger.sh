set -ex

# Kill stale instance so re-running script will restart
pkill -TERM -f "[s]ingle_ctrl|[t]ake_over|[a]loha_orchestrator|[p]iper_dagger_compat.launch.py" || true
sleep 2
pkill -KILL -f "[s]ingle_ctrl|[t]ake_over|[a]loha_orchestrator|[p]iper_dagger_compat.launch.py" || true

# Follower gravity compensation needs pinocchio in the controller venv
# Image's CV bridge path needs these exact versions (temporary)
/opt/robot-venv/bin/python3 -c "import pinocchio" 2>/dev/null || \
    /opt/robot-venv/bin/pip install "numpy==1.26.4" "pin==2.7.0"

# Post-launch verification
(
  set +x
  for _ in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q /robot/left/robot_left_controller \
        && ros2 node list 2>/dev/null | grep -q /robot/right/robot_right_controller; then
      break
    fi
    sleep 2
  done
  ok=1
  for n in /robot/left/robot_left_controller /robot/right/robot_right_controller; do
    table=$(ros2 param get "$n" mit_gravity_compensation_deflection_table 2>/dev/null || true)
    case "$table" in
      *'"2"'*) echo "[verify] $n: deflection table loaded" ;;
      *) echo "[verify] $n: MISSING deflection table!"; ok=0 ;;
    esac
  done
  if [ "$ok" = 1 ]; then
    echo "[verify] gravity compensation configured."
  else
    echo "[verify] WARNING: gravity compensation NOT fully configured."
  fi
) &

ros2 launch robo_orchard_teleop_ros2 piper_dagger_compat.launch.py \
    replay_time_s:="${REPLAY_TIME_S:-0.0}" \
    "$@"
