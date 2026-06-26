set -ex

ros2 launch robo_orchard_teleop_ros2 piper_dagger_compat.launch.py \
    replay_time_s:="${REPLAY_TIME_S:-0.0}" \
    "$@"
