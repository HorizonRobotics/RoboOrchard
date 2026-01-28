set -ex

ros2 launch robo_orchard_teleop_ros2 piper_dagger_compat.launch.py \
    replay_time_s:=0.0
