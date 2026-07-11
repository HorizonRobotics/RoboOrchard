set -ex

# Kill stale instance so re-running script will restart
pkill -TERM -f "[t]f_publisher" || true
sleep 1
pkill -KILL -f "[t]f_publisher" || true

ros2 run robo_orchard_data_ros2 tf_publisher \
    --ros-args
