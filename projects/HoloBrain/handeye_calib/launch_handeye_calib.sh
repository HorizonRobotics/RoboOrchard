set -ex

SCRIPT_REAL_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_REAL_PATH")
MODE="eye_in_hand"
ARUCO_MARKER_SIZE=0.1
ARUCO_MARKER_ID=100
CAMERA_FRAME_NAME="camera_frame"
ARUCO_MARKER_FRAME_NAME="marker_frame"
CAMERA_INFO_TOPIC="/your/camera/info/topic"
CAMERA_RAW_TOPIC="/your/camera/raw/topic"
END_EFFECTOR_FRAME_NAME="link6"
BASE_FRAME_NAME="base_link"
END_EFFECTOR_POSE_TOPIC="/your/end_effector/pose/topic"
RESULT_FILE="your/result/file.json"

python3 $SCRIPT_DIR/gen_handeye_calib_config.py \
    --mode $MODE \
    --camera_frame_name $CAMERA_FRAME_NAME \
    --marker_frame $ARUCO_MARKER_FRAME_NAME \
    --base_frame_name $BASE_FRAME_NAME \
    --end_effector_frame_name $END_EFFECTOR_FRAME_NAME \
    --end_effector_pose_topic_name $END_EFFECTOR_POSE_TOPIC \
    --result_file $RESULT_FILE

ros2 launch robo_orchard_handeye_calib_ros2 launch_handeye_calib.launch.py \
    aruco_marker_size:=$ARUCO_MARKER_SIZE \
    aruco_marker_id:=$ARUCO_MARKER_ID \
    camera_frame_name:=$CAMERA_FRAME_NAME \
    aruco_marker_frame_name:=$ARUCO_MARKER_FRAME_NAME \
    handeye_calib_config_file:=$SCRIPT_DIR/handeye_calib_config.json \
    camera_info_topic:=$CAMERA_INFO_TOPIC \
    camera_raw_topic:=$CAMERA_RAW_TOPIC