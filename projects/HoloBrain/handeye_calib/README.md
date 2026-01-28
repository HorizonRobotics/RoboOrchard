# 1. Setup  
Setup ./launch_handeye_calib.sh, you should modify following arguments to your own setting.
- `MODE`: calibration mode, "eye_in_hand" or "eye_to_hand".
- `ARUCO_MARKER_SIZE`: aruco marker size you used.
- `ARUCO_MARKER_ID`: aruco marker ID you used.
- `CAMERA_FRAME_NAME`: camera color frame ready to be calibrated.
- `ARUCO_MARKER_FRAME_NAME`: you could make it default as "marker_frame".
- `CAMERA_INFO_TOPIC`: camera_info topic name in ros2.
- `CAMERA_RAW_TOPIC`: camera_raw topic name in ros2.
- `END_EFFECTOR_FRAME_NAME`: end effector frame link name in urdf file.
- `BASE_FRAME_NAME`: base frame link name in urdf file.
- `END_EFFECTOR_POSE_TOPIC`: end effector topic name in ros2.
- `RESULT_FILE`: calibration result file name.

# 2. Launch calib node
```bash
bash launch_handeye_calib.sh
```

# 3. Calibrate  
Move your robot arm, record and save pose pairs in robo_orchard_inference_app, you could get result.