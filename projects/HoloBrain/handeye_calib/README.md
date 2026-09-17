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
- `RESULT_FILE`: exact calibration result file path; must not already exist.

# 2. Launch calib node
```bash
bash launch_handeye_calib.sh
```

# 3. Calibrate  
Move your robot arm, record and save pose pairs in robo_orchard_inference_app, you could get result.

After a successful save, the node clears the collected poses, count and current
observation cache. Record new poses to calibrate again with the same
configuration. Failed saves retain the samples. Set `publish_tf`
to `true` in the hand-eye calibration configuration to also publish the result
to the TF tree; it defaults to `false`.

To discard the current round without saving, call:

```bash
ros2 service call /handeye_calib/reset_data std_srvs/srv/Trigger '{}'
```

This clears samples, count and cached poses, including when already empty.
Existing result files and any previously published TF remain available.

## Output location

Specify exactly one of these configuration fields or generator arguments:

- `result_file` / `--result_file`: write only to this exact path. Its parent
  directory must exist. An existing file causes Save to fail without
  overwriting it, changing its name, or choosing another output location.
- `output_root` / `--output_root`: each Save allocates a new UTC timestamp
  directory, such as `20260917T083025.123456Z`, and writes `result.json` inside.
  The root is created if needed. Directory names include microseconds; an
  existing directory causes Save to fail without reusing it or adding a suffix.

For repeated calibration sessions, replace `--result_file $RESULT_FILE` in
`launch_handeye_calib.sh` with `--output_root /your/calibration/output`.
Both modes return the actual file path on success and clear the samples.
Filesystem errors return `success=false` and preserve the samples. There is
no automatic numbering or fallback path.

If optional TF publication fails after writing the file, Save returns failure
with the saved path and retains the samples. The file remains available; an
exact-path retry will not overwrite it. Fix the TF issue and choose a new
output path, or use `reset_data` to discard the retained session.
