# robo_orchard_image_tools

ROS 2 image processing utilities for high-bandwidth image streams.

## compressed_image_encoder

`compressed_image_encoder` subscribes to raw `sensor_msgs/msg/Image` topics,
encodes them with OpenCV, and publishes `sensor_msgs/msg/CompressedImage`
topics. It is intended for multi-camera data collection paths where Python
message conversion and image encoding overhead can throttle raw image
publishers.

Example:

```bash
ros2 run robo_orchard_image_tools compressed_image_encoder --ros-args \
  -p input_topics:="[/camera/color/image_raw]" \
  -p output_topics:="[/camera/color/image_raw/compressed]" \
  -p codec:=jpeg \
  -p jpeg_quality:=95 \
  -p input_reliability:=best_effort \
  -p output_reliability:=reliable
```

Parameters:

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `input_topics` | string array | `[]` | Raw image topics to subscribe. |
| `output_topics` | string array | `[]` | Compressed image topics to publish. Must match `input_topics` length. |
| `codec` | string | `jpeg` | `jpeg` or `png`. |
| `jpeg_quality` | int | `95` | OpenCV JPEG quality, clamped to `[1, 100]`. |
| `png_compression` | int | `1` | OpenCV PNG compression, clamped to `[0, 9]`. |
| `input_reliability` | string | `best_effort` | `best_effort` or `reliable`. |
| `output_reliability` | string | `reliable` | `best_effort` or `reliable`. |
| `input_depth` | int | `1` | Input QoS queue depth. |
| `output_depth` | int | `10` | Output QoS queue depth. |
| `log_every` | int | `512` | Per-topic progress log interval. Set to `0` to disable progress logs. |
