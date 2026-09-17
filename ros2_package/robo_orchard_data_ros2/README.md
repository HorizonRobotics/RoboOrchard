# robo_orchard_data_ros2

The RoboOrchard Data ROS2 package is a comprehensive and robust suite of tools designed for high-fidelity data acquisition in robotics, with a primary focus on the ROS 2 ecosystem.
It provides a powerful, configurable backend for recording data into the MCAP format, coupled with a user-friendly web-based application for interactive control and visualization.
This project is built to address the challenges of complex, real-world data collection scenarios, offering fine-grained control over data streams, ensuring data integrity, and simplifying the operator's workflow.

## Key Features

1. **Interactive Management with a Web GUI**: A user-friendly application built with Streamlit allows for easy starting, stopping, and monitoring of the recording process. It also features an integrated Foxglove panel for real-time data visualization.

2. **Flexible and Configurable ROS 2 Recording**: The system is built around a versatile Python ROS 2 node that provides a seamless bridge to the MCAP format, offering extensive configuration to handle complex recording scenarios.

3. **Advanced Topic Control**: Go beyond basic recording with powerful configuration options:

- Filter topics using regular expressions (include_patterns, exclude_patterns).

- Define per-topic Quality of Service (QoS) profiles for reliability and durability.

- Dynamically rename topics during recording.

4. **Real-time Data Integrity Monitoring**: Ensure the quality of your collected data with built-in monitoring tools:

- Frame Rate Monitoring: Set minimum and maximum frequency thresholds for any topic to detect sensor dropouts or data floods.

- Timestamp Anomaly Detection: Automatically detect and drop messages with significant timestamp jumps to prevent data corruption.

5. **Robust and Synchronized Recording**:

- Wait for Topics: Configure the recorder to wait for a specified set of topics to become active before starting, ensuring no data is missed at the beginning of a run.

- Static Topic Handling: Properly handles transient local topics (like /tf_static) to ensure they are captured correctly.

6. **Flexible Timestamping**: Choose whether to timestamp messages using the recorder's system clock or the original timestamp from the message header on a per-topic basis.

## Recorder lifecycle

`mcap_recorder_service` exposes `~/start_recording` (StartRecording),
`~/stop_recording` (Trigger), and `~/status`
(`robo_orchard_data_msg_ros2/msg/RecorderStatus`). The default status topic is
`/mcap_recorder_service/status`. Status is published at startup, on transitions,
and at 1 Hz, with a UUIDv4 `session_id`, destination, missing `waiting_topics`,
and `failure_reason`.

The recorder requires `rosbag2_py >= 0.15.14` (including the Humble backport
of `SequentialWriter.close()`) to report close failures explicitly. It rejects
bindings without this API at startup instead of pretending Stop succeeded.
Heartbeat and timeout checks use a steady clock, so paused or absent simulated
ROS time does not suppress them; status message stamps still use ROS time.

| State | Meaning |
| --- | --- |
| `idle` | No session started, or a waiting session was cancelled. |
| `waiting` | Writer initialized; waiting for required data and the first write. |
| `recording` | At least one message has been successfully written. |
| `finalizing` | Closing the writer after a Stop request. |
| `completed` | A session with data was stopped and closed successfully. |
| `failed` | Initialization, wait, write, registration, or cleanup failed. |

Start success acknowledges initialization, not that data is already being
written. `wait_for_topics` requires receipt of messages, not just discovery
of publishers. `wait_for_topics_timeout_s` defaults to 10 seconds and bounds
the initial wait until data can be written, even when no required topics are
configured. The timeout must be positive and finite. Timeout reports the
missing topics. The Recorder excludes its own status topic from discovery so
its heartbeat cannot turn an empty session into a successful recording.
Stop during waiting cancels without reporting completion. Close failures
return a failed Stop response. `completed` and `failed` persist until another
Start; they describe the latest session, not a new event on each heartbeat.
Completion does not certify data quality or robot task success.

A new accepted session gets a new UUID; its ID remains stable across statuses.
Existing destinations (including symbolic links) are rejected before creating
a writer, without touching an existing bag's metadata or marker. Destinations
must be exclusive to this recorder; cross-process reservation is not provided.
After a node restart status is `idle`, with an empty session ID. Clients should
show unknown/offline when heartbeats stop and must not treat startup idle as
proof that a previous session completed. Automatic crash recovery is not
provided.

The `__RECORDING__` file is created when a session enters waiting and removed
on cleanup. A process crash may leave it behind; absence of the marker alone
does not prove success. No additional result JSON is written. MCAP destination,
serialization and timestamp configuration are unchanged. Shutdown closes the
writer without requiring a live ROS publisher; it does not guarantee delivery
of a terminal snapshot. Writer operations, discovery and status callbacks are
serialized in one mutually exclusive callback group.
