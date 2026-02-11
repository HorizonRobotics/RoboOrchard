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

