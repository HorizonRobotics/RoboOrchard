# RoboOrchard

RoboOrchard is the **real robot system integration suite** for Horizon Robotics Robot Lab, providing a comprehensive framework for robotic manipulation, teleoperation, inference, and data management,
specifically designed for:

- **Research & Development**: Accelerate robotics algorithm development with a standardized framework
- **Rapid Prototyping**: Quick deployment of perception and control algorithms on real robots
- **Data Collection**: Streamlined data gathering for training and evaluation

### Key Features

- **[Hand-Eye Calibration](./ros2_package/robo_orchard_handeye_calib_ros2/)**: Automated calibration between camera and robot coordinate systems
- **[Teleoperation](./ros2_package/robo_orchard_teleop_ros2/)**: Remote control interface for manual robot operation
- **[Data Recording](./ros2_package/robo_orchard_data_ros2)**: Comprehensive data collection and logging capabilities.
- **[Model Deploy](./ros2_package/robo_orchard_deploy_ros2/)**: Efficiently deploy the manipulation model in real robot
- **ROS2 Integration**: Full ROS2 Humble support for distributed robotic systems

## Project Structure

```
roboorchard/
├── python/                       # Python packages
│   ├── robo_orchard_core/        # Core infrastructure (config, data structures)
│   ├── robo_orchard_schemas/     # Data schemas and validation
│   ├── robo_orchard_lab/         # Lab-specific utilities
│   └── robo_orchard_inference_app/ # Companision app
├── ros2_package/                 # ROS2 packages
│   ├── robo_orchard_data_ros2/   # Data handling
│   ├── robo_orchard_teleop_ros2/ # Teleoperation
│   ├── robo_orchard_piper_ros2/  # Piper arm control
│   ├── robo_orchard_deploy_ros2/ # Deployment tools
│   └── robo_orchard_handeye_calib_ros2/ # Hand-eye calibration
├── projects/HoloBrain/           # HoloBrain integration project
```

## Installation

### System Requirements

- **Python**: 3.10+
- **ROS2**: Humble
- **Operating System**: Ubuntu 22.04 recommended
- **CANTools**: For robotic arm CAN communication

### Quick Start (Development)

#### Step 1: Clone the Repository

```bash
git clone https://github.com/HorizonRobotics/RoboOrchard
cd roboorchard
```

#### Step 2: Set Up Python Environment

```bash
# Create virtual environment
python3 -m venv venv/roboorchard-venv
source venv/roboorchard-venv/bin/activate

# Install development dependencies
make dev-env
```

#### Step 3: Install Python Packages

```bash
# Install all packages in editable mode
make install-editable
```

Or install individual packages:

```bash
pip install -e python/robo_orchard_core
pip install -e python/robo_orchard_schemas
pip install -e python/robo_orchard_lab
pip install -e python/robo_orchard_inference_app
```

#### Step 4: Install ROS2 Packages

```bash
# Install ROS2 development dependencies
make ros2-dev-env

# Build all ROS2 packages
make ros2-build
```

Source the ROS2 workspace:

```bash
source install/setup.bash
```

## License

**RoboOrchard** is open-source and licensed under the [Apache License 2.0](https://github.com/HorizonRobotics/RoboOrchard/blob/master/LICENSE). If you are interested in contributing, please reach out to us.

## Contact

For questions and support, please contact the RoboOrchard Team at Horizon Robotics Robot Lab.
