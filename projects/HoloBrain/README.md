# Introduction

This project provides the real robot system integration code for the [HoloBrain](https://horizonrobotics.github.io/robot_lab/holobrain/) project.

For model training and evaluation, please refer to the HoloBrain project in [RoboOrchardLab](https://github.com/HorizonRobotics/RoboOrchardLab/tree/master/projects/holobrain).

# Quick Start

## Docker pull

```bash
docker pull horizonrobotics/holobrain:ubuntu22.04-py3.10-ros-humble-torch2.8.0
```

## Launch scripts
> [Host] means launch in host, [Docker] means launch in docker.

### [Host] Identify CAN Ports for Robotic Arms

Run the following script to list connected devices:

```bash
bash teleop/find-all-can-port.sh
```

>Tip: Connect the robotic arms one by one to identify the specific port corresponding to each arm.

Since hardware configurations vary across machines, a unified CAN port configuration is not possible. A template script is provided for you to customize:

```bash
cp teleop/templates/rename-can.sh teleop/
```

Open the file and modify the **USB_PORTS** field according to your actual hardware setup.

### [Host] Configure Extrinsic Parameters

Due to varying environmental setups, extrinsic parameters will differ. Please use the provided template to configure them:

```bash
cp data/tf/templates/gen_tf_config.py data/tf
```

Open the file and update the parameters to match your specific calibration data.

### [Host] Start Docker

As directory structures vary between machines, a unified startup script is not provided. Please customize the template below:

```bash
cp launch/templates/docker.sh launch/
```

Open docker.sh, modify the volume mount paths and necessary environment variables, and then start the container:

```bash
bash launch/docker.sh
```

### [Docker] Install RoboOrchard

**⚠️ Note: It is recommended to repeat this step whenever the RoboOrchard codebase is updated.**

Once inside the Docker container, create a virtual environment and follow the RoboOrchard documentation to install RoboOrchard and the corresponding ROS2 packages.

```bash
mkdir venv
python3 -m venv venv/robot-venv
source venv/robot-venv/bin/activate
source /opt/ros/humble/setup.bash

export ROBO_ORCHARD_PATH=/path/to/roboorchard

cd $ROBO_ORCHARD_PATH

git config --global --add safe.directory $ROBO_ORCHARD_PATH
git config --global --add safe.directory $ROBO_ORCHARD_PATH/python/robo_orchard_core
git config --global --add safe.directory $ROBO_ORCHARD_PATH/python/robo_orchard_schemas
git config --global --add safe.directory $ROBO_ORCHARD_PATH/python/robo_orchard_lab

# prepare env
make dev-env
make ros2-dev-env

# install python packages
make install-editable

# install ros2 packages
make ros2-build
```

### [Host] One-Click Start

1. Install dependencies:

```bash
pip install -r launch/requirements.txt
```

2. Create and customize the configuration file:

```bash
cp launch/templates/launch.yaml launch/
```

3. Execute the launch script:

```bash
./launch/start.sh
```

### [Host] Stop

To stop the system, run:

```bash
./launch/stop.sh
```
