FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

ARG ROS_DISTRO

RUN cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
    cp /etc/apt/sources.list.d/ros2.sources /etc/apt/sources.list.d/ros2.sources.bak && \
    sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    sed -i 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.aliyun.com/ros2/ubuntu|g' /etc/apt/sources.list.d/ros2.sources && \
    echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries

RUN apt update

RUN apt install -y \
    vim \
    can-utils \
    ethtool \
    iproute2 \
    curl \
    xdg-utils \
    clang \
    libclang-dev \
    unzip \
    python3.10-venv \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-librealsense2* \
    ros-$ROS_DISTRO-realsense2* \
    ros-$ROS_DISTRO-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Build prerequisites for the external Pico and Hexfellow packages (amd64).
ARG RUST_VERSION=1.99.0
ENV CARGO_HOME=/opt/rust/cargo
ENV RUSTUP_HOME=/opt/rust/rustup
ENV PATH="${CARGO_HOME}/bin:${PATH}"

RUN test "$(dpkg --print-architecture)" = "amd64" && \
    curl -fsSL https://sh.rustup.rs -o /tmp/holobrain-rustup.sh && \
    sh /tmp/holobrain-rustup.sh -y --profile minimal \
        --default-toolchain "$RUST_VERSION" --no-modify-path && \
    rm /tmp/holobrain-rustup.sh && \
    rustc --version && cargo --version

ARG PROTOC_VERSION=27.1
RUN curl -fsSL \
        "https://github.com/protocolbuffers/protobuf/releases/download/v${PROTOC_VERSION}/protoc-${PROTOC_VERSION}-linux-x86_64.zip" \
        -o /tmp/holobrain-protoc.zip && \
    unzip /tmp/holobrain-protoc.zip -d /usr/local && \
    rm /tmp/holobrain-protoc.zip && \
    protoc --version

RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3

RUN pip install torch==2.8.0 \
    torchvision==0.23.0 \
    torchaudio==2.8.0 \
    --index-url https://download.pytorch.org/whl/cpu

RUN mv /etc/apt/sources.list.bak /etc/apt/sources.list && \
    mv /etc/apt/sources.list.d/ros2.sources.bak /etc/apt/sources.list.d/ros2.sources

ENV DEBIAN_FRONTEND=dialog
