# Use an Ubuntu base image
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt update && apt install -y \
        locales \
        curl \
        gnupg \
        lsb-release \
        software-properties-common \
        build-essential \
        cmake \
        libusb-1.0-0-dev \
        pkg-config \
        libopencv-dev \
        wget \
        nano \
        unzip \
        git \
        python3 \
        python3-venv \
        python3-pip \
        usbutils \
        libssl-dev \
        libudev-dev \
        libgtk-3-dev \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        at \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*


# -------------------------
# Python Virtual Environment (Python 3.12)
# -------------------------
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/python -m pip install --upgrade pip setuptools wheel

ENV PATH="/opt/venv/bin:$PATH"


# -------------------------
# Python packages
# -------------------------
RUN pip install --no-cache-dir \
        opencv-python-headless \
        ultralytics \
        numpy \
        pyrealsense2 \
        matplotlib \
        lark


# -------------------------
# Install ROS 2 Jazzy
# -------------------------
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install -y \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# -------------------------
# TurtleBot3 Packages
# -------------------------
RUN apt update && apt install -y \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-bringup \
    ros-jazzy-dynamixel-sdk \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-diff-drive-controller

RUN pip install --no-cache-dir \
    dynamixel-sdk \
    pexpect

RUN echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc # If you are using LDS-01
RUN git clone https://github.com/Roboticbatmanduck/Rob4-Gr444-Projekt.git

# -------------------------
# Build system
# -------------------------
RUN pip install meson

# Auto source ROS
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# -------------------------
# Workspace setup
# -------------------------
ENV ROS_DISTRO=jazzy
ENV COLCON_WS=/workspace
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=30
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN mkdir -p ${COLCON_WS}/src
WORKDIR ${COLCON_WS}

CMD ["bash"]
