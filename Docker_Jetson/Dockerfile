# --------------------------------------------------
# Ultralytics Jetson base (JetPack 6 / Ubuntu 22.04)
# --------------------------------------------------
FROM ultralytics/ultralytics:latest-jetson-jetpack6

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --------------------------------------------------
# Base utilities (minimalt)
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    software-properties-common\
    locales \
    curl \
    gnupg \
    lsb-release \
    git \
    nano \
    wget \
    usbutils \
    python3-pexpect \
 && locale-gen en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe && apt-get update
# --------------------------------------------------
# ROS 2 Humble repository (Ubuntu 22.04 / jammy)
# --------------------------------------------------
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') &&\
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" &&\
    dpkg -i /tmp/ros2-apt-source.deb


# --------------------------------------------------
# ROS 2 Humble core (uden desktop/gazebo)
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-diagnostic-updater \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

# --------------------------------------------------
# (Valgfrit) TurtleBot3 runtime ONLY
#   ⚠️ ingen simulation / gazebo på Jetson
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-bringup \
    ros-humble-dynamixel-sdk \
 && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# ROS environment
# --------------------------------------------------
ENV ROS_DISTRO=humble
ENV COLCON_WS=/workspace
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=30
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# --------------------------------------------------
# RealSense SDK
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    apt-transport-https \
    lsb-release

RUN mkdir -p /etc/apt/keyrings &&\
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
gpg --dearmor | tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null


RUN echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
 tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update

RUN apt-get update && apt-get install -y \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "export LDS_MODEL=LDS-01" >> /root/.bashrc

WORKDIR ${COLCON_WS}

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
