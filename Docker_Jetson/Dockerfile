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

# --------------------------------------------------
# ROS 2 Humble repository (Ubuntu 22.04 / jammy)
# --------------------------------------------------
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" \
> /etc/apt/sources.list.d/ros2.list
# --------------------------------------------------
# ROS 2 Humble core (uden desktop/gazebo)
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
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

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "export LDS_MODEL=LDS-01" >> /root/.bashrc

WORKDIR ${COLCON_WS}

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
