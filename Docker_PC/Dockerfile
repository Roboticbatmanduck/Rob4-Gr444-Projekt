# --------------------------------------------------
# ROS2 Humble base
# --------------------------------------------------
FROM osrf/ros:humble-desktop-full

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
# ROS 2 Humble packages, bare for at være sikker :)
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-nav2-bringup \
    ros-humble-vision-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

# --------------------------------------------------
#  TurtleBot3 runtime med gazebo
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-bringup \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-gazebo \
 && rm -rf /var/lib/apt/lists/*

 
 RUN pip install \
    Ultralytics \
    opencv-python

#YOLO fix
#ROS2 Humble cv_bridge needs NumPy 1.x

RUN pip3 install --no-cache-dir --force-reinstall numpy==1.26.4


# ----------------------------
# LibRealSense2
# ----------------------------
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
