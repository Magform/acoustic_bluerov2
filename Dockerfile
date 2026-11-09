FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=humble \
    ROS_DOMAIN_ID=1 \
    WORKSPACE=/root/ros2_ws

# Setup ROS 2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> /root/.bashrc

# Install required dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ros-${ROS_DISTRO}-rmw-desert \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-simple-launch \
    ros-${ROS_DISTRO}-slider-publisher \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-std-msgs \
    socat \
    libsdl2-dev \
    libyaml-cpp-dev \
    mesa-utils \
    evtest \
    python3-evdev \
    python3-colcon-common-extensions \
    git && \
    rm -rf /var/lib/apt/lists/*

# Create workspace and clone all repositories into src/
WORKDIR ${WORKSPACE}/src
RUN git clone https://github.com/oKermorgant/pose_to_tf.git && \
    git clone https://github.com/CentraleNantesROV/thruster_manager.git && \
    git clone https://github.com/CentraleNantesROV/auv_control.git && \
    git clone https://github.com/CentraleNantesROV/bluerov2.git

COPY src ${WORKSPACE}/src/acoustic_bluerov2

WORKDIR ${WORKSPACE}
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /root/.bashrc

WORKDIR ${WORKSPACE}/src/acoustic_bluerov2/bridge
