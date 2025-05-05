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
    build-essential autoconf automake libxmu-dev libx11-dev libxmu-headers libxt-dev libtool gfortran bison flex \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-simple-launch \
    ros-${ROS_DISTRO}-slider-publisher \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-rmw-desert \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-std-msgs \
    neovim \
    evtest \
    python3-colcon-common-extensions \
    python3-pip \
    git && \
    rm -rf /var/lib/apt/lists/*

# Create workspace and clone all repositories into src/
WORKDIR ${WORKSPACE}
RUN mkdir src

WORKDIR ${WORKSPACE}/src
RUN git clone https://github.com/oKermorgant/pose_to_tf.git
RUN git clone https://github.com/CentraleNantesROV/thruster_manager.git
RUN git clone https://github.com/CentraleNantesROV/auv_control.git
RUN git clone https://github.com/CentraleNantesROV/bluerov2.git
RUN git clone https://github.com/Magform/acoustic_bluerov2.git

# Build the workspace
WORKDIR ${WORKSPACE}
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /root/.bashrc

# Build DESERT separately (non-ROS)
WORKDIR /root
RUN git clone https://github.com/signetlabdei/DESERT_Underwater.git && \
    mkdir -p ${WORKSPACE}/desert

WORKDIR DESERT_Underwater/DESERT_Framework
RUN yes | ./install.sh --target LOCAL --inst_mode development --dest_folder ${WORKSPACE}/desert --addons ALL && \
    echo "source ${WORKSPACE}/desert/environment" >> /root/.bashrc

# Set working directory and keep root user
WORKDIR /root
USER root
