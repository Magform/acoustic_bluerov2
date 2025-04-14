FROM osrf/ros:humble-desktop-full

ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=1

# Set up ROS 2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /root/.bashrc

# Install required dependencies
RUN apt-get update && apt-get install -y \
    build-essential autoconf automake libxmu-dev libx11-dev libxmu-dev libxmu-headers libxt-dev libtool gfortran bison flex \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-simple-launch \
    ros-${ROS_DISTRO}-slider-publisher \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-rmw-desert \
    neovim \
    python3-colcon-common-extensions \
    git && \
    rm -rf /var/lib/apt/lists/*

# Create base workspace directory
RUN mkdir -p /root/ros2_ws

# Build pose_to_tf
RUN mkdir -p /root/ros2_ws/pose_to_tf/src && \
    git clone https://github.com/oKermorgant/pose_to_tf.git /root/ros2_ws/pose_to_tf/src
WORKDIR /root/ros2_ws/pose_to_tf
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
RUN echo "source /root/ros2_ws/pose_to_tf/install/setup.sh" >> /root/.bashrc

# Build thruster_manager
RUN mkdir -p /root/ros2_ws/thruster_manager/src && \
    git clone https://github.com/CentraleNantesROV/thruster_manager /root/ros2_ws/thruster_manager/src
WORKDIR /root/ros2_ws/thruster_manager
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
RUN echo "source /root/ros2_ws/thruster_manager/install/setup.sh" >> /root/.bashrc

# Build auv_control
RUN mkdir -p /root/ros2_ws/auv_control/src && \
    git clone https://github.com/CentraleNantesROV/auv_control /root/ros2_ws/auv_control/src
WORKDIR /root/ros2_ws/auv_control
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && . /root/ros2_ws/thruster_manager/install/setup.sh && colcon build
RUN echo "source /root/ros2_ws/auv_control/install/setup.sh" >> /root/.bashrc

# Build bluerov2
RUN mkdir -p /root/ros2_ws/bluerov2/src && \
    git clone https://github.com/CentraleNantesROV/bluerov2.git /root/ros2_ws/bluerov2/src/
WORKDIR /root/ros2_ws/bluerov2
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /root/ros2_ws/pose_to_tf/install/setup.sh && \
    . /root/ros2_ws/thruster_manager/install/setup.sh && \
    . /root/ros2_ws/auv_control/install/setup.sh && \
    colcon build
RUN echo "source /root/ros2_ws/bluerov2/install/setup.sh" >> /root/.bashrc

# Build desert
run git clone https://github.com/signetlabdei/DESERT_Underwater && \
    mkdir -p /root/ros2_ws/desert
WORKDIR DESERT_Underwater/DESERT_Framework
RUN yes | ./install.sh --target LOCAL --inst_mode development --dest_folder /root/ros2_ws/desert --addons ALL
RUN echo "source /root/ros2_ws/desert/environment" >> /root/.bashrc

# Download rmw_desert files
RUN mkdir -p /root/ros2_ws/rmw_desert && \
    git clone https://github.com/signetlabdei/rmw_desert /root/ros2_ws/rmw_desert

# Download rmw_desert files
RUN mkdir -p /root/ros2_ws/rmw_desert && \
    git clone https://github.com/signetlabdei/rmw_desert /root/ros2_ws/rmw_desert

# Download acoustic_bluerov2 files
RUN mkdir -p /root/ros2_ws/acoustic_bluerov2 && \
    git clone https://github.com/Magform/acoustic_bluerov2 /root/ros2_ws/acoustic_bluerov2
RUN cp /root/ros2_ws/acoustic_bluerov2/bridge/ros_allowed_topics.conf /root/ros2_ws/rmw_desert/ros_allowed_topics.conf

# Set working directory for the container
WORKDIR /root

