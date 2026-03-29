# Mini Pupper ROS2 Jazzy Docker image
FROM ros:jazzy-ros-base-noble

LABEL maintainer="info@mangdang.net"
LABEL description="Mini Pupper ROS2 Jazzy development environment"

# Set non-interactive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    curl \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create workspace
WORKDIR /ros2_ws

# Copy source files
COPY . src/mini_pupper_ros/

# Install ROS dependencies
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Source workspace in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash"]
