# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base

# ── system deps ──────────────────────────────────────────────────────────────
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-rclpy \
    ros-jazzy-rclcpp \
    ros-jazzy-std-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-urdf \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-msgs \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-rviz2 \
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    ros-jazzy-camera-info-manager \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# ── workspace ────────────────────────────────────────────────────────────────
WORKDIR /ros2_ws
COPY . src/mini_pupper_ros/

RUN rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── entrypoint ───────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
