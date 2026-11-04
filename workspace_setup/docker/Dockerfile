FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///root/ros2_ws/cyclonedds.xml

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-tools \
    ros-jazzy-rqt \
    ros-jazzy-rqt-graph \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git \
    vim \
    nano \
    net-tools \
    iproute2 \
    iputils-ping \
    tcpdump \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ros2_ws/src \
    && grep -qxF 'source /opt/ros/jazzy/setup.bash' /root/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && grep -qxF 'source /root/ros2_ws/install/setup.bash 2>/dev/null || true' /root/.bashrc || echo 'source /root/ros2_ws/install/setup.bash 2>/dev/null || true' >> /root/.bashrc \
    && grep -qxF 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' /root/.bashrc || echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc \
    && grep -qxF 'export CYCLONEDDS_URI=file:///root/ros2_ws/cyclonedds.xml' /root/.bashrc || echo 'export CYCLONEDDS_URI=file:///root/ros2_ws/cyclonedds.xml' >> /root/.bashrc \
    && grep -qxF 'source /root/ros2_ws/.ros_domain_id.bash 2>/dev/null || true' /root/.bashrc || echo 'source /root/ros2_ws/.ros_domain_id.bash 2>/dev/null || true' >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["tail", "-f", "/dev/null"]
