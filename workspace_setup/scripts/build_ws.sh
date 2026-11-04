#!/bin/bash
set -e

echo "=== Updating system and installing PCL dependencies ==="
sudo apt-get update
sudo apt-get install -y libpcl-dev libpcap-dev ros-jazzy-pcl-conversions ros-jazzy-pcl-ros

echo "=== Installing ROS dependencies using rosdep ==="
# ROS 2 환경 소싱 (Ubuntu 24.04의 경우 보통 Jazzy 사용)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# src 폴더 내의 패키지들의 의존성 자동 설치
rosdep update || echo "rosdep update failed or already updated. Continuing..."
rosdep install --from-paths src --ignore-src -r -y

echo "=== Building the workspace ==="
colcon build --symlink-install

echo "=== Build Complete! ==="
