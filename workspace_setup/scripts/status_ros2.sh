#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-${HOME}/docker/ros2_jazzy}"
CONTAINER_NAME="${CONTAINER_NAME:-ros2_jazzy_lidar}"
IMAGE_NAME="${IMAGE_NAME:-guardianai/ros2-jazzy-lidar:latest}"

if ! docker info >/dev/null 2>&1; then
  echo "[ERROR] Docker daemon is not available."
  exit 1
fi

echo "[1/5] Docker container status..."
docker ps -a --filter "name=${CONTAINER_NAME}"

echo
echo "[2/5] Docker image status..."
docker images | grep 'guardianai/ros2-jazzy-lidar' || echo "[WARN] Image ${IMAGE_NAME} not found."

if ! docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo
  echo "[WARN] Container ${CONTAINER_NAME} does not exist yet."
  exit 0
fi

echo
echo "[3/5] Container environment..."
docker inspect "$CONTAINER_NAME" \
  --format '{{range .Config.Env}}{{println .}}{{end}}' \
  | grep -E '^(RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_DOMAIN_ID)=' || true

echo
echo "[4/5] Workspace mount..."
docker inspect "$CONTAINER_NAME" \
  --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}' \
  | grep '/root/ros2_ws' || true

if docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo
  echo "[5/5] ROS2 commands inside container..."
  docker exec "$CONTAINER_NAME" bash -lc 'source /root/.bashrc; which ros2; which rviz2 || true; echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"; echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"; echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"; ros2 --help >/dev/null && echo "ros2 --help: OK"; ros2 topic list || true'
else
  echo
  echo "[5/5] Container is not running. Start it with: ${PROJECT_DIR}/start_ros2.sh"
fi
