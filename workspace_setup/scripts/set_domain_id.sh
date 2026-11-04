#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-${HOME}/docker/ros2_jazzy}"
CONTAINER_NAME="${CONTAINER_NAME:-ros2_jazzy_lidar}"

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 ROS_DOMAIN_ID"
  exit 1
fi

ROS_DOMAIN_ID_VALUE="$1"
if ! [[ "$ROS_DOMAIN_ID_VALUE" =~ ^[0-9]+$ ]]; then
  echo "[ERROR] ROS_DOMAIN_ID must be a non-negative integer."
  exit 1
fi

if [[ ! -f "${PROJECT_DIR}/docker-compose.yml" ]]; then
  echo "[ERROR] docker-compose.yml not found in ${PROJECT_DIR}."
  echo "Run setup_workspace.sh first."
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "[ERROR] Docker daemon is not available."
  exit 1
fi

cd "$PROJECT_DIR"
if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "[INFO] Container ${CONTAINER_NAME} is not running. Starting it now..."
  docker compose up -d
fi

docker exec "$CONTAINER_NAME" bash -lc "sed -i '/ROS_DOMAIN_ID/d' /root/.bashrc && echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}' >> /root/.bashrc"

echo "[DONE] ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE} was added to /root/.bashrc in ${CONTAINER_NAME}."
echo "Open a new container shell for it to take effect: ${PROJECT_DIR}/enter_ros2.sh"
docker exec "$CONTAINER_NAME" bash -lc "grep 'ROS_DOMAIN_ID' /root/.bashrc"
