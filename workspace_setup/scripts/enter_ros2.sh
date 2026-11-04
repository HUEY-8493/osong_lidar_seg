#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-${HOME}/docker/ros2_jazzy}"
CONTAINER_NAME="${CONTAINER_NAME:-ros2_jazzy_lidar}"

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

docker exec -it "$CONTAINER_NAME" bash
