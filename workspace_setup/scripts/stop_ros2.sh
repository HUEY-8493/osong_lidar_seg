#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-${HOME}/docker/ros2_jazzy}"

if [[ ! -f "${PROJECT_DIR}/docker-compose.yml" ]]; then
  echo "[ERROR] docker-compose.yml not found in ${PROJECT_DIR}."
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "[ERROR] Docker daemon is not available."
  exit 1
fi

cd "$PROJECT_DIR"
docker compose down
echo "[DONE] ROS2 Jazzy container stopped."
