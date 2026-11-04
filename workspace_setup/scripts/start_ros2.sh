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
  echo "[ERROR] Docker daemon is not available. Start Docker or re-login after install_docker.sh."
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "[WARN] DISPLAY is empty. RViz/X11 GUI may not work, but the container will still start."
fi

cd "$PROJECT_DIR"
echo "[1/3] Starting ROS2 Jazzy container..."
docker compose up -d

echo "[2/3] Checking container status..."
docker ps --filter "name=${CONTAINER_NAME}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "[ERROR] Container ${CONTAINER_NAME} is not running."
  docker compose ps
  exit 1
fi

echo "[3/3] Entering container..."
docker exec -it "$CONTAINER_NAME" bash
