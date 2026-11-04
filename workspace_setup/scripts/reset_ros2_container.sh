#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-${HOME}/docker/ros2_jazzy}"
CONTAINER_NAME="${CONTAINER_NAME:-ros2_jazzy_lidar}"
RESTART=0

usage() {
  echo "Usage: $0 [--restart]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --restart)
      RESTART=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ ! -f "${PROJECT_DIR}/docker-compose.yml" ]]; then
  echo "[ERROR] docker-compose.yml not found in ${PROJECT_DIR}."
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "[ERROR] Docker daemon is not available."
  exit 1
fi

cd "$PROJECT_DIR"
echo "[1/2] Removing container only. Image and host workspace are preserved."
docker compose down
docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

if [[ "$RESTART" -eq 1 ]]; then
  echo "[2/2] Restarting container..."
  docker compose up -d
else
  echo "[2/2] Restart skipped. Use --restart to bring it back immediately."
fi

echo "[DONE] Reset complete. Host workspace was not deleted."
