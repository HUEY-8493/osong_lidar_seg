#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-${HOME}/docker/ros2_jazzy}"
IMAGE_NAME="${IMAGE_NAME:-guardianai/ros2-jazzy-lidar:latest}"
NO_CACHE_ARGS=()

usage() {
  echo "Usage: $0 [--no-cache]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-cache)
      NO_CACHE_ARGS=(--no-cache)
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
  echo "Run setup_workspace.sh first."
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "[ERROR] Docker daemon is not available. Start Docker or check docker group permissions."
  exit 1
fi

cd "$PROJECT_DIR"
echo "[1/2] Building image ${IMAGE_NAME}..."
docker compose build "${NO_CACHE_ARGS[@]}"

echo "[2/2] Verifying image..."
docker images | grep 'guardianai/ros2-jazzy-lidar' || {
  echo "[ERROR] Image ${IMAGE_NAME} was not found after build."
  exit 1
}

echo "[DONE] Image build complete."
