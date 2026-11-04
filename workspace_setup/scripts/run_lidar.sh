#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ros2_jazzy_lidar}"
DISPLAY_VALUE="${DISPLAY:-:0}"
# guardian_manager supplies these mode-specific values. Defaults preserve a
# direct/manual crane launch while keeping the bridge entirely Docker-local.
GUARDIAN_PERCEPTION_PARAMS_FILE="${GUARDIAN_PERCEPTION_PARAMS_FILE:-}"
GUARDIAN_SYSTEM_MODE="${GUARDIAN_SYSTEM_MODE:-${GUARDIAN_BRIDGE_MODE:-crane}}"
GUARDIAN_BRIDGE_MODE="${GUARDIAN_BRIDGE_MODE:-$GUARDIAN_SYSTEM_MODE}"
case "$GUARDIAN_SYSTEM_MODE" in crane|general) ;; *) echo "[ERROR] Mode must be crane or general." >&2; exit 2 ;; esac
if [ "$GUARDIAN_BRIDGE_MODE" != "$GUARDIAN_SYSTEM_MODE" ]; then
    echo "[ERROR] System and bridge modes disagree; bringup was not started." >&2
    exit 2
fi
GUARDIAN_BRIDGE_MANAGER_URL="${GUARDIAN_BRIDGE_MANAGER_URL:-http://127.0.0.1:8765/api/internal/bridge}"
GUARDIAN_BRIDGE_TOKEN="${GUARDIAN_BRIDGE_TOKEN:-}"

# 컨테이너 자체가 존재하는지 확인
if ! docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    echo "[ERROR] Container '${CONTAINER_NAME}' does not exist."
    exit 1
fi

# 컨테이너가 꺼져 있으면 자동 시작
if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    echo "[INFO] Starting container: ${CONTAINER_NAME}"
    docker start "$CONTAINER_NAME" >/dev/null
fi

echo "[INFO] Container is running."

# X11 권한 허용
xhost +SI:localuser:root >/dev/null 2>&1 || true

echo "[INFO] DISPLAY=${DISPLAY_VALUE}"
echo "[INFO] Running LSLiDAR bringup..."

docker exec -d \
    -e DISPLAY="$DISPLAY_VALUE" \
    -e GUARDIAN_SYSTEM_MODE="$GUARDIAN_SYSTEM_MODE" \
    -e GUARDIAN_PERCEPTION_PARAMS_FILE="$GUARDIAN_PERCEPTION_PARAMS_FILE" \
    -e GUARDIAN_BRIDGE_MODE="$GUARDIAN_BRIDGE_MODE" \
    -e GUARDIAN_BRIDGE_MANAGER_URL="$GUARDIAN_BRIDGE_MANAGER_URL" \
    -e GUARDIAN_BRIDGE_TOKEN="$GUARDIAN_BRIDGE_TOKEN" \
    "$CONTAINER_NAME" \
    bash -lc '
        source /opt/ros/jazzy/setup.bash
        source /root/ros2_ws/install/setup.bash

        LAUNCH_ARGS=(system_mode:="${GUARDIAN_SYSTEM_MODE}")
        if [ "$GUARDIAN_SYSTEM_MODE" = "crane" ]; then
            # Guardian Manager starts RViz; avoid starting another instance here.
            LAUNCH_ARGS+=(use_rviz:=false)
        fi
        [ -n "${GUARDIAN_PERCEPTION_PARAMS_FILE:-}" ] && LAUNCH_ARGS+=(perception_params_file:="${GUARDIAN_PERCEPTION_PARAMS_FILE}")
        [ -n "${GUARDIAN_BRIDGE_MODE:-}" ] && LAUNCH_ARGS+=(guardian_bridge_mode:="${GUARDIAN_BRIDGE_MODE}")
        [ -n "${GUARDIAN_BRIDGE_MANAGER_URL:-}" ] && LAUNCH_ARGS+=(guardian_bridge_manager_url:="${GUARDIAN_BRIDGE_MANAGER_URL}")
        [ -n "${GUARDIAN_BRIDGE_TOKEN:-}" ] && LAUNCH_ARGS+=(guardian_bridge_token:="${GUARDIAN_BRIDGE_TOKEN}")

        exec ros2 launch lidar_bringup lidar_bringup.launch.py "${LAUNCH_ARGS[@]}"
    '

echo "[INFO] LSLiDAR bringup started."
