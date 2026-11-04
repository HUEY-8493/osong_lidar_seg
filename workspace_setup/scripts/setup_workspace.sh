#!/usr/bin/env bash
set -euo pipefail

HOST_WS="${HOST_WS:-${HOME}/ros2_jazzy_ws}"
HOST_SRC="${HOST_WS}/src"
COMPOSE_DIR="${COMPOSE_DIR:-${HOME}/docker/ros2_jazzy}"
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKER_SRC_DIR="${PACKAGE_DIR}/docker"
SCRIPT_SRC_DIR="${PACKAGE_DIR}/scripts"
FORCE=0
INSTALL_ALIASES=0
ASK_ALIASES=1
ROS_DOMAIN_ID_VALUE=""

usage() {
  cat <<EOF
Usage: $0 [--force] [--aliases] [--no-aliases] [--domain-id ID]

Options:
  --force        Overwrite existing files in ${COMPOSE_DIR}.
  --aliases      Add convenience aliases to ~/.bashrc without prompting.
  --no-aliases   Do not prompt for aliases.
  --domain-id ID Add export ROS_DOMAIN_ID=ID to ${HOST_WS}/.ros_domain_id.bash.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --force)
      FORCE=1
      shift
      ;;
    --aliases)
      INSTALL_ALIASES=1
      ASK_ALIASES=0
      shift
      ;;
    --no-aliases)
      INSTALL_ALIASES=0
      ASK_ALIASES=0
      shift
      ;;
    --domain-id)
      ROS_DOMAIN_ID_VALUE="${2:-}"
      if [[ -z "$ROS_DOMAIN_ID_VALUE" ]]; then
        echo "[ERROR] --domain-id requires a value."
        exit 1
      fi
      shift 2
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

copy_file() {
  local src="$1"
  local dst="$2"
  local mode="${3:-0644}"

  if [[ -e "$dst" && "$FORCE" -ne 1 ]]; then
    local backup="${dst}.bak.$(date +%Y%m%d%H%M%S)"
    echo "[INFO] Existing file or directory found: ${dst}"
    echo "[INFO] Backing up to: ${backup}"
    cp -r "$dst" "$backup"
  fi

  # If the destination is a directory (often created by docker-compose volume mounts by mistake), remove it
  if [[ -d "$dst" ]]; then
    rm -rf "$dst"
  fi

  install -m "$mode" "$src" "$dst"
  echo "[OK] Copied $(basename "$src") -> ${dst}"
}

echo "[1/5] Creating host ROS2 workspace and compose directory..."
mkdir -p "$HOST_SRC"
mkdir -p "$COMPOSE_DIR"
touch "${HOME}/.Xauthority"

echo "[2/5] Copying Docker files..."
copy_file "${DOCKER_SRC_DIR}/Dockerfile" "${COMPOSE_DIR}/Dockerfile" 0644
copy_file "${DOCKER_SRC_DIR}/docker-compose.yml" "${COMPOSE_DIR}/docker-compose.yml" 0644
copy_file "${DOCKER_SRC_DIR}/cyclonedds.xml" "${COMPOSE_DIR}/cyclonedds.xml" 0644
copy_file "${DOCKER_SRC_DIR}/cyclonedds.xml" "${HOST_WS}/cyclonedds.xml" 0644

echo "[3/5] Copying helper scripts..."
for script in build_image.sh start_ros2.sh stop_ros2.sh enter_ros2.sh status_ros2.sh reset_ros2_container.sh set_domain_id.sh run_lidar.sh; do
  copy_file "${SCRIPT_SRC_DIR}/${script}" "${COMPOSE_DIR}/${script}" 0755
done

if [[ -n "$ROS_DOMAIN_ID_VALUE" ]]; then
  echo "[4/5] Writing optional ROS_DOMAIN_ID helper file..."
  printf 'export ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID_VALUE" > "${HOST_WS}/.ros_domain_id.bash"
  echo "[OK] Wrote ${HOST_WS}/.ros_domain_id.bash"
else
  echo "[4/5] ROS_DOMAIN_ID not configured. Use set_domain_id.sh after the container starts if needed."
fi

if [[ "$ASK_ALIASES" -eq 1 ]]; then
  read -r -p "Add convenience aliases to ~/.bashrc? [y/N] " answer
  if [[ "$answer" =~ ^[Yy]$ ]]; then
    INSTALL_ALIASES=1
  fi
fi

if [[ "$INSTALL_ALIASES" -eq 1 ]]; then
  echo "[5/5] Registering convenience aliases in ~/.bashrc..."
  ALIAS_BLOCK_START="# >>> ros2 jazzy docker aliases >>>"
  ALIAS_BLOCK_END="# <<< ros2 jazzy docker aliases <<<"
  if grep -Fq "$ALIAS_BLOCK_START" "${HOME}/.bashrc"; then
    echo "[INFO] Alias block already exists in ~/.bashrc."
  else
    cat >> "${HOME}/.bashrc" <<EOF

${ALIAS_BLOCK_START}
alias rosjazzy='${COMPOSE_DIR}/start_ros2.sh'
alias rosjazzy-enter='${COMPOSE_DIR}/enter_ros2.sh'
alias rosjazzy-stop='${COMPOSE_DIR}/stop_ros2.sh'
alias rosjazzy-status='${COMPOSE_DIR}/status_ros2.sh'
alias rosjazzy-reset='${COMPOSE_DIR}/reset_ros2_container.sh'
alias rosjazzy-domain='${COMPOSE_DIR}/set_domain_id.sh'
alias rosjazzy-build='${COMPOSE_DIR}/build_image.sh'
alias rosjazzy-lidar='${COMPOSE_DIR}/run_lidar.sh'
${ALIAS_BLOCK_END}
EOF
    echo "[OK] Aliases added. Open a new shell or run: source ~/.bashrc"
  fi
else
  echo "[5/5] Alias registration skipped."
fi

echo "[DONE] Workspace setup complete."
echo "Compose directory: ${COMPOSE_DIR}"
echo "Host workspace:    ${HOST_WS}"
echo "Next step: cd ${COMPOSE_DIR} && ./build_image.sh && ./start_ros2.sh"
