#!/usr/bin/env bash
set -euo pipefail

DOCKER_GPG_DIR="/etc/apt/keyrings"
DOCKER_GPG_FILE="${DOCKER_GPG_DIR}/docker.asc"
DOCKER_APT_FILE="/etc/apt/sources.list.d/docker.list"
CONFLICT_PACKAGES=(docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc)
DOCKER_PACKAGES=(docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin)

if [[ ! -r /etc/os-release ]]; then
  echo "[ERROR] /etc/os-release not found. This script supports Ubuntu only."
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" ]]; then
  echo "[ERROR] Unsupported OS: ${PRETTY_NAME:-unknown}. This script supports Ubuntu only."
  exit 1
fi

if ! command -v sudo >/dev/null 2>&1; then
  echo "[ERROR] sudo is required. Please install sudo or run from a sudo-capable account."
  exit 1
fi

echo "[1/6] Removing packages that can conflict with Docker CE..."
for package in "${CONFLICT_PACKAGES[@]}"; do
  sudo apt-get remove -y "$package" >/dev/null 2>&1 || true
done

echo "[2/6] Installing apt prerequisites..."
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg

echo "[3/6] Registering Docker official apt repository..."
sudo install -m 0755 -d "$DOCKER_GPG_DIR"
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o "$DOCKER_GPG_FILE"
sudo chmod a+r "$DOCKER_GPG_FILE"

ARCH="$(dpkg --print-architecture)"
CODENAME="${VERSION_CODENAME:-}"
if [[ -z "$CODENAME" ]]; then
  echo "[ERROR] Ubuntu VERSION_CODENAME was not found in /etc/os-release."
  exit 1
fi

echo "deb [arch=${ARCH} signed-by=${DOCKER_GPG_FILE}] https://download.docker.com/linux/ubuntu ${CODENAME} stable" \
  | sudo tee "$DOCKER_APT_FILE" >/dev/null

echo "[4/6] Installing Docker CE packages..."
sudo apt-get update
sudo apt-get install -y "${DOCKER_PACKAGES[@]}"

echo "[5/6] Enabling and starting Docker service..."
sudo systemctl enable docker
sudo systemctl start docker

echo "[6/6] Adding current user to docker group..."
sudo groupadd -f docker
sudo usermod -aG docker "$USER"

echo "[DONE] Docker installation finished."
echo "Next step: log out/in, reboot, or run 'newgrp docker' before using docker without sudo."
