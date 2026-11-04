#!/usr/bin/env bash
set -euo pipefail

SYSCTL_FILE="/etc/sysctl.d/99-cyclonedds.conf"

if ! command -v sudo >/dev/null 2>&1; then
  echo "[ERROR] sudo is required to write ${SYSCTL_FILE}."
  exit 1
fi

echo "[1/3] Writing CycloneDDS host socket buffer sysctl configuration..."
sudo tee "$SYSCTL_FILE" >/dev/null <<'EOF'
net.core.rmem_max=2147483647
net.core.rmem_default=10485760
net.core.wmem_max=2147483647
net.core.wmem_default=10485760
EOF

echo "[2/3] Applying sysctl settings on the host..."
sudo sysctl --system

echo "[3/3] Verifying active values..."
sysctl net.core.rmem_max
sysctl net.core.rmem_default
sysctl net.core.wmem_max
sysctl net.core.wmem_default

echo "[DONE] Host socket buffers are configured for CycloneDDS."
