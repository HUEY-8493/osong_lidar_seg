#!/usr/bin/env bash
set -e

# eno1: 기존 네트워크 (게이트웨이/인터넷용)
sudo nmcli connection modify "netplan-eno1" \
  ipv4.method manual \
  ipv4.addresses "192.168.223.152/24" \
  ipv4.gateway "192.168.223.1" \
  ipv4.dns "8.8.8.8 1.1.1.1" \
  ipv4.never-default no \
  ipv6.method ignore

sudo nmcli connection up "netplan-eno1"

# enp2s0: 192.168.1.x 로컬 네트워크용
# 이미 프로필이 있으면 수정하고, 없으면 새로 생성
if nmcli connection show "enp2s0-static" >/dev/null 2>&1; then
  sudo nmcli connection modify "enp2s0-static" \
    ipv4.method manual \
    ipv4.addresses "192.168.1.102/24" \
    ipv4.never-default yes \
    ipv6.method ignore
else
  sudo nmcli connection add type ethernet ifname enp2s0 con-name "enp2s0-static" \
    ipv4.method manual \
    ipv4.addresses "192.168.1.102/24" \
    ipv4.never-default yes \
    ipv6.method ignore
fi

sudo nmcli connection up "enp2s0-static" || true

echo
echo "=== Network status ==="
nmcli device status
ip -br addr show eno1 enp2s0
ip route