#!/usr/bin/env bash
set -euo pipefail

CHANNEL="${1:-can0}"
BITRATE="${2:-1000000}"

sudo ip link set "${CHANNEL}" down 2>/dev/null || true
sudo ip link set "${CHANNEL}" type can bitrate "${BITRATE}" berr-reporting on restart-ms 100
sudo ip link set "${CHANNEL}" txqueuelen 1000
sudo ip link set "${CHANNEL}" up
ip -details -statistics link show "${CHANNEL}"

