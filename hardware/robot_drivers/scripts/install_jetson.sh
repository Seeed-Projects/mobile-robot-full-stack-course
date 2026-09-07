#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_USER="${SUDO_USER:-$(id -un)}"
RUN_GROUP="$(id -gn "${RUN_USER}")"
UNIT_DIR="/etc/systemd/system"

sudo apt-get update
sudo apt-get install -y can-utils gpiod python3-venv

python3 -m venv "${PROJECT_DIR}/.venv"
"${PROJECT_DIR}/.venv/bin/python" -m pip install --upgrade pip
"${PROJECT_DIR}/.venv/bin/python" -m pip install -e "${PROJECT_DIR}"

sudo install -m 0644 "${PROJECT_DIR}/deploy/systemd/dm-h65-can-termination.service" "${UNIT_DIR}/dm-h65-can-termination.service"
sudo install -m 0644 "${PROJECT_DIR}/deploy/systemd/dm-h65-can.service" "${UNIT_DIR}/dm-h65-can.service"
sudo install -m 0644 "${PROJECT_DIR}/deploy/systemd/dm-h65-webui.service.in" "${UNIT_DIR}/dm-h65-webui.service"
sudo sed -i \
    -e "s|@USER@|${RUN_USER}|g" \
    -e "s|@GROUP@|${RUN_GROUP}|g" \
    -e "s|@PROJECT_DIR@|${PROJECT_DIR}|g" \
    "${UNIT_DIR}/dm-h65-webui.service"

sudo systemctl daemon-reload
sudo systemctl enable --now dm-h65-can-termination.service dm-h65-can.service dm-h65-webui.service

echo "Installed DM-H65 SDK in ${PROJECT_DIR}"
echo "WebUI: http://<jetson-ip>:8765/"
echo "Status: systemctl status dm-h65-webui.service"

