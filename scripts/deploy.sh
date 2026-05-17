#!/usr/bin/env bash
# Deploy VMS to Raspberry Pi, build C libraries, install Python package.
#
# Usage: ./scripts/deploy.sh [PI_HOST]
#   PI_HOST defaults to "pi@vtol.local"
#
# Prerequisites on Pi:
#   sudo apt install cmake build-essential pigpio python3-pip
#   Run scripts/setup_rt.sh once for real-time kernel config

set -euo pipefail

PI_HOST="${1:-pi@vtol.local}"
REMOTE_DIR="/home/pi/vms"

echo "==> Syncing project to ${PI_HOST}:${REMOTE_DIR}"
rsync -avz --delete \
    --exclude='build/' \
    --exclude='logs/' \
    --exclude='.pytest_cache/' \
    --exclude='__pycache__/' \
    --exclude='*.egg-info/' \
    ./ "${PI_HOST}:${REMOTE_DIR}/"

echo "==> Building C libraries on Pi"
ssh "${PI_HOST}" "cd ${REMOTE_DIR} && mkdir -p build && cd build && cmake .. && make -j4"

echo "==> Installing Python package on Pi"
ssh "${PI_HOST}" "cd ${REMOTE_DIR} && pip3 install -e ."

echo "==> Done. Run on Pi with:"
echo "    ssh ${PI_HOST} 'cd ${REMOTE_DIR} && sudo vms-start --config config'"
