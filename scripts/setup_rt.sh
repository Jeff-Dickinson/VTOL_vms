#!/usr/bin/env bash
# Configure Raspberry Pi 4B for real-time control loop.
#
# Run once after fresh OS install. Requires reboot.
#
# What this does:
#   1. Isolate CPU core 3 from the Linux scheduler (for 400Hz RT thread)
#   2. Set CPU governor to "performance" (disable frequency scaling)
#   3. Enable pigpio daemon
#   4. Set memlock limits for real-time scheduling

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: run as root (sudo $0)"
    exit 1
fi

echo "==> Configuring kernel boot parameters"
CMDLINE="/boot/cmdline.txt"
if [ -f "/boot/firmware/cmdline.txt" ]; then
    CMDLINE="/boot/firmware/cmdline.txt"
fi

# Add isolcpus=3 if not already present
if ! grep -q "isolcpus=3" "${CMDLINE}"; then
    sed -i 's/$/ isolcpus=3/' "${CMDLINE}"
    echo "    Added isolcpus=3 to ${CMDLINE}"
else
    echo "    isolcpus=3 already set"
fi

echo "==> Setting CPU governor to performance"
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if [ -f "$cpu" ]; then
        echo "performance" > "$cpu"
    fi
done

# Make governor setting persist via rc.local or cron
GOVERNOR_SCRIPT="/etc/rc.local.d/cpu_governor.sh"
mkdir -p /etc/rc.local.d
cat > "${GOVERNOR_SCRIPT}" << 'SCRIPT'
#!/bin/sh
for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo "performance" > "$gov" 2>/dev/null
done
SCRIPT
chmod +x "${GOVERNOR_SCRIPT}"

# Add to rc.local if it exists
if [ -f /etc/rc.local ]; then
    if ! grep -q "cpu_governor" /etc/rc.local; then
        sed -i '/^exit 0/i /etc/rc.local.d/cpu_governor.sh' /etc/rc.local
    fi
fi

echo "==> Configuring real-time scheduling limits"
cat > /etc/security/limits.d/99-vms-rt.conf << 'LIMITS'
# Allow pi user to use real-time scheduling and lock memory
pi   -   rtprio  99
pi   -   memlock unlimited
LIMITS

echo "==> Enabling pigpio daemon"
systemctl enable pigpiod
systemctl start pigpiod || true

echo "==> Done. Reboot required for isolcpus to take effect."
echo "    sudo reboot"
