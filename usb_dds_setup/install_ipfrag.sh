#!/bin/bash
set -e

CONF_FILE="ipfrag.conf"
TARGET_DIR="/etc/sysctl.d"
TARGET_FILE="$TARGET_DIR/99-throughput.conf"

echo "Installing custom sysctl settings..."

# Copy the sysctl config file
if [ -f "$CONF_FILE" ]; then
    sudo cp "$CONF_FILE" "$TARGET_FILE"
    echo "Copied $CONF_FILE to $TARGET_FILE"
else
    echo "Error: $CONF_FILE not found!"
    exit 1
fi

# Reload sysctl settings immediately
echo "Reloading sysctl..."
sudo sysctl --system

# Verify applied values
echo "Verifying values..."

# IP fragmentation
sysctl net.ipv4.ipfrag_time
sysctl net.ipv4.ipfrag_high_thresh
sysctl net.ipv4.ipfrag_low_thresh

# Socket buffers
sysctl net.core.rmem_default
sysctl net.core.wmem_default
sysctl net.core.rmem_max
sysctl net.core.wmem_max

# UDP
sysctl net.ipv4.udp_rmem_min
sysctl net.ipv4.udp_wmem_min
sysctl net.ipv4.udp_mem

# Backlog
sysctl net.core.netdev_max_backlog

# Resource limits
sysctl fs.file-max
sysctl vm.max_map_count

echo "Done. Settings will persist across reboots."

