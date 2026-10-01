#!/bin/bash
set -e

echo "========================================"
echo "Starting ROS 2 container"
echo "User: $(whoami)"
echo "Home directory: $HOME"
echo "Working directory: $(pwd)"
echo "========================================"

if [ -n "${XDG_RUNTIME_DIR:-}" ]; then
    mkdir -p "$XDG_RUNTIME_DIR"
    chmod 700 "$XDG_RUNTIME_DIR"
fi

# Source ROS 2 environment
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${ROS_SETUP}" ]; then
    echo "Sourcing ROS 2 environment (${ROS_DISTRO})..."
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
else
    echo "ROS 2 environment not found at ${ROS_SETUP}"
    exit 1
fi

if [ "${1:-}" = "sleep" ] && [ "${2:-}" = "infinity" ] && [ "$#" -eq 2 ]; then
    echo "Skipping workspace build for keepalive process."
    echo "Run /entrypoint.sh bash to build/source the workspace in an interactive shell."
    exec "$@"
fi

python3 /usr/local/bin/download_slam_dataset.py ros2 --data-dir "$HOME/data"

# Detect workspace path
WORKSPACE="$HOME/ros2_ws"
SRC_DIR="$WORKSPACE/src"
BASHRC="$HOME/.bashrc"
WORKSPACE_SETUP="$WORKSPACE/install/setup.bash"
BASHRC_SOURCE_LINE="source \$HOME/ros2_ws/install/setup.bash"

ensure_bashrc_source() {
    grep -qxF "$BASHRC_SOURCE_LINE" "$BASHRC" 2>/dev/null || echo "$BASHRC_SOURCE_LINE" >> "$BASHRC"
}

# Check if workspace exists
if [ ! -d "$SRC_DIR" ]; then
    echo "$SRC_DIR not found. Creating new workspace..."
    mkdir -p "$SRC_DIR"
else
    echo "Found existing workspace at $WORKSPACE"
fi

# Build if install space missing
if [ ! -f "$WORKSPACE_SETUP" ]; then
    echo "No install/setup.bash found — building with colcon..."
    cd "$WORKSPACE"
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers "$(nproc)"
else
    echo "Workspace already built, sourcing environment..."
fi

if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
    ensure_bashrc_source
else
    echo "Workspace setup file was not created: $WORKSPACE_SETUP"
    exit 1
fi

exec "$@"
