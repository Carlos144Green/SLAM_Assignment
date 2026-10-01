#!/bin/bash
set -e

echo "========================================"
echo "Starting Kalibr container"
echo "User: $(whoami)"
echo "Home directory: $HOME"
echo "Working directory: $(pwd)"
echo "========================================"

# Source ROS environment
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    echo "Sourcing ROS environment..."
    source /opt/ros/noetic/setup.bash
else
    echo "ROS environment not found at /opt/ros/noetic/setup.bash"
    exit 1
fi

if [ "${1:-}" = "sleep" ] && [ "${2:-}" = "infinity" ] && [ "$#" -eq 2 ]; then
    echo "Skipping workspace build for keepalive process."
    echo "Run /entrypoint.sh bash to build/source the workspace in an interactive shell."
    exec "$@"
fi

python3 /usr/local/bin/download_slam_dataset.py ros1 --data-dir "$HOME/data"

# Detect workspace path
WORKSPACE="$HOME/catkin_ws"
SRC_DIR="$WORKSPACE/src"
BASHRC="$HOME/.bashrc"
WORKSPACE_SETUP="$WORKSPACE/devel/setup.bash"
BASHRC_SOURCE_LINE="source \$HOME/catkin_ws/devel/setup.bash"

ensure_bashrc_source() {
    grep -qxF "$BASHRC_SOURCE_LINE" "$BASHRC" 2>/dev/null || echo "$BASHRC_SOURCE_LINE" >> "$BASHRC"
}

# Check if workspace exists
if [ ! -d "$SRC_DIR" ]; then
    echo "$SRC_DIR not found. Creating new workspace..."
    mkdir -p "$SRC_DIR"
    cd "$WORKSPACE"
    catkin init
else
    echo "Found existing workspace at $WORKSPACE"
fi

if [ ! -d "$WORKSPACE/.catkin_tools" ]; then
    echo "Initializing catkin workspace..."
    cd "$WORKSPACE"
    catkin init
fi

# Build if devel space missing
if [ ! -f "$WORKSPACE_SETUP" ]; then
    echo "No devel/setup.bash found — performing Release build..."
    cd "$WORKSPACE"
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin build -j"$(nproc)"
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
