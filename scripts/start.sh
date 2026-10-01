#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 ros1|ros2" >&2
}

TARGET="${1:-}"
if [[ "${TARGET}" != "ros1" && "${TARGET}" != "ros2" ]]; then
  usage
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DOCKER_DIR="${PROJECT_ROOT}/docker"
COMPOSE_FILE_PATH="${COMPOSE_FILE:-${DOCKER_DIR}/docker-compose.yaml}"
COMPOSE_GPU_FILE_PATH="${COMPOSE_GPU_FILE:-${DOCKER_DIR}/docker-compose.gpu.yaml}"

ORG="${ORG:-omnivio}"
TAG="${TAG:-latest}"

HOST_USERNAME="$(id -un)"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
XDG_RUNTIME_DIR_HOST="${XDG_RUNTIME_DIR:-/run/user/${HOST_UID}}"

export USERNAME="${HOST_USERNAME}"
export HOST_UID
export HOST_GID
export PROJECT_ROOT
export XDG_RUNTIME_DIR_HOST
export DATA_DIR="${DATA_DIR:-${PROJECT_ROOT}/dataset/${TARGET}}"
export OUTPUT_DIR="${OUTPUT_DIR:-${PROJECT_ROOT}/output}"

ROS1_ROOT_IMAGE="${ROS1_ROOT_IMAGE:-ubuntu:20.04}"
ROS2_ROOT_IMAGE="${ROS2_ROOT_IMAGE:-ubuntu:24.04}"

ROS1_BASE_IMAGE="${ORG}/ros1-base:${TAG}"
ROS1_ROS_IMAGE="${ORG}/ros1-ros:${TAG}"
ROS1_PYTHON_IMAGE="${ORG}/ros1-python:${TAG}"

ROS2_BASE_IMAGE="${ORG}/ros2-base:${TAG}"
ROS2_ROS_IMAGE="${ORG}/ros2-ros:${TAG}"
ROS2_CYCLONE_IMAGE="${ORG}/ros2-cyclone:${TAG}"
ROS2_EXTENDED_IMAGE="${ORG}/ros2-extended:${TAG}"

declare -A DOCKER_BUILDS=(
  ["${ROS1_BASE_IMAGE}"]="${DOCKER_DIR}/ros1/Dockerfile.base"
  ["${ROS1_ROS_IMAGE}"]="${DOCKER_DIR}/ros1/Dockerfile.ros"
  ["${ROS1_PYTHON_IMAGE}"]="${DOCKER_DIR}/ros1/Dockerfile.python"
  ["${ROS2_BASE_IMAGE}"]="${DOCKER_DIR}/ros2/Dockerfile.base"
  ["${ROS2_ROS_IMAGE}"]="${DOCKER_DIR}/ros2/Dockerfile.ros"
  ["${ROS2_CYCLONE_IMAGE}"]="${DOCKER_DIR}/ros2/Dockerfile.cyclone"
  ["${ROS2_EXTENDED_IMAGE}"]="${DOCKER_DIR}/ros2/Dockerfile.extended"
)

declare -A PARENTS=(
  ["${ROS1_BASE_IMAGE}"]="${ROS1_ROOT_IMAGE}"
  ["${ROS1_ROS_IMAGE}"]="${ROS1_BASE_IMAGE}"
  ["${ROS1_PYTHON_IMAGE}"]="${ROS1_ROS_IMAGE}"
  ["${ROS2_BASE_IMAGE}"]="${ROS2_ROOT_IMAGE}"
  ["${ROS2_ROS_IMAGE}"]="${ROS2_BASE_IMAGE}"
  ["${ROS2_CYCLONE_IMAGE}"]="${ROS2_ROS_IMAGE}"
  ["${ROS2_EXTENDED_IMAGE}"]="${ROS2_CYCLONE_IMAGE}"
)

BUILD_SEQUENCE=(
  "${ROS1_BASE_IMAGE}"
  "${ROS1_ROS_IMAGE}"
  "${ROS1_PYTHON_IMAGE}"
  "${ROS2_BASE_IMAGE}"
  "${ROS2_ROS_IMAGE}"
  "${ROS2_CYCLONE_IMAGE}"
  "${ROS2_EXTENDED_IMAGE}"
)

declare -A CONTAINER_NAMES=(
  ["ros1"]="kalibr-omni"
  ["ros2"]="vio-omni"
)

build_image() {
  local image_tag="$1"
  local dockerfile="${DOCKER_BUILDS[$image_tag]}"
  local base_from="${PARENTS[$image_tag]}"

  echo "Building ${image_tag} from ${base_from}"

  DOCKER_BUILDKIT=1 docker build \
    --build-arg BASE_FROM="${base_from}" \
    --build-arg USERNAME="${HOST_USERNAME}" \
    --build-arg USER_UID="${HOST_UID}" \
    --build-arg USER_GID="${HOST_GID}" \
    -t "${image_tag}" \
    -f "${dockerfile}" \
    "${DOCKER_DIR}"
}

enable_x11() {
  if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || true
    xhost +SI:localuser:"${HOST_USERNAME}" >/dev/null 2>&1 || true
  fi
}

nvidia_gpu_available() {
  command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1
}

container_name="${CONTAINER_NAMES[$TARGET]}"
if docker ps --format '{{.Names}}' | grep -qx "${container_name}"; then
  exec docker exec -it "${container_name}" /entrypoint.sh bash
fi

if [[ ! -f "${COMPOSE_FILE_PATH}" ]]; then
  echo "Compose file not found: ${COMPOSE_FILE_PATH}" >&2
  echo "Create docker/docker-compose.yaml first, then rerun: $0 ${TARGET}" >&2
  exit 1
fi

COMPOSE_ARGS=(-f "${COMPOSE_FILE_PATH}")
if nvidia_gpu_available; then
  if [[ ! -f "${COMPOSE_GPU_FILE_PATH}" ]]; then
    echo "GPU Compose override not found: ${COMPOSE_GPU_FILE_PATH}" >&2
    exit 1
  fi
  COMPOSE_ARGS+=(-f "${COMPOSE_GPU_FILE_PATH}")
  echo "Using NVIDIA GPU Compose override"
else
  echo "No NVIDIA GPU detected; running CPU-only Compose config"
fi

mkdir -p "${OUTPUT_DIR}"
mkdir -p "${DATA_DIR}"
mkdir -p "${XDG_RUNTIME_DIR_HOST}" 2>/dev/null || true
chmod 700 "${XDG_RUNTIME_DIR_HOST}" 2>/dev/null || true

for image in "${BUILD_SEQUENCE[@]}"; do
  build_image "${image}"
done

enable_x11

docker compose "${COMPOSE_ARGS[@]}" up -d "${TARGET}"
exec docker compose "${COMPOSE_ARGS[@]}" exec "${TARGET}" /entrypoint.sh bash
