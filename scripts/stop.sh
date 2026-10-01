#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE_PATH="${COMPOSE_FILE:-${PROJECT_ROOT}/docker/docker-compose.yaml}"

HOST_USERNAME="$(id -un)"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
XDG_RUNTIME_DIR_HOST="${XDG_RUNTIME_DIR:-/run/user/${HOST_UID}}"

export USERNAME="${HOST_USERNAME}"
export HOST_UID
export HOST_GID
export PROJECT_ROOT
export XDG_RUNTIME_DIR_HOST
export OUTPUT_DIR="${OUTPUT_DIR:-${PROJECT_ROOT}/output}"

if [[ -f "${COMPOSE_FILE_PATH}" ]]; then
  docker compose -f "${COMPOSE_FILE_PATH}" down --remove-orphans
  exit 0
fi

for container in kalibr-omni vio-omni; do
  if docker ps -a --format '{{.Names}}' | grep -qx "${container}"; then
    docker stop "${container}" >/dev/null 2>&1 || true
    docker rm "${container}" >/dev/null 2>&1 || true
  fi
done
