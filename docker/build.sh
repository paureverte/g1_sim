#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
USER_ID=$(id -u)
GROUP_ID=$(id -g)

NO_CACHE=""
if [[ "${1:-}" == "--no-cache" ]]; then
    NO_CACHE="--no-cache"
fi

echo "Building g1_sim:jazzy image for user $USER_ID..."

DOCKER_BUILDKIT=1 docker build -t g1_sim:jazzy \
    -f "$SCRIPT_DIR/Dockerfile" \
    --build-arg USER_ID=$USER_ID \
    --build-arg GROUP_ID=$GROUP_ID \
    $NO_CACHE "$WORKSPACE_DIR"

echo "Built g1_sim:jazzy. You can now run the container with: docker compose -f docker/docker-compose.yml up -d"