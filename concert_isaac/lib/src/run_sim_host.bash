#!/bin/bash

set -euo pipefail

# shortcuts
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
COMPOSE_DIR="${SCRIPT_DIR}/../../docker/iit-concert-ros-pkg-isaac"

cd "${COMPOSE_DIR}"

mkdir -p /tmp/.xbot2_isaac

docker compose up -d isaac-sim --no-recreate

ARGS="$*"
docker compose exec isaac-sim bash -i /workspace/iit-concert-ros-pkg/concert_isaac/lib/src/run_sim.bash ${ARGS}
