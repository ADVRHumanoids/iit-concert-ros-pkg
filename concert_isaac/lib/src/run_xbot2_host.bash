#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
COMPOSE_DIR="${SCRIPT_DIR}/../../docker/iit-concert-ros-pkg-isaac"

cd "${COMPOSE_DIR}"

mkdir -p /tmp/.xbot2_isaac

docker compose run --rm concert-xbot2
