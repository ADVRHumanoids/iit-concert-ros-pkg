#!/bin/bash
# Setup script for concert-focal-ros1 docker container

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Source the main setup
if [ -f "$SCRIPT_DIR/../../setup.sh" ]; then
    source "$SCRIPT_DIR/../../setup.sh"
fi

# Container-specific environment
export CONCERT_DOCKER_ENV="concert-focal-ros1"
