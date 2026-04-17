#!/bin/bash

set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# env for jazzy and omni ros2 bridge; must be set before launching the process (setting it inside Python has no effect)
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib

# install concert isaac if needed
/workspace/isaaclab/isaaclab.sh -p -m pip install -e /workspace/iit-concert-ros-pkg/concert_isaac/python >/dev/null 2>&1

# run the sim and forward all args
/workspace/isaaclab/isaaclab.sh -p "${SCRIPT_DIR}/run_sim.py" "$@"
