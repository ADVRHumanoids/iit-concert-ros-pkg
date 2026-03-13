#!/usr/bin/env bash

set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /opt/xbot/setup.sh

if [ -f /home/user/xbot2_ws/setup.bash ]; then
    source /home/user/xbot2_ws/setup.bash
fi

if [ -f /home/user/env/bin/activate ]; then
    source /home/user/env/bin/activate
fi
set -u

export PYTHONPATH="/home/user/xbot2_ws/src/modular/src${PYTHONPATH:+:${PYTHONPATH}}"

required_libs=(
    /opt/xbot/lib/libxbotdev_zmq_hal.so
    /opt/xbot/lib/libxbotctrl_zmq_io.so
    /opt/xbot/lib/libzmq_io_protoc.so
)

for lib in "${required_libs[@]}"; do
    if [ ! -f "${lib}" ]; then
        echo "[concert-xbot2] missing expected library: ${lib}" >&2
        exit 1
    fi
done

required_assets=(
    /workspace/iit-concert-ros-pkg/concert_usd/usd/concert_complete/concert_complete.usd
    /workspace/iit-concert-ros-pkg/concert_urdf/urdf/concert_complete.urdf
    /workspace/iit-concert-ros-pkg/concert_srdf/srdf/ModularBot.srdf
)

for asset in "${required_assets[@]}"; do
    if [ ! -f "${asset}" ]; then
        echo "[concert-xbot2] missing committed deployment asset: ${asset}" >&2
        exit 1
    fi
done

if ! ros2 pkg prefix --share dagana_urdf >/dev/null 2>&1; then
    echo "[concert-xbot2] dagana_urdf package is missing from the workspace" >&2
    exit 1
fi

generator=/home/user/xbot2_ws/src/iit-concert-ros-pkg/concert_examples/src/concert_example.py
python3 "${generator}" -o urdf -a gazebo_urdf:=false floating_base:=true -r modularbot >/dev/null
python3 "${generator}" -o srdf -a gazebo_urdf:=false -r modularbot >/dev/null

echo "[concert-xbot2] image is ready"
printf '  %s\n' "${required_libs[@]}"
printf '  %s\n' "${required_assets[@]}"
