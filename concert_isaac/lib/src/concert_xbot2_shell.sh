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

publish_log=/tmp/concert_xbot2_descriptions/publisher.log
socket_dir=/tmp/.xbot2_isaac
mkdir -p "$(dirname "${publish_log}")"
mkdir -p "${socket_dir}"

# Fresh containers often reuse the same low PID, so stale client sockets from a
# previous aborted xbot2 run can collide with the next bind().
find "${socket_dir}" -maxdepth 1 -name 'xbot2_isaac_server.sock.client.*' -delete 2>/dev/null || true

# Run the publisher in a separate session so an accidental Ctrl+C at the shell
# prompt does not forward SIGINT to the ROS2 publisher process.
setsid /workspace/iit-concert-ros-pkg/concert_isaac/lib/src/concert_xbot2_publish_descriptions.sh >"${publish_log}" 2>&1 < /dev/null &
publisher_pid=$!

for topic in /robot_description /robot_description_semantic; do
    ready=0
    for _ in $(seq 1 30); do
        if ! kill -0 "${publisher_pid}" 2>/dev/null; then
            echo "[concert-xbot2] description publisher exited unexpectedly" >&2
            cat "${publish_log}" >&2
            exit 1
        fi

        if timeout 5 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
            ready=1
            break
        fi

        sleep 1
    done

    if [ "${ready}" -ne 1 ]; then
        echo "[concert-xbot2] timed out waiting for topic ${topic}" >&2
        cat "${publish_log}" >&2
        exit 1
    fi
done

cd /home/user/xbot2_ws/src/iit-concert-ros-pkg/concert_config/xbot2

exec bash
