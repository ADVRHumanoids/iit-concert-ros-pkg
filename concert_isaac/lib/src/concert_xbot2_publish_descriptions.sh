#!/usr/bin/env bash

set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /opt/xbot/setup.sh
source /home/user/xbot2_ws/setup.bash

if [ -f /home/user/env/bin/activate ]; then
    source /home/user/env/bin/activate
fi
set -u

export PYTHONPATH="/home/user/xbot2_ws/src/modular/src${PYTHONPATH:+:${PYTHONPATH}}"

repo_root=/workspace/iit-concert-ros-pkg
output_dir=/tmp/concert_xbot2_descriptions
generator=/home/user/xbot2_ws/src/iit-concert-ros-pkg/concert_examples/src/concert_example.py
usd_root="${repo_root}/concert_usd"
urdf_root="${repo_root}/concert_urdf"
srdf_root="${repo_root}/concert_srdf"
urdf_file="${output_dir}/modularbot.urdf"
srdf_file="${output_dir}/modularbot.srdf"
params_file="${output_dir}/robot_description_publisher.yaml"

mkdir -p "${output_dir}"

for required_file in \
    "${usd_root}/usd/concert_complete/concert_complete.usd" \
    "${urdf_root}/urdf/concert_complete.urdf" \
    "${srdf_root}/srdf/ModularBot.srdf"; do
    if [ ! -f "${required_file}" ]; then
        echo "[concert-xbot2] missing committed deployment asset: ${required_file}" >&2
        exit 1
    fi
done

python3 "${generator}" -o urdf -a gazebo_urdf:=false floating_base:=true -r modularbot >"${urdf_file}"
python3 "${generator}" -o srdf -a gazebo_urdf:=false -r modularbot >"${srdf_file}"

# Post-process only the two upstream generator issues that were already present
# in the previous validated flow.
sed 's/<material>/<material name="imu_material">/' "${urdf_file}" > "${urdf_file}.tmp" && mv "${urdf_file}.tmp" "${urdf_file}"
sed '/<group name="hands">/,/<\/group>/d' "${srdf_file}" > "${srdf_file}.tmp" && mv "${srdf_file}.tmp" "${srdf_file}"

{
    printf "/**:\n"
    printf "  ros__parameters:\n"
    printf "    robot_description: |\n"
    sed 's/^/      /' "${urdf_file}"
    printf "    robot_description_semantic: |\n"
    sed 's/^/      /' "${srdf_file}"
} >"${params_file}"

exec ros2 run xbot2_ros robot_description_publisher --ros-args --params-file "${params_file}"
