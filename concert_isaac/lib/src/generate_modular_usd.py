import os 
import subprocess
import argparse


"""
Generate a modular USD file from modular's python script

This script operates as follows:
1. It takes in the path to a modular python script and an output name for the USD file.
2. It runs the modular python script with the '-o urdf' and '-o srdf' flags to get the URDF and SRDF strings.
3. It writes the URDF and SRDF strings to the appropriate assets/urdf and assets/srdf directories.
4. It then invokes the Isaac convert_urdf.py script inside the docker container to generate the USD file from the URDF. Any extra arguments passed to this script after a '--' separator are forwarded verb

Usage:
    # Run without extra args
    python generate_modular_usd.py my_robot.py my_robot

    # Forward extra args to convert_urdf.py (e.g. --merge-joints, --collision-approximation)
    python generate_modular_usd.py my_robot.py my_robot -- --merge-joints --collision-approximation convexDecomposition
"""


parser = argparse.ArgumentParser(
    description="Generate a modular USD file from modular's python script",
    epilog="Arguments after -- are forwarded verbatim to the Isaac convert_urdf.py script.",
)
parser.add_argument("modular_py", type=str, help="Path to the modular robot description script")
parser.add_argument("output_name", type=str, help="Name of the output USD file (without extension)")

args, extra_args = parser.parse_known_args()
# Strip a leading '--' separator if present (e.g. script.py foo bar -- --merge-joints)
if extra_args and extra_args[0] == "--":
    extra_args = extra_args[1:]

# now we need to find the docker container where isaac is running
script_dir = os.path.dirname(os.path.abspath(__file__))
docker_dir = os.path.join(script_dir, "..", "..", "docker")

# look for a folder containing a compose.yml file 
isaac_container = None
for entry in os.listdir(docker_dir):
    entry_path = os.path.join(docker_dir, entry)
    if os.path.isdir(entry_path) and "compose.yml" in os.listdir(entry_path):
        isaac_container = os.path.abspath(entry_path)
        print(f"Found Isaac container: {isaac_container}")
        break

# first, we call the modular python script to get the URDF and SRDF strings
urdf = subprocess.check_output(["python3", args.modular_py, "-o", "urdf"]).decode("utf-8")
srdf = subprocess.check_output(["python3", args.modular_py, "-o", "srdf"]).decode("utf-8")

if '-link.stl' in urdf:
    raise ValueError("URDF contains '-link.stl' which is not a valid filename. Please check the modular script for correct STL filenames (should be 'concert_elbow_link.stl' not 'concert_elbow-link.stl').")

# assets dir 
assets_dir = os.path.join(script_dir, "..", "..", "assets")

# write urdf, srdf
urdf_path = os.path.join(assets_dir, "urdf", f"{args.output_name}.urdf")
with open(urdf_path, "w") as f:
    if os.path.exists(urdf_path):
        print(f"Warning: URDF file already exists at {urdf_path}, overwriting...")
    f.write(urdf)

srdf_path = os.path.join(assets_dir, "srdf", f"{args.output_name}.srdf")
with open(srdf_path, "w") as f:
    if os.path.exists(srdf_path):
        print(f"Warning: SRDF file already exists at {srdf_path}, overwriting...")
    f.write(srdf)

# create output directory for USD if it doesn't exist
usd_output_dir_host = os.path.join(assets_dir, "usd", args.output_name)
if not os.path.exists(usd_output_dir_host):
    os.makedirs(usd_output_dir_host)

# invoke usd generation script inside the isaac container 
usd_gen_script = '/workspace/isaaclab/scripts/tools/convert_urdf.py'
usd_input_path = f"/workspace/iit-concert-ros-pkg/concert_isaac/assets/urdf/{args.output_name}.urdf"
usd_output_dir = f"/workspace/iit-concert-ros-pkg/concert_isaac/assets/usd/{args.output_name}"
usd_output_path = f"{usd_output_dir}/{args.output_name}.usd"
docker_service_name = "isaac-sim"  # this should match the service name in compose.yml
extra_args_str = " ".join(extra_args)
cmd = f'docker compose -f {isaac_container}/compose.yml exec {docker_service_name} bash -ic "export AMENT_PREFIX_PATH=/workspace/iit-concert-ros-pkg; python {usd_gen_script} {usd_input_path} {usd_output_path} {extra_args_str}"'

print(f"Running command: {cmd}")
subprocess.run(cmd, shell=True, check=True)