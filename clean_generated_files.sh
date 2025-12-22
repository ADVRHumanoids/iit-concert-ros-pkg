#!/bin/bash
# Clean generated files (e.g., from xacro, gazebo model generation)

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "Cleaning generated files in iit-concert-ros-pkg..."

# Clean generated URDFs
find "$SCRIPT_DIR" -name "*.urdf" -type f -delete

# Clean generated meshes (if any are auto-generated)
# find "$SCRIPT_DIR" -path "*/generated/*" -type f -delete

echo "Done."
