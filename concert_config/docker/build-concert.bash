#!/bin/bash
set -e

# Simple but powerful wrapper script that combines Concert configuration
# with full passthrough of arguments to the submodule build system

# Navigate to script directory for reliable relative path operations
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
cd "$SCRIPT_DIR"

# Determine which config to load based on argument or default to ROS2
if [[ "$1" == "ros1" ]] || [[ "$1" == "--ros1" ]]; then
    CONFIG_FILE="concert-config_ros1.env"
    BUILD_DIR="../docker-base/robot-template/_build/robot-focal-ros1"
    shift  # Remove the ros1 argument before passing to build.bash
else
    # Default to ROS2, but also handle explicit ros2 flag
    if [[ "$1" == "ros2" ]] || [[ "$1" == "--ros2" ]]; then
        shift  # Remove the ros2 argument
    fi
    CONFIG_FILE="concert-config_ros2.env"
    BUILD_DIR="../docker-base/robot-template/_build/robot-noble-ros2"
fi

# Check if docker-base submodule exists
if [ ! -d "../docker-base" ]; then
    echo "ERROR: docker-base submodule not found!"
    echo "Please initialize the submodule with:"
    echo "  git submodule update --init"
    exit 1
fi

# Load Concert-specific configuration into environment
# This makes all CONCERT configuration available to the submodule script
source "$CONFIG_FILE"

# Display configuration for user verification
# This helps users confirm their environment is set up correctly
echo "========================================"
echo "Building Concert docker images"
echo "========================================"
echo "  ROBOT_NAME: $ROBOT_NAME"
echo "  RECIPES_TAG: $RECIPES_TAG"
echo "  BASE_IMAGE_NAME: $BASE_IMAGE_NAME"
echo "  DISTRO: $DISTRO"
echo "  ROS_VERSION: $ROS_VERSION"
echo "  ROBOT_PACKAGES: $ROBOT_PACKAGES"
echo "  ADDITIONAL_PACKAGES: $ADDITIONAL_PACKAGES"
echo "  DOCKER_REGISTRY: $DOCKER_REGISTRY"
echo "  BUILD_DIR: $BUILD_DIR"

# Show what arguments are being passed to help with debugging
if [ $# -gt 0 ]; then
    echo "  ARGUMENTS: $@"
else
    echo "  ARGUMENTS: (none - using submodule defaults)"
fi
echo "========================================"

# Navigate to the submodule build directory
if [ ! -d "$BUILD_DIR" ]; then
    echo "ERROR: Build directory does not exist: $BUILD_DIR"
    echo "Make sure docker-base submodule is properly initialized:"
    echo "  git submodule update --init --recursive"
    exit 1
fi

cd "$BUILD_DIR"
echo "Using build directory: $(pwd)"

# Execute the submodule build script with ALL arguments passed through
# The "$@" variable expands to all command-line arguments exactly as received
# This preserves argument quoting, spacing, and special characters
echo ""
echo "Delegating to docker-base build script..."
echo ""
./build.bash "$@"

# Note: Any arguments you pass to this script (like --push, --pull, --help)
# will be automatically forwarded to the submodule's build.bash script
# This means your wrapper inherits all the capabilities of the submodule
