SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

export XBOT2_DEFAULT_HW=sim
export CONCERT_LAUNCHER_DEFAULT_CONFIG="$SCRIPT_DIR/gui/ros2/concert_launcher_config.yaml"
set_xbot2_config "$SCRIPT_DIR/modular.yaml"
