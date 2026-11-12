SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

export XBOT2_DEFAULT_HW=ec_idle
export ECAT_MASTER_CONFIG="$SCRIPT_DIR/ecat/ecat_config.yaml"
alias ecat_master="repl -f $ECAT_MASTER_CONFIG"

export CONCERT_LAUNCHER_ROS2_CONFIG="$SCRIPT_DIR/gui/ros2/concert_launcher_config.yaml"

# XBot2 configs - using ModularBot naming
export XBOT2_CONFIG_ROS2="$SCRIPT_DIR/xbot2/ModularBot.yaml"

export CONCERT_LAUNCHER_DEFAULT_CONFIG="$CONCERT_LAUNCHER_ROS2_CONFIG"
set_xbot2_config "$XBOT2_CONFIG_ROS2"
