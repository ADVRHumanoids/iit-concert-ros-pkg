#!/bin/bash
# setup-host.sh - Run this once to set up your host environment

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SETUP_PATH="$SCRIPT_DIR/setup.sh"
LINE_TO_ADD="source $SETUP_PATH"

if grep -qFx "$LINE_TO_ADD" ~/.bashrc; then
    echo "Concert configuration already in .bashrc"
else
    echo "Adding Concert configuration to .bashrc"
    echo "" >> ~/.bashrc
    echo "# Concert configuration" >> ~/.bashrc
    echo "$LINE_TO_ADD" >> ~/.bashrc
    echo "Configuration added. Please run 'source ~/.bashrc' or open a new terminal."
fi