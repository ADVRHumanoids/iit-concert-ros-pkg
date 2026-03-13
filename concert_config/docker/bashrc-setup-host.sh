#!/bin/bash
# setup-host.sh - Run this once to set up your host environment

IS_SOURCED=0
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  IS_SOURCED=1
else
  set -euo pipefail
fi

finish() {
  local code="$1"
  if [[ $IS_SOURCED -eq 1 ]]; then
    return "$code"
  else
    exit "$code"
  fi
}

usage() {
  cat <<'EOF'
Usage: bashrc-setup-host.sh [--ros1 | --ros2]

Adds (or updates) the Concert configuration section in ~/.bashrc so that the
requested ROS version is exported before sourcing concert_config/setup.sh.
EOF
  finish 1
}

if [[ $# -eq 0 ]]; then
  echo "ERROR: You must specify --ros1 or --ros2." >&2
  usage
fi

ros_variant=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros1)
      ros_variant="ros1"
      shift
      ;;
    --ros2)
      ros_variant="ros2"
      shift
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "ERROR: Unknown option '$1'." >&2
      usage
      ;;
  esac
done

if [[ -z "$ros_variant" ]]; then
  echo "ERROR: Missing required flag --ros1 or --ros2." >&2
  usage
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SETUP_PATH="$SCRIPT_DIR/../setup.sh"
BASHRC_PATH="$HOME/.bashrc"
START_MARK="# >>> Concert configuration (managed by bashrc-setup-host.sh)"
END_MARK="# <<< Concert configuration (managed by bashrc-setup-host.sh)"
LEGACY_LINE="source $SETUP_PATH"
LEGACY_COMMENT="# Concert configuration"

touch "$BASHRC_PATH"

remove_block() {
  local tmp
  tmp="$(mktemp)"
  awk -v start="$START_MARK" -v end="$END_MARK" '
    $0==start {in_block=1; next}
    $0==end && in_block {in_block=0; next}
    !in_block {print}
  ' "$BASHRC_PATH" > "$tmp"
  mv "$tmp" "$BASHRC_PATH"
}

remove_line() {
  local line="$1" tmp
  tmp="$(mktemp)"
  awk -v needle="$line" '$0==needle {next} {print}' "$BASHRC_PATH" > "$tmp"
  mv "$tmp" "$BASHRC_PATH"
}

if grep -qF "$START_MARK" "$BASHRC_PATH"; then
  remove_block
fi

if grep -qF "$LEGACY_LINE" "$BASHRC_PATH"; then
  remove_line "$LEGACY_LINE"
fi

if grep -qF "$LEGACY_COMMENT" "$BASHRC_PATH"; then
  remove_line "$LEGACY_COMMENT"
fi

BLOCK_CONTENT=$(cat <<EOF
$START_MARK
export ROS_VERSION=$ros_variant
source $SETUP_PATH
$END_MARK
EOF
)

{
  echo ""
  echo "$BLOCK_CONTENT"
} >> "$BASHRC_PATH"

echo "Configured ~/.bashrc for ROS variant '$ros_variant'."
echo "Please run 'source ~/.bashrc' or open a new terminal to apply the change."
finish 0