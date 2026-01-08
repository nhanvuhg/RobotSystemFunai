#!/usr/bin/env bash
set -euo pipefail

# Wrapper to run Festo gripper node inside a virtualenv and install deps if missing.
# Location: ~/ros2_ws/run_gripper_node.sh

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$WS/venv_gripper"
GRIPPER_SCRIPT="$WS/src/robot_control_main/gripper_festo_node.py"

echo "[gripper-wrapper] workspace: $WS"

# Create venv if missing
if [ ! -d "$VENV_DIR" ]; then
  echo "[gripper-wrapper] Creating venv at $VENV_DIR"
  python3 -m venv --system-site-packages "$VENV_DIR"
  "$VENV_DIR/bin/pip" install --upgrade pip setuptools
  echo "[gripper-wrapper] Installing Python deps (this may take a while)"
  # Install festo-cpx-io with CLI extras and rich for logging
  "$VENV_DIR/bin/pip" install "festo-cpx-io[cli]" rich || {
    echo "[gripper-wrapper] pip install failed; please inspect network or install manually" >&2
  }
fi

if [ ! -f "$GRIPPER_SCRIPT" ]; then
  echo "[gripper-wrapper] ERROR: gripper script not found: $GRIPPER_SCRIPT" >&2
  exit 1
fi

echo "[gripper-wrapper] Activating venv and launching gripper node"
# shellcheck source=/dev/null
source "$VENV_DIR/bin/activate"
python3 "$GRIPPER_SCRIPT"
