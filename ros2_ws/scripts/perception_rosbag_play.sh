#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  perception_rosbag_play.sh <bag_path_or_name> [rate]

Examples:
  ./perception_rosbag_play.sh ~/rosbags/holoassist/2026-04-01_pass_workspace_monitor_remote_ui
  ./perception_rosbag_play.sh 2026-04-01_rate_gate_trial_01 0.5

Environment:
  HOLOASSIST_BAG_DIR  Default bag root if a bare bag name is provided.
                      Default: $HOME/rosbags/holoassist
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 1 ]]; then
  usage
  exit 0
fi

INPUT="$1"
PLAY_RATE="${2:-1.0}"
DEFAULT_BAG_DIR="${HOLOASSIST_BAG_DIR:-$HOME/rosbags/holoassist}"

if [[ -d "${INPUT}" ]]; then
  BAG_PATH="${INPUT}"
else
  BAG_PATH="${DEFAULT_BAG_DIR%/}/${INPUT}"
fi

if [[ ! -d "${BAG_PATH}" ]]; then
  echo "Bag path not found: ${BAG_PATH}" >&2
  exit 1
fi

echo "=== Perception rosbag playback ==="
echo "Bag   : ${BAG_PATH}"
echo "Rate  : ${PLAY_RATE}x"
echo
echo "Bag info:"
ros2 bag info "${BAG_PATH}"
echo
echo "Playback command:"
echo "  ros2 bag play \"${BAG_PATH}\" --clock --rate \"${PLAY_RATE}\""
echo

ros2 bag play "${BAG_PATH}" --clock --rate "${PLAY_RATE}"
