#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_DIR="${SCRIPT_DIR}/rosbag_profiles"

usage() {
  cat <<'EOF'
Usage:
  perception_rosbag_record.sh <profile> [duration_s] [date_yyyy-mm-dd] [output_dir]

Profiles:
  test_a | pass_workspace_monitor_remote_ui
  test_b | credit_boundary_candidate_human_entry
  test_c | rate_gate_trial_01

Examples:
  ./perception_rosbag_record.sh test_a 45
  ./perception_rosbag_record.sh test_b 60 2026-04-01
  ./perception_rosbag_record.sh rate_gate_trial_01 75 "$(date +%F)" "$HOME/rosbags/holoassist"

Notes:
  - Requires 'ros2 bag' to be available in the current shell.
  - Uses zstd file compression by default.
  - Recording stops automatically using a built-in timer.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 1 ]]; then
  usage
  exit 0
fi

PROFILE_IN="$1"
DURATION_S="${2:-}"
DATE_STAMP="${3:-$(date +%F)}"
OUTPUT_DIR="${4:-${HOLOASSIST_BAG_DIR:-$HOME/rosbags/holoassist}}"

PROFILE_FILE=""
DEFAULT_DURATION=""
BAG_SUFFIX=""

case "${PROFILE_IN}" in
  test_a|pass_workspace_monitor_remote_ui)
    PROFILE_FILE="${PROFILE_DIR}/pass_workspace_monitor_remote_ui.topics"
    DEFAULT_DURATION="20"
    BAG_SUFFIX="pass_workspace_monitor_remote_ui"
    ;;
  test_b|credit_boundary_candidate_human_entry)
    PROFILE_FILE="${PROFILE_DIR}/credit_boundary_candidate_human_entry.topics"
    DEFAULT_DURATION="25"
    BAG_SUFFIX="credit_boundary_candidate_human_entry"
    ;;
  test_c|rate_gate_trial_01)
    PROFILE_FILE="${PROFILE_DIR}/rate_gate_trial_01.topics"
    DEFAULT_DURATION="30"
    BAG_SUFFIX="rate_gate_trial_01"
    ;;
  *)
    echo "Unknown profile: ${PROFILE_IN}" >&2
    usage
    exit 1
    ;;
esac

if [[ ! -f "${PROFILE_FILE}" ]]; then
  echo "Topic profile file not found: ${PROFILE_FILE}" >&2
  exit 1
fi

if [[ -z "${DURATION_S}" ]]; then
  DURATION_S="${DEFAULT_DURATION}"
fi

if ! [[ "${DURATION_S}" =~ ^[0-9]+$ ]]; then
  echo "Duration must be an integer number of seconds, got: ${DURATION_S}" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

mapfile -t TOPICS < <(sed -e 's/#.*$//' -e 's/[[:space:]]*$//' "${PROFILE_FILE}" | awk 'NF > 0')
if [[ "${#TOPICS[@]}" -eq 0 ]]; then
  echo "No topics resolved from profile ${PROFILE_FILE}" >&2
  exit 1
fi

BAG_NAME="${DATE_STAMP}_${BAG_SUFFIX}"
BAG_PATH="${OUTPUT_DIR%/}/${BAG_NAME}"

CMD=(
  ros2 bag record
  -o "${BAG_PATH}"
  --compression-mode file
  --compression-format zstd
  "${TOPICS[@]}"
)

echo "=== Perception rosbag record ==="
echo "Profile      : ${PROFILE_IN}"
echo "Topics file  : ${PROFILE_FILE}"
echo "Bag name     : ${BAG_NAME}"
echo "Output dir   : ${OUTPUT_DIR}"
echo "Duration (s) : ${DURATION_S}"
echo "Topics:"
for topic in "${TOPICS[@]}"; do
  echo "  - ${topic}"
done
echo
echo "Command:"
printf '  %q' "${CMD[@]}"
echo
echo

SIGNAL_TARGET=""
REC_PID=""

if command -v setsid >/dev/null 2>&1; then
  setsid "${CMD[@]}" &
  REC_PID="$!"
  SIGNAL_TARGET="-$REC_PID"
else
  "${CMD[@]}" &
  REC_PID="$!"
  SIGNAL_TARGET="$REC_PID"
fi

(
  sleep "${DURATION_S}"
  echo
  echo "Duration reached (${DURATION_S}s). Stopping recorder..."
  if kill -0 "${REC_PID}" 2>/dev/null; then
    kill -INT "${SIGNAL_TARGET}" 2>/dev/null || true
    sleep 4
  fi
  if kill -0 "${REC_PID}" 2>/dev/null; then
    echo "Recorder still running; sending SIGTERM..."
    kill -TERM "${SIGNAL_TARGET}" 2>/dev/null || true
    sleep 3
  fi
  if kill -0 "${REC_PID}" 2>/dev/null; then
    echo "Recorder still running; sending SIGKILL..."
    kill -KILL "${SIGNAL_TARGET}" 2>/dev/null || true
  fi
) &
STOPPER_PID="$!"

set +e
wait "${REC_PID}"
STATUS=$?
set -e

kill -TERM "${STOPPER_PID}" 2>/dev/null || true
wait "${STOPPER_PID}" 2>/dev/null || true

if [[ "${STATUS}" -ne 0 && "${STATUS}" -ne 130 && "${STATUS}" -ne 137 && "${STATUS}" -ne 143 ]]; then
  echo "Recording failed with exit code ${STATUS}" >&2
  exit "${STATUS}"
fi

echo
echo "Saved bag:"
echo "  ${BAG_PATH}"
echo "Verify with:"
echo "  ros2 bag info ${BAG_PATH}"
