#!/bin/bash
# Run the HoloAssist dashboard on the Steam Deck.
# Connects to the laptop's bridge server over USB-C Ethernet.
#
# Usage:
#   ./deck_dashboard.sh                          # default ws://10.0.0.1:9090
#   ./deck_dashboard.sh ws://192.168.0.102:9090  # custom URL
#   ./deck_dashboard.sh --no-fullscreen          # windowed mode
BRIDGE_URL="${1:-ws://10.0.0.1:9090}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ "$1" = "--no-fullscreen" ]; then
    BRIDGE_URL="${2:-ws://10.0.0.1:9090}"
    python3 "$SCRIPT_DIR/dashboard/main.py" --bridge "$BRIDGE_URL"
else
    python3 "$SCRIPT_DIR/dashboard/main.py" --bridge "$BRIDGE_URL" --fullscreen
fi
