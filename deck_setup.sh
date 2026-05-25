#!/bin/bash
# One-time setup for the Steam Deck (run in Desktop Mode with internet).
# Installs Python dependencies into ~/.local/ (persists across SteamOS updates).
#
# After running this, switch the Deck to the robot WiFi / USB-C Ethernet
# and use deck_dashboard.sh to launch.
set -e

echo "=== HoloAssist Steam Deck Setup ==="
echo ""

# Ensure pip is available
python3 -m ensurepip --user 2>/dev/null || true

# Install dependencies (user-local, no root needed)
echo "Installing PyQt5 + websockets..."
python3 -m pip install --user --upgrade pip
python3 -m pip install --user PyQt5 websockets

echo ""
echo "=== Setup complete ==="
echo ""
echo "USB-C Ethernet setup (run once on the Deck):"
echo "  On the laptop:  sudo ip addr add 10.0.0.1/24 dev <usb-iface>"
echo "  On the Deck:    sudo ip addr add 10.0.0.2/24 dev <usb-iface>"
echo "  Test:           ping 10.0.0.1"
echo ""
echo "To run:"
echo "  Laptop:  ./bridge.sh"
echo "  Deck:    ./deck_dashboard.sh"
echo ""
echo "Or with a custom bridge URL:"
echo "  ./deck_dashboard.sh ws://192.168.0.102:9090"
