#!/bin/bash
# ---------------------------------------------------------------------------
# ssh_tunnel.sh - Run THIS script ON YOUR LAPTOP (macOS/Linux)
#
# Opens an SSH tunnel so you can view the portal that is running on the
# Telstra server as http://localhost:7860 on your own machine.
#
# Usage:  bash ssh_tunnel.sh <username>@<server-ip>
# Example: bash ssh_tunnel.sh sallu@192.168.1.50
# ---------------------------------------------------------------------------
set -e

USER_HOST="${1:?Usage: bash ssh_tunnel.sh <username>@<server-ip>}"
PORT="${2:-22}"

echo "============================================================"
echo " Opening tunnel: $USER_HOST -> localhost:7860"
echo " Keep this terminal open. Press Ctrl+C to close."
echo " Then open http://localhost:7860 in your browser."
echo "============================================================"
echo ""

ssh -N -L 7860:localhost:7860 -p "$PORT" "$USER_HOST"
