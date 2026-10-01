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
echo "   7860  dashboard (Mapping / Fleet View / Patrolling / Foxglove view)"
echo "   8766  patrol planner, embedded in the dashboard"
echo "   8767  3D viewer for the Foxglove view tab"
echo " Keep this terminal open. Press Ctrl+C to close."
echo " Then open http://localhost:7860 in your browser."
echo "============================================================"
echo ""

# 8767 is the Foxglove-style 3D viewer's port. The dashboard embeds it in an
# iframe by absolute URL, so without this forward the tab renders blank while
# everything else works -- a confusing failure, hence the explicit listing above.
ssh -N -L 7860:localhost:7860 -L 8766:localhost:8766 -L 8767:localhost:8767 -p "$PORT" "$USER_HOST"
