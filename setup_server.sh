#!/bin/bash
# ---------------------------------------------------------------------------
# setup_server.sh - Run THIS script ON THE TELSTRA SERVER (next to the Go2)
#
# What it does:
#   1. Detects your network interface and writes it into config.py (INTERFACE)
#   2. Lets you point UNITREE_ROS2_SETUP_SH_PATH to the real unitree_ros2
#   3. Installs the Python dependencies
#   4. Starts the portal on port 7860
#
# Usage:  bash setup_server.sh
# ---------------------------------------------------------------------------
set -e

echo "============================================================"
echo " Unitree Go2 Web Portal - Server Setup"
echo "============================================================"

# --- Where is this repo? ---------------------------------------------------
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"
echo "[1/4] Repo location: $REPO_DIR"

# --- Detect default network interface ---------------------------------------
echo "[2/4] Detecting your network interface..."
iface="$(ip -o -4 route show to default 2>/dev/null | awk '{print $5}' | head -n1)"
if [ -z "$iface" ]; then
    echo "  -> Could not auto-detect. Listing interfaces below."
    ip a
    echo "  -> Type the interface name that connects to the robot network (e.g. enp132s0):"
    read -r iface
fi
echo "  -> Detected interface: $iface"

# Update INTERFACE in config.py
sed -i.bak "s|^INTERFACE=.*|INTERFACE=\"$iface\" # auto-set by setup_server.sh|" config.py
echo "  -> config.py INTERFACE set to: $iface"

# --- Locate unitree_ros2 setup.sh -------------------------------------------
echo ""
echo "[3/4] Looking for unitree_ros2 setup.sh..."
found=""
for p in "$HOME/unitree_ros2/setup.sh" /home/*/unitree_ros2/setup.sh; do
    if [ -f "$p" ]; then found="$p"; break; fi
done
if [ -n "$found" ]; then
    echo "  -> Found $found . Writing it to config.py"
    # escape slashes for sed
    esc="$(printf '%s' "$found" | sed 's|/|\\/|g')"
    sed -i.bak "s|^UNITREE_ROS2_SETUP_SH_PATH=.*|UNITREE_ROS2_SETUP_SH_PATH=\"$found\"|" config.py
else
    echo "  -> Not found. Edit config.py and set UNITREE_ROS2_SETUP_SH_PATH manually."
fi

# --- Install dependencies ----------------------------------------------------
echo ""
echo "[4/4] Installing Python dependencies..."
pip install -r requirements.txt || pip3 install -r requirements.txt
echo ""
echo "Dependencies installed."

echo ""
echo "============================================================"
echo " Setup complete. Config values now in config.py:"
grep -nE "^(INTERFACE|UNITREE_ROS2_SETUP_SH_PATH)=" config.py || true
echo "============================================================"
echo ""
echo "Starting the portal..."
echo "  -> Wait for 'Running on local URL: http://0.0.0.0:7860'"
echo ""
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -n "$found" ]; then source "$found" 2>/dev/null || true; fi
exec python main.py
