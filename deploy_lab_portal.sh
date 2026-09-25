#!/bin/bash
# ---------------------------------------------------------------------------
# deploy_lab_portal.sh - One-command deploys of the QOD Lab Go2 map portal.
#
# Run this FROM YOUR LAPTOP to push an updated dashboard to the server. It:
#   1. Copies lab_portal.py (+ controller scripts) into the robot_hivemind
#      container
#   2. Ensures gradio/numpy/opencv are present (no-op once installed)
#   3. Restarts the portal on port 7860
#
# The robot_hivemind container is created by restore_dashboard.sh and
# auto-starts the portal on boot (boot.sh entrypoint), so the dashboard comes
# back on its own after a server reboot.
#
# Usage:
#   bash deploy_lab_portal.sh                 # uses defaults below
#   SERVER=user@host bash deploy_lab_portal.sh
#
# Prereqs: SSH key auth to the server already configured (we did this earlier).
# ---------------------------------------------------------------------------
set -e

SERVER="${SERVER:-salil.kulkarni@10.4.48.11}"
HOST="${SERVER#*@}"
CONTAINER="robot_hivemind"
PORT=7860
FILES="lab_portal.py lab_start.sh start_dashboard.sh boot.sh"

echo "============================================================"
echo " Deploying QOD Lab Go2 map portal -> $SERVER"
echo "============================================================"

if [ ! -f lab_portal.py ]; then
    echo "ERROR: lab_portal.py not found in current dir. Run from the repo root."
    exit 1
fi

echo "[1/4] Checking container $CONTAINER exists..."
if ! ssh -o BatchMode=yes "$SERVER" "docker inspect $CONTAINER >/dev/null 2>&1"; then
    echo "ERROR: container '$CONTAINER' does not exist on the server."
    echo "Create it once with:  bash restore_dashboard.sh"
    exit 1
fi

echo "[2/4] Copying dashboard files into container..."
scp -o BatchMode=yes $FILES "$SERVER:/tmp/"
ssh -o BatchMode=yes "$SERVER" "for f in $FILES; do docker cp /tmp/\$f $CONTAINER:/workspace/\$f; done"
ssh -o BatchMode=yes "$SERVER" "docker exec $CONTAINER chmod +x /workspace/lab_start.sh /workspace/start_dashboard.sh /workspace/boot.sh"

echo "[3/4] Ensuring Python deps present (installs only if missing)..."
ssh -o BatchMode=yes "$SERVER" "docker exec $CONTAINER bash -lc 'python3 -c \"import gradio,numpy,cv2\" 2>/dev/null || python3 -m pip install --no-cache-dir \"numpy<2\" \"opencv-python-headless<5\" gradio'"

echo "[4/4] Restarting portal in container..."
ssh -o BatchMode=yes "$SERVER" "docker exec $CONTAINER bash -lc '/workspace/lab_start.sh restart'"

echo ""
echo "DONE. Portal updated and restarted on the server."
echo "Anyone on the network can open  http://$HOST:$PORT"
exit 0