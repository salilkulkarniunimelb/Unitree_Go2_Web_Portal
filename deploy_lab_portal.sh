#!/bin/bash
# ---------------------------------------------------------------------------
# deploy_lab_portal.sh - One-command deploys of the QOD Lab Go2 map portal.
#
# Run this FROM YOUR LAPTOP. It:
#   1. Copies lab_portal.py to the server + into the robot_hivemind container
#   2. Installs gradio/numpy/opencv inside the container (idempotent)
#   3. (Re)starts the portal on port 7860 inside the container
#
# Usage:
#   bash deploy_lab_portal.sh                 # uses defaults below
#   SERVER=user@host bash deploy_lab_portal.sh
#
# Prereqs: SSH key auth to the server already configured (we did this earlier).
# ---------------------------------------------------------------------------
set -e

SERVER="${SERVER:-salil.kulkarni@10.4.48.11}"
CONTAINER="robot_hivemind"
PORT_FILE="lab_portal.py"
REMOTE_TMP="/tmp/${PORT_FILE}"
BELOW="/workspace/${PORT_FILE}"
LOG="/workspace/lab_portal.log"

echo "============================================================"
echo " Deploying QOD Lab Go2 map portal -> $SERVER"
echo "============================================================"

if [ ! -f "$PORT_FILE" ]; then
    echo "ERROR: $PORT_FILE not found in current dir. Run from the repo root."
    exit 1
fi

echo "[1/4] Copying $PORT_FILE to server..."
scp -o BatchMode=yes "$PORT_FILE" "$SERVER:$REMOTE_TMP"

echo "[2/4] Copying into container $CONTAINER..."
ssh -o BatchMode=yes "$SERVER" "docker cp $REMOTE_TMP $CONTAINER:$BELOW"

echo "[3/4] Ensuring Python deps present (installs only if missing)..."
ssh -o BatchMode=yes "$SERVER" "docker exec $CONTAINER bash -lc 'python3 -c \"import gradio,numpy,cv2\" 2>/dev/null || python3 -m pip install --no-cache-dir numpy opencv-python-headless gradio'"

echo "[4/4] Restarting portal in container..."
ssh -o BatchMode=yes "$SERVER" "docker cp /tmp/lab_start.sh $CONTAINER:/workspace/lab_start.sh"
ssh -o BatchMode=yes "$SERVER" "docker exec $CONTAINER bash -lc 'chmod +x /workspace/lab_start.sh && /workspace/lab_start.sh restart'"

echo ""
echo "DONE. Portal restarted on the server."
echo "Open http://localhost:7860 (or http://$SERVER:7860)"
exit 0
