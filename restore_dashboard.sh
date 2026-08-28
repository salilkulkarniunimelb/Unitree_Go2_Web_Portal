#!/bin/bash
# ---------------------------------------------------------------------------
# restore_dashboard.sh - ONE-COMMAND restore of the QOD Lab Go2 map dashboard.
#
# Run from YOUR LAPTOP. Fully self-contained: recreates the robot_hivemind
# container if missing, installs deps, copies the portal into it, starts it,
# opens the SSH tunnel, and opens the dashboard in your browser.
#
# Usage:
#   bash restore_dashboard.sh
#
# After breaking lab_portal.py:  git checkout 29555bd -- lab_portal.py && bash restore_dashboard.sh
# ---------------------------------------------------------------------------
set -e

SERVER="${SERVER:-salil.kulkarni@10.4.48.11}"
CONTAINER="robot_hivemind"
IMAGE="${IMAGE:-unimelb-humble:base}"
PORT=7860

SSH="ssh -o BatchMode=yes $SERVER"
SCP="scp -o BatchMode=yes"

echo "============================================================"
echo " Restoring QOD Lab Go2 map dashboard -> $SERVER"
echo "============================================================"

# --- 1) Make sure the portal files exist locally ---------------------------
for f in lab_portal.py lab_start.sh; do
    [ -f "$f" ] || { echo "ERROR: $f missing. Run from the repo root, or restore it: git checkout 29555bd -- $f"; exit 1; }
done

# --- 2) Container must exist ------------------------------------------------
echo "[1/5] Checking container $CONTAINER..."
if ! $SSH "docker inspect $CONTAINER >/dev/null 2>&1"; then
    echo "  -> Container missing. Recreating from $IMAGE (host networking)..."
    $SSH "docker run -d --name $CONTAINER --network host --restart unless-stopped \
        $IMAGE sleep infinity"
    echo "  -> Container recreated."
else
    if ! $SSH "docker inspect -f '{{.State.Running}}' $CONTAINER" | grep -q true; then
        echo "  -> Container exists but stopped. Starting it..."
        $SSH "docker start $CONTAINER"
    fi
fi

# --- 3) Copy portal files into /workspace ----------------------------------
echo "[2/5] Copying portal files into container..."
$SCP lab_portal.py lab_start.sh "$SERVER:/tmp/"
$SSH "docker cp /tmp/lab_portal.py $CONTAINER:/workspace/lab_portal.py"
$SSH "docker cp /tmp/lab_start.sh $CONTAINER:/workspace/lab_start.sh"
$SSH "docker exec $CONTAINER chmod +x /workspace/lab_start.sh"

# --- 4) Ensure Python deps --------------------------------------------------
echo "[3/5] Ensuring Python deps (installs only if missing)..."
$SSH "docker exec $CONTAINER bash -lc '
    if ! python3 -c \"import gradio,numpy,cv2\" 2>/dev/null; then
        if ! python3 -m pip --version >/dev/null 2>&1; then
            apt-get update -qq && apt-get install -y -qq python3-pip
        fi
        python3 -m pip install --no-cache-dir numpy opencv-python-headless gradio
    else
        echo deps-ok
    fi
'"

# --- 5) Start the portal ------------------------------------------------------
echo "[4/5] Starting the portal..."
$SSH "docker exec $CONTAINER bash -lc '/workspace/lab_start.sh stop || true'"
$SSH "docker exec $CONTAINER bash -lc '/workspace/lab_start.sh start'"

# --- 6) Tunnel + open browser --------------------------------------------------
echo "[5/5] Opening SSH tunnel..."
if ! lsof -iTCP:$PORT -sTCP:LISTEN >/dev/null 2>&1; then
    nohup $SSH -N -L $PORT:localhost:$PORT >/dev/null 2>&1 < /dev/null &
    disown || true
    sleep 4
fi

echo ""
echo "DONE. Dashboard restored."
echo "  -> http://localhost:$PORT        (open this in your browser)"
echo ""
echo "Repo backup: pushed to GitHub ->"
echo "  git push origin main            (do this once to upload the backup)"
exit 0