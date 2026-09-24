#!/bin/bash
# ---------------------------------------------------------------------------
# restore_dashboard.sh - ONE-COMMAND restore of the QOD Lab Go2 map dashboard.
#
# Run from YOUR LAPTOP. Fully self-contained: recreates the robot_hivemind
# container if missing (from the snapshot image, which has the code, Python
# deps and the auto-boot entrypoint baked in), syncs the latest files,
# verifies deps/ffmpeg, starts the portal, opens the SSH tunnel, and opens
# the dashboard in your browser.
#
# The container runs with --restart unless-stopped and its entrypoint
# (boot.sh) auto-starts the portal on every server boot / docker restart, so
# the dashboard comes back on its own even if the machine reboots.
#
# Usage:
#   bash restore_dashboard.sh
#
# After breaking lab_portal.py:  git checkout 29555bd -- lab_portal.py && bash restore_dashboard.sh
# ---------------------------------------------------------------------------
set -e

SERVER="${SERVER:-salil.kulkarni@10.4.48.11}"
HOST="${SERVER#*@}"
CONTAINER="robot_hivemind"
IMAGE="${IMAGE:-unimelb-humble:dashboard}"   # snapshot: code + deps + boot.sh baked in
ENTRYPOINT="bash /workspace/boot.sh"
PORT=7860
FILES="lab_portal.py lab_start.sh start_dashboard.sh boot.sh"
WEB_BACKEND_FILES="web_backend/qod_consumer.py web_backend/qod_detections.py"

SSH="ssh -o BatchMode=yes $SERVER"
SCP="scp -o BatchMode=yes"

echo "============================================================"
echo " Restoring QOD Lab Go2 map dashboard -> $SERVER"
echo "============================================================"

# --- 1) Make sure the portal files exist locally ---------------------------
for f in $FILES; do
    [ -f "$f" ] || { echo "ERROR: $f missing. Run from the repo root, or restore it: git checkout 29555bd -- $f"; exit 1; }
done

# --- 2) Container must exist ------------------------------------------------
echo "[1/7] Checking container $CONTAINER..."
NEW_CONTAINER=0
if ! $SSH "docker inspect $CONTAINER >/dev/null 2>&1"; then
    echo "  -> Container missing. Recreating from $IMAGE (host networking, auto-boot)..."
    $SSH "docker run -d --name $CONTAINER --network host --restart unless-stopped $IMAGE $ENTRYPOINT"
    echo "  -> Container recreated. boot.sh auto-starts the portal."
    NEW_CONTAINER=1
else
    if ! $SSH "docker inspect -f '{{.State.Running}}' $CONTAINER" | grep -q true; then
        echo "  -> Container exists but stopped. Starting it..."
        $SSH "docker start $CONTAINER"
    fi
fi

# --- 3) Copy portal files into /workspace ----------------------------------
echo "[2/7] Copying portal files into container..."
$SCP $FILES "$SERVER:/tmp/"
for f in $FILES; do
    $SSH "docker cp /tmp/$f $CONTAINER:/workspace/$f"
done
$SSH "docker exec $CONTAINER chmod +x /workspace/lab_start.sh /workspace/start_dashboard.sh /workspace/boot.sh"
# The QOD WebRTC camera consumer + detection-subscriber modules (used by
# lab_portal.py).
$SSH "docker exec $CONTAINER mkdir -p /workspace/web_backend"
for wf in $WEB_BACKEND_FILES; do
    if [ -f "$wf" ]; then
        bn=$(basename "$wf")
        echo "  -> Copying $wf into container..."
        $SCP "$wf" "$SERVER:/tmp/$bn"
        $SSH "docker cp /tmp/$bn $CONTAINER:/workspace/web_backend/$bn"
    fi
done
# The dashboard logo and any assets are served from the container, so copy
# the whole assets/ folder (missing logo caused a FileNotFoundError crash).
if [ -d assets ]; then
    echo "  -> Copying assets/ into container..."
    if ! $SSH "docker exec $CONTAINER ls /workspace/assets >/dev/null 2>&1"; then
        $SSH "docker exec $CONTAINER mkdir -p /workspace/assets"
    fi
    for f in assets/*; do
        [ -f "$f" ] || continue
        bn=$(basename "$f")
        $SCP "$f" "$SERVER:/tmp/$bn"
        $SSH "docker cp /tmp/$bn $CONTAINER:/workspace/assets/$bn"
    done
fi

# --- 4) Ensure Python deps --------------------------------------------------
echo "[3/7] Ensuring Python deps (installs only if missing)..."
# NOTE: numpy is pinned to <2 because the ROS cv_bridge package was compiled
# against NumPy 1.x and crashes with NumPy 2.x (_ARRAY_API not found), and
# opencv is pinned to <5 to stay numpy<2 compatible.
$SSH "docker exec $CONTAINER bash -lc '
    if ! python3 -c \"import gradio,numpy,cv2\" 2>/dev/null; then
        if ! python3 -m pip --version >/dev/null 2>&1; then
            apt-get update -qq && apt-get install -y -qq python3-pip
        fi
        python3 -m pip install --no-cache-dir \"numpy<2\" \"opencv-python-headless<5\" gradio
    elif python3 -c \"import numpy,sys; sys.exit(0 if numpy.__version__.startswith(chr(49)) else 1)\" 2>/dev/null; then
        echo deps-ok
    else
        echo \"  -> numpy >= 2 detected, downgrading to <2 for cv_bridge compat...\"
        python3 -m pip install --no-cache-dir \"numpy<2\" \"opencv-python-headless<5\"
    fi
'"

# --- 5) Ensure ffmpeg (for the live H.264 camera pipeline) -----------------
echo "[4/7] Ensuring ffmpeg (installs only if missing)..."
$SSH "docker exec $CONTAINER bash -lc '
    if ! command -v ffmpeg >/dev/null 2>&1; then
        echo \"  -> installing ffmpeg...\"
        apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends ffmpeg
    else
        echo ffmpeg-ok
    fi
'"

# --- 5) Ensure QOD WebRTC consumer deps (aiortc, websockets, redis) --------------
echo "[5/7] Ensuring QOD WebRTC camera deps (aiortc, websockets, redis)..."
$SSH "docker exec $CONTAINER bash -lc '
    if ! python3 -c \"import aiortc,websockets,redis\" 2>/dev/null || ! python3 -c \"import websockets;assert int(websockets.__version__.split(chr(46))[0])>=10\" 2>/dev/null; then
        echo \"  -> installing/fixing aiortc + websockets + redis (QOD camera/detection consumers)...\"
        python3 -m pip install --no-cache-dir \"aiortc>=1.15.0\" \"websockets>=10,<17\" redis
    else
        echo webrtc-deps-ok
    fi
'"

# --- 6) Start the portal ------------------------------------------------------
echo "[6/7] Starting the portal..."
if [ "$NEW_CONTAINER" = "1" ]; then
    # A fresh container already has boot.sh auto-starting the portal.
    echo "  -> New container: boot.sh is starting the portal..."
else
    $SSH "docker exec $CONTAINER bash -lc '/workspace/lab_start.sh stop || true'"
    $SSH "docker exec $CONTAINER bash -lc '/workspace/lab_start.sh start'"
fi

echo "  -> Waiting for the portal to answer on port $PORT..."
for i in $(seq 1 30); do
    if $SSH "curl -s --noproxy '*' -o /dev/null http://$HOST:$PORT/ 2>/dev/null"; then
        echo "  -> Portal is UP."
        break
    fi
    sleep 2
done

# --- 7) Tunnel + open browser --------------------------------------------------
echo "[7/7] Opening SSH tunnel..."
if ! lsof -iTCP:$PORT -sTCP:LISTEN >/dev/null 2>&1; then
    nohup $SSH -N -L $PORT:localhost:$PORT >/dev/null 2>&1 < /dev/null &
    disown || true
    sleep 4
fi

echo ""
echo "DONE. Dashboard restored and auto-start enabled."
echo "  -> http://localhost:$PORT       (tunneled, this laptop)"
echo "  -> http://$HOST:$PORT     (anyone on the network)"
exit 0