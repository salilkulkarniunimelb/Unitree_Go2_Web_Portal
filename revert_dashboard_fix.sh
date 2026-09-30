#!/bin/bash
# ---------------------------------------------------------------------------
# revert_dashboard_fix.sh - roll the dashboard back to the pre-fix state.
#
# Use this if the watchdog / lab_start.sh changes cause a problem. It restores
# the original scripts, the original docker image, and the original container.
#
#   bash revert_dashboard_fix.sh              # roll back (asks first)
#   bash revert_dashboard_fix.sh --yes        # roll back without prompting
#   bash revert_dashboard_fix.sh --status     # show what would be restored
#
# Run from the repo root.
# ---------------------------------------------------------------------------
set -e

SERVER="${SERVER:-salil.kulkarni@10.4.48.11}"
HOST="${SERVER#*@}"
CONTAINER="robot_hivemind"
PORT=7860

GOOD_IMAGE="unimelb-humble:dashboard"
# NOTE: this is the pre-fix snapshot, so reverting also rolls lab_portal.py back
# to the older code that was baked in (Sep 17). To keep the NEWEST dashboard code
# but drop only the watchdog, re-deploy instead:
#   bash restore_dashboard.sh
OLD_IMAGE="unimelb-humble:dashboard-prefix-backup"
OLD_RUNNING="unimelb-humble:robot_hivemind-prefix-backup"

SSH="ssh -o BatchMode=yes $SERVER"
REPO="$(cd "$(dirname "$0")" && pwd)"
BACKUP_DIR="$REPO/.dashboard_fix_backup"
FILES="boot.sh lab_start.sh restore_dashboard.sh deploy_lab_portal.sh start_dashboard.sh"

ASSUME_YES=0
[ "${1:-}" = "--yes" ] && ASSUME_YES=1
STATUS_ONLY=0
[ "${1:-}" = "--status" ] && STATUS_ONLY=1

echo "============================================================"
echo " Revert QOD Lab Go2 map dashboard to the pre-fix state"
echo "============================================================"

# --- Locate the backup ------------------------------------------------------
if [ ! -d "$BACKUP_DIR" ]; then
    echo "ERROR: no backup found at $BACKUP_DIR"
    echo "You can still restore the old image with:"
    echo "  $SSH \"docker tag $OLD_IMAGE $GOOD_IMAGE\""
    exit 1
fi
TS=$(ls -1 "$BACKUP_DIR" | grep -E '^[0-9]{8}_[0-9]{6}$' | sort | tail -1)
[ -n "$TS" ] || { echo "ERROR: no timestamped backup inside $BACKUP_DIR"; exit 1; }
SRC="$BACKUP_DIR/$TS"

echo "Local backup:  $SRC"
echo "Restoring image tag: $GOOD_IMAGE  ->  $OLD_IMAGE"
echo "Recreating container: $CONTAINER from $OLD_IMAGE"
echo ""

if [ "$STATUS_ONLY" = "1" ]; then
    echo "--status only, nothing changed."
    $SSH "docker images --format '{{.Repository}}:{{.Tag}} {{.ID}}' | grep -E 'dashboard|hivemind'" || true
    exit 0
fi

if [ "$ASSUME_YES" != "1" ]; then
    printf "Revert now? This replaces the running container. [y/N] "
    read -r reply
    case "$reply" in
        [yY]*) ;;
        *) echo "Aborted - nothing changed."; exit 0 ;;
    esac
fi

# --- 1) Restore the scripts locally -----------------------------------------
echo "[1/4] Restoring original scripts from $TS..."
for f in $FILES; do
    if [ -f "$SRC/$f" ]; then
        cp -p "$SRC/$f" "$REPO/$f"
        echo "  -> restored $f"
    fi
done

# --- 2) Put the old image back under the name the scripts use ---------------
echo "[2/4] Restoring the original image tag..."
if ! $SSH "docker image inspect $OLD_IMAGE >/dev/null 2>&1"; then
    echo "  !! $OLD_IMAGE is missing on the server."
    echo "  !! Falling back to $OLD_RUNNING (the pre-fix running container)."
    OLD_IMAGE="$OLD_RUNNING"
fi
$SSH "docker tag $OLD_IMAGE $GOOD_IMAGE"
echo "  -> $GOOD_IMAGE now points at the pre-fix image."

# --- 3) Recreate the container exactly as it was ----------------------------
echo "[3/4] Recreating the container from the pre-fix image..."
$SSH "docker rm -f $CONTAINER >/dev/null 2>&1 || true"
$SSH "docker run -d --name $CONTAINER --network host --restart unless-stopped \
      $GOOD_IMAGE bash /workspace/boot.sh >/dev/null"
echo "  -> container recreated (no watchdog: it will NOT auto-restart the portal)."

# --- 4) Start the portal the old way, then verify ----------------------------
echo "[4/4] Starting the portal the pre-fix way..."
$SSH "docker exec $CONTAINER bash -lc '/workspace/lab_start.sh start'" || true

PORTAL_UP=0
for i in $(seq 1 30); do
    if $SSH "curl -fsS --noproxy '*' -o /dev/null -m 5 http://localhost:$PORT/ 2>/dev/null"; then
        PORTAL_UP=1
        break
    fi
    sleep 2
done

echo ""
if [ "$PORTAL_UP" = "1" ]; then
    echo "DONE. Reverted to the pre-fix dashboard."
    echo "  -> http://$HOST:$PORT"
    echo "  -> NOTE: the watchdog is gone, so the portal can die again and will"
    echo "     need 'bash restore_dashboard.sh' (or lab_start.sh start) by hand."
else
    echo "WARNING: the portal did not answer on port $PORT after 60s."
    echo "Check:  $SSH \"docker logs --tail 30 $CONTAINER\""
    echo "        $SSH \"docker exec $CONTAINER bash -lc 'tail -30 /workspace/lab_portal.log'\""
fi
exit 0
