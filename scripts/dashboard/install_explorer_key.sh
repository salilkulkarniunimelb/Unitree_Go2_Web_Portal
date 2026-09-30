#!/bin/bash
# ---------------------------------------------------------------------------
# install_explorer_key.sh - RUNS ON THE SERVER HOST as the selini user.
#
# Sets up the one SSH key the dashboard's Start Exploring button is allowed to
# use, and copies the private half into the portal container.
#
# Two halves, and the asymmetry is the point:
#
#   public  -> ~/.ssh/authorized_keys, pinned with `restrict` AND
#              `command=` pointing at explorer_host.sh. That single line is the
#              whole security boundary: it means this key cannot open a shell,
#              cannot get a pty, cannot forward a port, and cannot run any
#              command other than explorer_host.sh -- whatever the client
#              actually asks for. So the portal can be left unauthenticated on
#              0.0.0.0 without handing out host access.
#
#   private -> into the portal container at /root/.ssh/explorer_key, because
#              that is where the ssh client runs. It never touches the laptop
#              or the repo, so it cannot leak through a git checkout.
#
# Idempotent: re-running reuses the existing key, so the authorized_keys entry
# and the container copy stay in step. Run it again after any
# restore_dashboard.sh, which recreates the container and so empties /root/.ssh.
# ---------------------------------------------------------------------------
set -o pipefail

USER_SEL="selini.samaranayake"
REPO="$HOME/unimelb_project/hardware_code"
HOST_SCRIPT="$REPO/scripts/dashboard/explorer_host.sh"
KEYDIR="$REPO/.explorer"
KEY="$KEYDIR/id_ed25519"
KNOWN="$KEYDIR/known_hosts"
# The PORTAL container (the one running lab_portal.py), not the compute
# container. Override with CONTAINER=... if the portal is ever renamed.
CONTAINER="${CONTAINER:-robot_hivemind_portal}"
PORTAL_KEY=/root/.ssh/explorer_key
PORTAL_KNOWN=/root/.ssh/known_hosts

log() { echo "[install-key] $*"; }
die() { echo "[install-key] ERROR: $*" >&2; exit 1; }

[ -x "$HOST_SCRIPT" ] || die "forced-command script missing: $HOST_SCRIPT"

# --- 1. the key ------------------------------------------------------------
mkdir -p "$KEYDIR" || die "cannot create $KEYDIR"
chmod 700 "$KEYDIR"
if [ -f "$KEY" ]; then
    log "reusing existing key $KEY"
else
    ssh-keygen -t ed25519 -N '' -C 'lab-portal-explorer-button' -f "$KEY" >/dev/null \
        || die "ssh-keygen failed"
    log "generated $KEY"
fi
chmod 600 "$KEY"

# --- 2. pin the host's own key --------------------------------------------
# The portal must verify the server it connects to. Pinned out of band, so a
# swapped host key is a hard error instead of a silent trust-on-first-use.
#
# Note the awk '{print $2, $3}': ssh-keyscan emits
# "<host> <keytype> <base64>", so fields 2 and 3 are the key itself. Taking
# fields 1 and 2 instead yields a line with no key material in it, which fails
# host key verification in a way that looks like an attack rather than a typo.
SCAN="$(ssh-keyscan -t ed25519 127.0.0.1 2>/dev/null | head -1)"
[ -n "$SCAN" ] || die "could not read the local host key with ssh-keyscan"
KEYDATA="$(echo "$SCAN" | awk '{print $2" "$3}')"
case "$KEYDATA" in
    ssh-ed25519\ AAA*) ;;
    *) die "unexpected host key format: $KEYDATA" ;;
esac
# Plain host names, not [host]:port -- the bracket form is only for non-default
# ports, and using it here makes every connection fail verification.
{
    echo "127.0.0.1 $KEYDATA"
    echo "localhost $KEYDATA"
    echo "$(hostname -I 2>/dev/null | awk '{print $1}') $KEYDATA"
} > "$KNOWN"
chmod 600 "$KNOWN"
log "pinned host key into $KNOWN"

# --- 3. authorize it, pinned to the one script -----------------------------
mkdir -p "$HOME/.ssh" && chmod 700 "$HOME/.ssh"
AUTH="$HOME/.ssh/authorized_keys"
touch "$AUTH"
# Drop any previous entry for this key so the forced command always matches the
# current path, then re-add it. Matching on the comment keeps this idempotent.
grep -v 'lab-portal-explorer-button' "$AUTH" > "$AUTH.tmp" 2>/dev/null || true
{
    echo "command=\"$HOST_SCRIPT\",restrict $(cat "$KEY.pub")"
} >> "$AUTH.tmp"
mv "$AUTH.tmp" "$AUTH"
chmod 600 "$AUTH"
log "installed restricted authorized_keys entry"
log "  $(grep 'lab-portal-explorer-button' "$AUTH" | cut -c1-60)..."

# --- 4. put the private half where the portal can use it -------------------
docker inspect "$CONTAINER" >/dev/null 2>&1 || die "container $CONTAINER not found"
docker exec "$CONTAINER" mkdir -p /root/.ssh || die "cannot mkdir in $CONTAINER"
docker cp "$KEY"     "$CONTAINER:$PORTAL_KEY"  || die "docker cp of the key failed"
docker cp "$KNOWN"   "$CONTAINER:$PORTAL_KNOWN" || die "docker cp of known_hosts failed"
docker exec "$CONTAINER" chmod 700 /root/.ssh
docker exec "$CONTAINER" chmod 600 "$PORTAL_KEY" "$PORTAL_KNOWN"
log "installed key into $CONTAINER"

# --- 5. prove the boundary before declaring success ------------------------
# An installation that silently does not restrict anything is worse than one
# that fails, so the canary is mandatory: if the key can still create a file,
# say so loudly rather than reporting OK.
rm -f /tmp/.explorer_canary
out="$(docker exec "$CONTAINER" ssh -T -i "$PORTAL_KEY" \
        -o IdentitiesOnly=yes -o "UserKnownHostsFile=$PORTAL_KNOWN" \
        -o StrictHostKeyChecking=yes -o BatchMode=yes \
        "$USER_SEL@127.0.0.1" "touch /tmp/.explorer_canary" </dev/null 2>&1)"
if [ -e /tmp/.explorer_canary ]; then
    die "SECURITY: the portal key was able to run an arbitrary command. Check the restrict/command= entry in $AUTH"
fi
log "verified: the key cannot run anything but explorer_host.sh"

status_out="$(docker exec "$CONTAINER" ssh -T -i "$PORTAL_KEY" \
        -o IdentitiesOnly=yes -o "UserKnownHostsFile=$PORTAL_KNOWN" \
        -o StrictHostKeyChecking=yes -o BatchMode=yes \
        "$USER_SEL@127.0.0.1" status </dev/null 2>&1 | tail -1)"
[ -n "$status_out" ] || die "the portal cannot reach the forced command"
log "portal -> host check: $status_out"
log "done"
