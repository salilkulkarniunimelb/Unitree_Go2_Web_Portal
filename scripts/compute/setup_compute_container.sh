#!/bin/bash
# ---------------------------------------------------------------------------
# setup_compute_container.sh - creates/manages the `robot_hivemind` COMPUTE
# container. Runs on the server host.
#
# This is deliberately a different container from the dashboard. They used to
# share the name "robot_hivemind", which broke in a quiet way: the dashboard's
# restore script only recreates a container when `docker inspect` fails, so it
# happily copied lab_portal.py into a compute container whose entrypoint was a
# bare `bash`, restarted it, and then reported "the portal never answered on
# port 7860" with nothing pointing at the real cause.
#
#   robot_hivemind        compute  (this script)   hardware_code bind-mounted
#   robot_hivemind_portal dashboard                serves 7860
#   robot_hivemind_luna   compute                 the container the explorer
#                                                 builds and launches in
#
# What "smoothly" means here, versus the ad-hoc container this replaces:
#   * Restart=unless-stopped, so it comes back after a reboot instead of
#     needing a manual `docker start` (the ad-hoc one had Restart=no).
#   * The ROS environment and the built workspace are sourced by the default
#     command, so `docker exec robot_hivemind ros2 ...` and `colcon build`
#     work immediately instead of failing with "ros2: command not found".
#   * It holds `sleep infinity` rather than an interactive shell, so the
#     container stays up whether or not anyone is attached.
#
# Idempotent: does nothing if the container already exists and is healthy.
# Refuses to recreate a container that is actually running something, so it
# cannot destroy an in-flight build or launch. Use --force to override.
# ---------------------------------------------------------------------------
set -o pipefail

CONTAINER="${CONTAINER:-robot_hivemind}"
IMAGE="${IMAGE:-unimelb-humble:base}"
REPO="${REPO:-$HOME/unimelb_project/hardware_code}"
WS=/workspace/hardware_code/ros2_ws
FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

log() { echo "[compute] $*"; }
die() { echo "[compute] ERROR: $*" >&2; exit 1; }

command -v docker >/dev/null || die "docker not available"
[ -d "$REPO" ] || die "host repo not found: $REPO"
docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image not found: $IMAGE"

if docker inspect "$CONTAINER" >/dev/null 2>&1; then
    RUNNING=$(docker inspect -f '{{.State.Running}}' "$CONTAINER")
    MOUNTED=$(docker inspect -f '{{range .Mounts}}{{.Destination}} {{end}}' "$CONTAINER")

    # Is it doing real work? The default command is `sleep infinity` plus a
    # sourced setup, so anything else is a live build, launch or shell.
    if [ "$RUNNING" = "true" ]; then
        BUSY=$(docker exec "$CONTAINER" bash -lc \
            'ps -eo cmd --no-headers | grep -vE "^sleep infinity|^bash$|ps -eo|grep -vE|/ros_entrypoint" | wc -l' 2>/dev/null || echo 0)
        if [ "${BUSY:-0}" -gt 0 ] && [ "$FORCE" -eq 0 ]; then
            die "$CONTAINER is running $BUSY process(es) beyond its idle shell.
       Refusing to recreate it and destroy that work.
       Check what they are:
         docker exec $CONTAINER ps -eo pid,cmd --no-headers
       Then re-run with --force if you really mean to replace it."
        fi
        if [ "$FORCE" -eq 0 ]; then
            log "$CONTAINER already exists and is idle - nothing to do."
            log "  (mounted: ${MOUNTED:-none})"
            exit 0
        fi
        log "--force given: recreating $CONTAINER."
    else
        log "$CONTAINER exists but is stopped; recreating it cleanly."
    fi
    docker rm -f "$CONTAINER" >/dev/null || die "could not remove the old $CONTAINER"
fi

log "creating $CONTAINER from $IMAGE (host networking, auto-restart)"
# The default command sources ROS and the built workspace, then holds the
# container open. `exec` matters: it replaces the shell so bash is not left as
# an extra process reaping nothing.
docker run -d \
    --name "$CONTAINER" \
    --network host \
    --restart unless-stopped \
    -v "$REPO:/workspace/hardware_code" \
    -w "$WS" \
    "$IMAGE" \
    bash -c "source /opt/ros/humble/setup.bash && \
             [ -f install/setup.bash ] && source install/setup.bash; \
             echo '[compute] ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-<unset>}'; \
             exec sleep infinity" >/dev/null || die "docker run failed"

sleep 2
docker exec "$CONTAINER" bash -lc \
    'source /opt/ros/humble/setup.bash && [ -f install/setup.bash ] && source install/setup.bash && ros2 pkg prefix go2_hardware_autonomy' \
    >/dev/null 2>&1 \
    || log "WARNING: ros2 could not resolve the workspace inside $CONTAINER yet (a build may be needed)."

log "ready:"
docker inspect "$CONTAINER" --format '  running={{.State.Running}} restart={{.HostConfig.RestartPolicy.Name}} mounts={{range .Mounts}}{{.Destination}} {{end}}'
log "  use it with:  docker exec -it $CONTAINER bash"
