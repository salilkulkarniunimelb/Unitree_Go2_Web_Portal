#!/bin/bash
# boot.sh - main command for the robot_hivemind container.
# Auto-starts the dashboard portal whenever the container starts (server boot /
# docker restart), then keeps the container AND the portal alive. Do not edit.
#
# Why the supervisor loop instead of `sleep infinity`: the portal is a plain
# python process and it dies on its own fairly often (camera consumer dropping,
# an unhandled exception, a stray OOM-kill of the child). With a bare
# `sleep infinity` the container stayed "Up" while nothing served port 7860, so
# the dashboard looked dead and the only way back was restore_dashboard.sh by
# hand. The loop below notices the dead child and restarts it on its own.
PORT_LABEL=7860
BACKOFF_MIN=5
BACKOFF_MAX=60
backoff=$BACKOFF_MIN

# Only auto-start on boot if the operator has not deliberately stopped the
# portal. A recorded "stopped" survives a container restart, so `stop` really
# means stop; `lab_start.sh start` clears it and the watchdog takes over.
# When the state file is absent (fresh container) it defaults to running, so a
# new container comes up serving without anyone touching it.
if bash /workspace/lab_start.sh supervise; then
    bash /workspace/lab_start.sh start || true
else
    echo "[boot.sh] portal was stopped by the operator - not auto-starting."
fi

while true; do
    # Respect the operator's last intent. `lab_start.sh stop` records "stopped"
    # so a deliberate stop is not immediately undone by this loop; `start`
    # records "run" and the watchdog takes over again from there.
    if ! bash /workspace/lab_start.sh supervise; then
        backoff=$BACKOFF_MIN
        sleep 15
        continue
    fi

    if bash /workspace/lab_start.sh is-alive; then
        backoff=$BACKOFF_MIN
        sleep 15
        continue
    fi

    echo "[boot.sh] portal is not serving on port $PORT_LABEL - restarting (backoff ${backoff}s)"
    bash /workspace/lab_start.sh start || true

    # Escalating backoff: if the portal cannot stay up (e.g. a syntax error in
    # lab_portal.py, or missing deps) do not spin-restart it every 5s forever.
    sleep "$backoff"
    backoff=$((backoff * 2))
    [ "$backoff" -gt "$BACKOFF_MAX" ] && backoff=$BACKOFF_MAX
done