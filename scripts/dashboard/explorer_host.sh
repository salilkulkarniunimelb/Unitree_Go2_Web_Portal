#!/bin/bash
# ---------------------------------------------------------------------------
# explorer_host.sh - RUNS ON THE SERVER HOST. Not run by hand.
#
# This is the forced command behind the dashboard's "Start Exploring" button.
# The portal (which lives in the robot_hivemind container) opens an SSH
# connection whose key is pinned in authorized_keys to this exact script via
#   command="/home/selini.samaranayake/.config/lab-portal-explorer/explorer_host.sh"
# so that key can never run anything else, no matter what the portal asks for.
# sshd passes the client's requested command in $SSH_ORIGINAL_COMMAND.
#
# The explorer cannot run in the portal's own container: robot_hivemind has no
# volume mounts and the unimelb-humble:dashboard image ships no ros2_ws/src at
# all. The code lives on this host, bind-mounted into robot_hivemind_luna,
# which is where the build and the launch actually happen. That container is
# the one with ROS humble, the 11 workspace packages and ROS_DOMAIN_ID=70.
#
# Accepted $SSH_ORIGINAL_COMMAND values (anything else is refused):
#   start <luna|astro>   zenoh bridge -> ensure container -> build -> launch
#   stop                 kill the running launch and its node tree
#   status               one-line report of what is running
#   log                  tail of the detached launch's output
#
# `start` streams the bridge and build phases over stdout -- that is the part
# worth watching, and it is bounded -- then detaches ros2 launch and returns.
# The explorer therefore survives a portal restart, a network blip or a closed
# browser: it is owned by the container, not by the SSH session. The log
# action is how the dashboard keeps showing what the launch is doing afterwards.
# ---------------------------------------------------------------------------
set -o pipefail

CONTAINER=robot_hivemind_luna
# Host-local state, deliberately outside every git repository. hardware_code
# here is a pull target: nobody edits it on this server, so writing our log,
# pidfile and meta into its tree would dirty a repo we do not own and get wiped
# by the next `git pull`/`git clean`. $HOME/.config is ours alone.
EXPLORER_HOME="${EXPLORER_HOME:-$HOME/.config/lab-portal-explorer}"
STATE_DIR="$EXPLORER_HOME"
# Read-only use of the pulled repo: the zenoh bridge launcher is their script,
# so run theirs rather than reimplementing the container invocation.
BRIDGE="$HOME/unimelb_project/hardware_code/scripts/docker/start_zenoh_bridge_container_server.sh"
WS="/workspace/hardware_code/ros2_ws"
# Written INSIDE the container: the pid of the ros2 launch session leader, so
# stop can signal the whole node tree rather than just the docker exec client.
LAUNCH_PIDFILE=/workspace/.explorer_launch.pid
# ros2 launch output is written here rather than streamed over SSH, so the
# explorer keeps running if the portal or the SSH link goes away.
LAUNCH_LOGFILE=/workspace/explorer_launch.log

PHASE="[explorer] PHASE="
log()  { echo "[explorer] $*"; }
fail() { echo "[explorer] ERROR: $*" >&2; exit 1; }

usage() {
    cat >&2 <<EOF
explorer_host.sh: unsupported request.
  start <luna|astro> | stop | status
EOF
    exit 2
}

# --- parse the request strictly -------------------------------------------
# With a forced command, sshd does NOT pass the client's command line as
# arguments -- it runs this script with an empty argv and puts the requested
# command in $SSH_ORIGINAL_COMMAND. So that is what must be parsed here;
# reading "$1"/"$2" would silently see an empty request and always fall
# through to usage. The "$*" fallback only exists so the script can also be
# exercised by hand.
#
# This input is attacker-influenced (it is whatever the portal asked for), so
# it is matched against a fixed grammar below and never interpolated into a
# shell string. Globbing is disabled so word splitting cannot expand a "*".
REQUEST="${SSH_ORIGINAL_COMMAND:-$*}"
set -f
# shellcheck disable=SC2086 # deliberate word splitting of $REQUEST
set -- $REQUEST
set +f
ACTION="${1:-}"
ROBOT="${2:-}"

case "$ACTION" in
    start)
        case "$ROBOT" in
            luna|astro) ;;
            *) fail "start needs a robot of exactly 'luna' or 'astro' (got '$ROBOT')" ;;
        esac
        ;;
    stop|status|log) ;;
    *) usage ;;
esac

mkdir -p "$STATE_DIR" || fail "cannot create $STATE_DIR"

# --- helpers ---------------------------------------------------------------
container_running() {
    [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]
}

# True when a ros2 launch from a previous run is still alive. Checked INSIDE
# the container: the host only ever saw the docker exec client, which says
# nothing about the nodes it started.
launch_pid() {
    container_running || return 1
    docker exec "$CONTAINER" cat "$LAUNCH_PIDFILE" 2>/dev/null
}

launch_alive() {
    local pid
    pid="$(launch_pid 2>/dev/null)" || return 1
    [ -n "$pid" ] || return 1
    docker exec "$CONTAINER" kill -0 "$pid" 2>/dev/null
}

start_bridge() {
    [ -x "$BRIDGE" ] || { log "zenoh bridge script not found at $BRIDGE - skipping"; return 0; }
    echo "$PHASE"bridge
    log "starting zenoh bridge (host)"
    # The script is chatty and occasionally returns non-zero while still
    # leaving a healthy bridge up, so its status is reported, not enforced.
    if bash "$BRIDGE"; then
        log "zenoh bridge reported OK"
    else
        log "WARNING: the zenoh bridge script exited non-zero; continuing anyway"
    fi
}

do_start() {
    if launch_alive; then
        log "an explorer is already running in $CONTAINER - stop it first"
        return 1
    fi

    # Recorded before the (long) build so `status` can name the robot even
    # while the build is still in progress.
    echo "$ROBOT" > "$STATE_DIR/last_robot" 2>/dev/null || true

    start_bridge

    if ! container_running; then
        log "container $CONTAINER is not running - starting it"
        docker start "$CONTAINER" >/dev/null || fail "could not start $CONTAINER"
        # Give ROS entrypoint + host networking a moment before exec'ing.
        sleep 5
    fi
    container_running || fail "$CONTAINER is not running after start"

    # Everything from here runs inside the container, streamed straight back.
    # `setsid` puts ros2 launch in its own session so stop can signal the whole
    # process group (ros2 launch spawns a dozen nodes; killing only the parent
    # would orphan the rest, leaving them publishing /map).
    docker exec "$CONTAINER" bash -c '
        cd '"$WS"' || exit 1
        source /opt/ros/humble/setup.bash
        [ -f install/setup.bash ] && source install/setup.bash

        echo "'"$PHASE"'env ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
        echo "'"$PHASE"'build"
        colcon build --symlink-install \
          --packages-select go2_hardware_autonomy orchestrator \
          --cmake-args -DBUILD_TESTING=OFF
        build_rc=$?
        if [ $build_rc -ne 0 ]; then
            echo "'"$PHASE"'build-failed rc=$build_rc"
            exit $build_rc
        fi

        echo "$PHASE"launch
        # Detached on purpose. setsid puts ros2 launch in its own session so
        # stop can signal the whole process group (it spawns a dozen nodes;
        # killing only the parent would orphan the rest, leaving them
        # publishing /map). Output goes to a file, not down this SSH pipe, so
        # that closing the browser or restarting the portal does not kill the
        # explorer -- `log` is how the dashboard reads it back.
        setsid ros2 launch go2_hardware_autonomy \
            go2_explorer_splitcomp_server_launch.py \
            robot_namespace:='"$ROBOT"' \
            > '"$LAUNCH_LOGFILE"' 2>&1 < /dev/null &
        echo $! > '"$LAUNCH_PIDFILE"'
        # A bad launch file exits within a second or so; catch that here so the
        # caller gets a real error instead of a dashboard claiming success.
        sleep 3
        if kill -0 "$(cat '"$LAUNCH_PIDFILE"')" 2>/dev/null; then
            echo "'"$PHASE"'launched"
        else
            echo "'"$PHASE"'launch-failed"
            tail -n 20 '"$LAUNCH_LOGFILE"' 2>/dev/null
            exit 1
        fi
    '
}

do_stop() {
    local pid
    pid="$(launch_pid 2>/dev/null || true)"
    if [ -z "$pid" ]; then
        log "no explorer is running in $CONTAINER"
        rm -f "$STATE_DIR/last_robot" 2>/dev/null || true
        return 0
    fi

    log "stopping explorer (pid $pid) in $CONTAINER"
    # Negative pid => the whole process group, i.e. ros2 launch and every node
    # it started. Escalate if they do not go quietly.
    docker exec "$CONTAINER" bash -c "kill -TERM -$pid 2>/dev/null" || true
    for _ in $(seq 1 20); do
        docker exec "$CONTAINER" kill -0 "$pid" 2>/dev/null || break
        sleep 1
    done
    if docker exec "$CONTAINER" kill -0 "$pid" 2>/dev/null; then
        log "still alive after SIGTERM - sending SIGKILL"
        docker exec "$CONTAINER" bash -c "kill -KILL -$pid 2>/dev/null" || true
    fi
    docker exec "$CONTAINER" rm -f "$LAUNCH_PIDFILE" 2>/dev/null || true
    rm -f "$STATE_DIR/last_robot" 2>/dev/null || true
    log "stopped"
}

do_status() {
    if launch_alive; then
        echo "running pid=$(launch_pid) robot=$(cat "$STATE_DIR/last_robot" 2>/dev/null || echo unknown)"
    else
        echo "stopped"
    fi
}

do_log() {
    # The launch writes here rather than to stdout, so this is the only way to
    # see what it is doing once the start call has returned.
    container_running || { echo "(container $CONTAINER is not running)"; return 0; }
    docker exec "$CONTAINER" tail -c 8000 "$LAUNCH_LOGFILE" 2>/dev/null \
        || echo "(no launch log yet)"
}

case "$ACTION" in
    start)  do_start ;;
    stop)   do_stop ;;
    status) do_status ;;
    log)    do_log ;;
esac
