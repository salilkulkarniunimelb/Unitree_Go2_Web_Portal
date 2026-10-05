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
# The explorer cannot run in the portal's own container: robot_hivemind_portal
# has no volume mounts and the unimelb-humble:dashboard image ships no
# ros2_ws/src at all. The code lives on this host, bind-mounted into
# robot_hivemind, which is where the build and the launch actually happen. That
# container is the one with ROS humble, the 11 workspace packages and
# ROS_DOMAIN_ID=70.
#
# robot_hivemind is shared with hand-run compute (colcon build, ros2 run), which
# is the point: it already holds the workspace this needs, so a separate
# robot_hivemind_luna only added a second container to keep in sync -- and a
# name to drift out of date with. Both roles are safe in one container because
# the portal keeps its own container (so `docker restart` on a dashboard deploy
# still cannot disturb a live launch) and because
# scripts/compute/setup_compute_container.sh refuses to recreate this container
# while anything beyond its idle shell is running, which is exactly the
# protection a live launch needs.
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

CONTAINER=robot_hivemind
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
# Separate pid/log for the patrol server (Goal Setting). Deliberately NOT the
# same file as the explorer's: stopping exploration must never take the patrol
# server down, and sharing a pidfile would make the two kill each other.
PATROL_PIDFILE=/workspace/.patrol_launch.pid
PATROL_LOGFILE=/workspace/patrol_launch.log
# The completed map the robots localise on after exploring.
PATROL_MAP=/workspace/hardware_code/maps/amcl_completed_map.yaml

PHASE="[explorer] PHASE="
log()  { echo "[explorer] $*"; }
fail() { echo "[explorer] ERROR: $*" >&2; exit 1; }

usage() {
    cat >&2 <<EOF
explorer_host.sh: unsupported request.
  start <luna|astro> | stop | status
  patrol-start | patrol-stop | patrol-status | patrol-log
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
    # Localisation server (Goal Setting). No robot argument: one server-side
    # map_server plus relays republish the completed map to BOTH robots'
    # namespaces, so there is nothing to select here. The patrol UI is NOT part
    # of this launch -- see do_patrol_start for why.
    patrol-start|patrol-stop|patrol-status|patrol-log) ;;
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

# Same pair of helpers for the patrol server, against its own pidfile.
patrol_launch_pid() {
    container_running || return 1
    docker exec "$CONTAINER" cat "$PATROL_PIDFILE" 2>/dev/null
}

patrol_alive() {
    local pid
    pid="$(patrol_launch_pid 2>/dev/null)" || return 1
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

do_patrol_start() {
    # Idempotent-ish: refuse rather than stack a second launch, because two
    # would fight over /map and the AMCL filter.
    if patrol_alive; then
        log "the patrol server is already running in $CONTAINER"
        return 1
    fi

    # The zenoh bridge and the container are deliberately left alone: the
    # operator presses this after Stop Exploring, and neither of those is
    # stopped by Stop Exploring. Only the launch itself is (re)started here.
    if ! container_running; then
        log "container $CONTAINER is not running - starting it"
        docker start "$CONTAINER" >/dev/null || fail "could not start $CONTAINER"
        sleep 5
    fi
    container_running || fail "$CONTAINER is not running after start"

    docker exec "$CONTAINER" bash -c '
        cd '"$WS"' || exit 1
        source /opt/ros/humble/setup.bash
        [ -f install/setup.bash ] && source install/setup.bash

        if [ ! -f '"$PATROL_MAP"' ]; then
            echo "PATROL_MAP_MISSING"
            exit 2
        fi

        # Only the LOCALISATION server, not orchestrator/hivemind_patrol_server.
        #
        # hivemind_patrol_server.launch.py also includes edge_patrol_ui, whose
        # patrol_testbench process imports scipy.ndimage. In robot_hivemind that
        # import is broken (numpy 2.2.6 against a scipy 1.8.0 built for numpy
        # <1.25: "ImportError: numpy.core.multiarray failed to import"), so that
        # node died within a second of every launch and took the whole
        # launch down with it -- map_server and the four map relays included,
        # leaving /luna/lidar_slam_2d_map and /astro/lidar_slam_2d_map with no
        # publisher at all. That is why the map never appeared.
        #
        # go2_hivemind_localization_server_launch.py is the localisation half of
        # that same include tree, and it is scipy-free. It runs map_server plus
        # occupancy_grid_relay_node for BOTH robots -- /luna/map, /astro/map,
        # /luna/lidar_slam_2d_map and /astro/lidar_slam_2d_map -- which is
        # exactly the pair of topics the two dashboard map panels subscribe to
        # (lab_portal.py LUNA_TOPICS/ASTRO_TOPICS). One launch, both robots.
        #
        # The arg is map_file, not map_yaml: hivemind_patrol_server renamed it
        # on the way in. explorer_robot is deliberately not passed -- it only
        # picks a default filename, and map_file overrides that anyway.
        #
        # Foxglove is left off because the container already has bridges bound
        # to 8765; a fifth one would fail to bind and die.
        #
        # Detached with setsid for the same reason as the explorer: stop has to
        # be able to signal the whole process group.
        setsid ros2 launch go2_hardware_autonomy \
            go2_hivemind_localization_server_launch.py \
            map_file:='"$PATROL_MAP"' \
            launch_foxglove_bridge:=false \
            > '"$PATROL_LOGFILE"' 2>&1 < /dev/null &
        echo $! > '"$PATROL_PIDFILE"'
        # A bad launch file exits within a second or so; surface that as a real
        # error rather than the dashboard claiming success.
        sleep 5
        if kill -0 "$(cat '"$PATROL_PIDFILE"')" 2>/dev/null; then
            echo "PATROL_LAUNCHED"
        else
            echo "PATROL_LAUNCH_FAILED"
            tail -n 20 '"$PATROL_LOGFILE"' 2>/dev/null
            exit 1
        fi
    '
}

do_patrol_stop() {
    local pid
    pid="$(patrol_launch_pid 2>/dev/null || true)"
    if [ -z "$pid" ]; then
        log "no patrol server is running in $CONTAINER"
        return 0
    fi

    log "stopping patrol server (pid $pid) in $CONTAINER"
    docker exec "$CONTAINER" bash -c "kill -TERM -$pid 2>/dev/null" || true
    for _ in $(seq 1 20); do
        docker exec "$CONTAINER" kill -0 "$pid" 2>/dev/null || break
        sleep 1
    done
    if docker exec "$CONTAINER" kill -0 "$pid" 2>/dev/null; then
        log "still alive after SIGTERM - sending SIGKILL"
        docker exec "$CONTAINER" bash -c "kill -KILL -$pid 2>/dev/null" || true
    fi
    docker exec "$CONTAINER" rm -f "$PATROL_PIDFILE" 2>/dev/null || true
    log "patrol server stopped"
}

do_patrol_status() {
    if patrol_alive; then
        echo "running pid=$(patrol_launch_pid)"
    else
        echo "stopped"
    fi
}

do_patrol_log() {
    container_running || { echo "(container $CONTAINER is not running)"; return 0; }
    docker exec "$CONTAINER" tail -c 8000 "$PATROL_LOGFILE" 2>/dev/null \
        || echo "(no patrol log yet)"
}

case "$ACTION" in
    start)  do_start ;;
    stop)   do_stop ;;
    status) do_status ;;
    log)    do_log ;;
    patrol-start)  do_patrol_start ;;
    patrol-stop)   do_patrol_stop ;;
    patrol-status) do_patrol_status ;;
    patrol-log)    do_patrol_log ;;
esac
