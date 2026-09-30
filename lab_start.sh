#!/bin/bash
# lab_start.sh - Run INSIDE the robot_hivemind_portal container.
# Start / stop / restart / status for the QOD lab portal (lab_portal.py).
#
#   bash lab_start.sh start
#   bash lab_start.sh status
#   bash lab_start.sh stop
#   bash lab_start.sh restart
# ---------------------------------------------------------------------------
cd /workspace || exit 1
PIDFILE=/workspace/lab_portal.pid
LOGFILE=/workspace/lab_portal.log
# Records the operator's last intent. The supervisor in boot.sh restarts the
# portal ONLY while this file says "run"; `stop` rewrites it to "stopped" so a
# deliberate stop is not immediately undone by the watchdog.
STATEFILE=/workspace/lab_portal.state
LOCKFILE=/workspace/lab_portal.lock
PORT=7860
# Match the portal process (lab_portal.py), NOT this script's own command line.
PROC="python3 .*lab_portal.py"
# Server/LAN IP (container uses host networking, so this is the server's IP).
LAN_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

kill_pids() {
    # Kill any live portal processes found by pattern (ignore the grep itself).
    pids=$(ps -eo pid,cmd | grep -iE "lab_portal.py" | grep -v grep | awk '{print $1}')
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
    # also remove stale pidfile entry if that pid is gone
    [ -f "$PIDFILE" ] && { old=$(cat "$PIDFILE"); kill -0 "$old" 2>/dev/null || rm -f "$PIDFILE"; }
}

wait_port_free() {
    for i in $(seq 1 30); do
        if ! (ss -tln 2>/dev/null | grep -q ":$PORT "); then
            return 0
        fi
        sleep 1
    done
    return 1
}

is_up() {
    curl -s -o /dev/null -m 5 -w '%{http_code}' http://localhost:$PORT/ 2>/dev/null | grep -qE '^(200|302|303|307|401|403)$'
}

# A listener on :7860 is not proof the portal is alive - a crashed process can
# leave a stale socket, and a stale PID file can name a long-dead pid. Require
# BOTH a live lab_portal.py process AND a real HTTP answer.
is_alive() {
    pids=$(ps -eo pid,cmd | grep -E "python3 .*lab_portal\.py" | grep -v grep | awk '{print $1}')
    [ -n "$pids" ] || return 1
    is_up
}

# The operator's intent, as recorded by the last start/stop. Defaults to "run"
# when the file is missing, so a fresh container comes up serving by default
# (and a boot.sh that predates this file keeps working).
want_running() {
    [ "$(cat "$STATEFILE" 2>/dev/null)" = "stopped" ] && return 1
    return 0
}

start() {
    # Record intent BEFORE launching, so a crash during startup still leaves the
    # watchdog willing to bring the portal back.
    echo run > "$STATEFILE"

    # Serialise every start/stop. Without this, a manual `lab_start.sh start`
    # and the boot.sh watchdog can both decide the portal is down at the same
    # moment and each spawns a python3 lab_portal.py - the second then loses the
    # race for :7860 and dies, leaving a confusing extra process and a burst of
    # wasted work. flock is released automatically when this script exits.
    # -w bounds the wait so a stale lock degrades into an error instead of
    # hanging the caller (and the watchdog) indefinitely.
    exec 9>"$LOCKFILE"
    if ! flock -w 120 9; then
        echo "ERROR: timed out waiting for the portal lock ($LOCKFILE)."
        echo "       If no lab_start.sh is running, remove the stale lock file."
        return 1
    fi

    # Re-check under the lock: the watchdog may have just restarted the portal
    # while we waited for it, in which case starting again would be wrong.
    if is_alive; then
        echo "Already UP -> http://localhost:$PORT"
        return 0
    fi

    kill_pids
    echo "Waiting for port $PORT to be free..."
    sleep 2
    if ! wait_port_free; then
        echo "ERROR: port $PORT still in use. Cannot start."
        return 1
    fi
    source /opt/ros/humble/setup.bash
    # Also source the hardware ROS2 workspace (provides unitree_go msg types)
    # used for the H.264 front-camera stream. Optional if not present.
    if [ -f /workspace/hardware_code/ros2_ws/install/setup.bash ]; then
        source /workspace/hardware_code/ros2_ws/install/setup.bash
    fi
    # Guard: the QOD WebRTC camera consumer (aiortc) and uvicorn both need
    # websockets >= 10. Debian ships websockets 9.1 in /usr/lib/python3/
    # dist-packages; if that one wins on import (no websockets.server.
    # ServerProtocol, breaks the portal + camera), force-reinstall the pip
    # build into /usr/local so it wins on sys.path.
    if ! python3 -c \
        "import websockets;assert int(websockets.__version__.split('.')[0])>=10" 2>/dev/null; then
        echo "  -> fixing websockets (found \
$(python3 -c 'import websockets;print(websockets.__version__)' 2>/dev/null || echo missing))..."
        python3 -m pip install --no-cache-dir --force-reinstall --no-deps \
            "websockets>=10,<17"
    fi
    # 9>&- is essential: the portal is a long-lived daemon, so if it inherited
    # fd 9 it would hold the flock open for its whole life and every future
    # start/stop would block forever waiting on the lock.
    setsid nohup python3 lab_portal.py 9>&- > "$LOGFILE" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    echo "Started pid $(cat "$PIDFILE"). Waiting for it to serve..."
    for i in $(seq 1 30); do
        if is_up; then
            # confirm the process is actually still alive (not a stale listener)
            pid=$(cat "$PIDFILE")
            if kill -0 "$pid" 2>/dev/null; then
                echo "UP -> http://localhost:$PORT (this server: http://${LAN_IP:-10.4.48.11}:$PORT)"
                return 0
            else
                echo "Port responded but pid $pid is gone. Check $LOGFILE"
                return 1
            fi
        fi
        sleep 1
    done
    echo "Not ready. Check: $LOGFILE"
    return 1
}

stop() {
    # Record intent FIRST: this is what stops the boot.sh watchdog from
    # immediately restarting what we are about to kill.
    echo stopped > "$STATEFILE"
    # Same lock as start(), so a stop can never interleave with a start.
    exec 9>"$LOCKFILE"
    flock -w 120 9 || echo "WARN: timed out waiting for the portal lock; stopping anyway."
    kill_pids
    # wait for port to release
    for i in $(seq 1 20); do
        if wait_port_free; then
            echo "Stopped (port $PORT free)."
            return 0
        fi
        sleep 1
    done
    echo "WARN: could not confirm port $PORT freed."
}

status() {
    if ! want_running; then
        echo "STOPPED by operator (lab_start.sh start to bring it back)."
        return 0
    fi
    if is_alive; then
        pid=$(cat "$PIDFILE" 2>/dev/null)
        echo "UP -> http://localhost:$PORT (this server: http://${LAN_IP:-10.4.48.11}:$PORT) (pid ${pid:-?})"
        tail -3 "$LOGFILE"
    elif is_up; then
        echo "DEGRADED: something is answering on $PORT but it is not a healthy lab_portal.py. Check: $LOGFILE"
        return 1
    else
        echo "Not serving on port $PORT (supervisor will restart it)."
        return 1
    fi
}

case "${1:-status}" in
    start)     start ;;
    stop)      stop ;;
    restart)   stop; start ;;
    status)    status ;;
    is-alive)  is_alive && exit 0 || exit 1 ;;
    supervise) # true iff the operator wants the portal up (watchdog uses this)
                want_running && exit 0 || exit 1 ;;
    *) echo "usage: $0 {start|stop|restart|status|is-alive|supervise}"; exit 2 ;;
esac
