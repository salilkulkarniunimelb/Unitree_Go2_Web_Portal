#!/bin/bash
# lab_start.sh - Run INSIDE the robot_hivemind container.
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
PORT=7860
# Match the portal process (lab_portal.py), NOT this script's own command line.
PROC="python3 .*lab_portal.py"

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
    curl -s -o /dev/null http://localhost:$PORT/ 2>/dev/null
}

start() {
    kill_pids
    echo "Waiting for port $PORT to be free..."
    sleep 2
    if ! wait_port_free; then
        echo "ERROR: port $PORT still in use. Cannot start."
        return 1
    fi
    source /opt/ros/humble/setup.bash
    setsid nohup python3 lab_portal.py > "$LOGFILE" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    echo "Started pid $(cat "$PIDFILE"). Waiting for it to serve..."
    for i in $(seq 1 30); do
        if is_up; then
            # confirm the process is actually still alive (not a stale listener)
            pid=$(cat "$PIDFILE")
            if kill -0 "$pid" 2>/dev/null; then
                echo "UP -> http://localhost:$PORT"
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
    if is_up; then
        pid=$(cat "$PIDFILE" 2>/dev/null)
        echo "UP -> http://localhost:$PORT (pid ${pid:-?})"
        tail -3 "$LOGFILE"
    else
        echo "Not serving on port $PORT."
    fi
}

case "${1:-status}" in
    start)     start ;;
    stop)      stop ;;
    restart)   stop; start ;;
    status)    status ;;
    *) echo "usage: $0 {start|stop|restart|status}" ;;
esac
