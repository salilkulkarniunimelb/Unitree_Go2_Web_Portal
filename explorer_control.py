"""Dashboard-side controller for the split-compute explorer.

The dashboard (lab_portal.py) runs INSIDE the robot_hivemind container, which
cannot start the explorer itself: that container has no volume mounts, no
docker socket, and the unimelb-humble:dashboard image ships no ros2_ws/src at
all. The code and the build toolchain live on the server host, bind-mounted
into the robot_hivemind_luna container, and that is where a launch has to
happen.

So this module does not build or launch anything itself. It drives the host
over a deliberately restricted SSH connection:

    portal (robot_hivemind)  --ssh-->  host  --docker exec-->  robot_hivemind_luna
      explorer_control.py                explorer_host.sh          colcon / ros2 launch

The key used for that connection is pinned in the host's authorized_keys with
`restrict` and `command=.../explorer_host.sh`, so it can only ever run that one
script, no matter what this file asks for. That is what makes an
unauthenticated dashboard button safe to expose on 0.0.0.0. The host script
matches the request against a fixed grammar (`start luna|astro`, `stop`,
`status`, `log`) and never interpolates it into a shell string.

Lifetimes are split deliberately. The host streams the Zenoh bridge and
`colcon build` back over the connection -- bounded work, and the part an
operator actually wants to watch -- then detaches `ros2 launch` and returns.
The explorer is therefore owned by the container and survives the browser
closing, the portal restarting or the network dropping; `log` is how the
dashboard keeps showing it. Nothing here may assume a long-lived child, and
liveness has to be answered by the host (`status`), not by looking at a local
pid, because after `start` returns there is no local process at all.

Everything ROS_DOMAIN_ID related is a non-issue by construction: nothing
crosses domains, the explorer is launched by the host in the same container
that already runs on ROS_DOMAIN_ID=70, and the Zenoh namespaces (/luna/,
/astro/) give the per-robot separation.
"""

import os
import re
import subprocess
import threading
import time

# ----------------------- how to reach the host -------------------------------
# 127.0.0.1, not 10.4.48.11: the portal container shares the host's network
# namespace (--network host), so the host's sshd is on loopback and this also
# works if the host's LAN address ever changes.
SSH_TARGET = "selini.samaranayake@127.0.0.1"

# Private half of the key installed by scripts/dashboard/install_explorer_key.sh.
# Only the portal user can read it, and the host side is pinned to one script.
SSH_KEY = "/root/.ssh/explorer_key"
# Host key pinned out of band, so a rewritten or intercepted host cannot be
# silently trusted mid-session and no interactive prompt can wedge the button.
SSH_KNOWN_HOSTS = "/root/.ssh/known_hosts"

# Hardened, and non-interactive in every direction: if the key or the host key
# is wrong the button must fail fast with an error rather than block on a
# prompt that nothing in a Gradio callback can answer.
SSH_OPTS = (
    "-i", SSH_KEY,
    "-o", "IdentitiesOnly=yes",
    "-o", f"UserKnownHostsFile={SSH_KNOWN_HOSTS}",
    "-o", "StrictHostKeyChecking=yes",
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=10",
    # Drop the connection if the host stops answering, so a long build cannot
    # sit in a half-open state forever with no way to tell from the dashboard.
    "-o", "ServerAliveInterval=15",
    "-o", "ServerAliveCountMax=4",
)

# Dropdown label -> Zenoh/ROS namespace.
# The launch file hard-validates robot_namespace against exactly
# {"luna", "astro"}, and the host wrapper refuses anything else, so a label
# outside this map never reaches a command line.
ROBOT_NAMESPACES = {"Luna": "luna", "Astro": "astro"}

# The two packages the explorer needs, named only in user-facing text: the
# build itself happens on the host, in explorer_host.sh.
BUILD_PACKAGES = ("go2_hardware_autonomy", "orchestrator")

# Where the portal keeps its own bookkeeping.
PID_FILE = "/workspace/explorer.pid"
LOG_FILE = "/workspace/explorer.log"
META_FILE = "/workspace/explorer.meta"

# Bytes of log shown in the dashboard. The explorer is extremely chatty (lidar
# slam + graph opt at info), so show only the tail.
LOG_TAIL_BYTES = 6000
# How much of the host's launch log to pull back per tick.
REMOTE_LOG_BYTES = 6000

# Phase markers emitted by explorer_host.sh. The dashboard reads the last one
# back to tell "still building" apart from "launching/running", which is the
# only way to explain why nothing is moving yet: a cold colcon build of these
# two packages takes minutes.
PHASE_PREFIX = "[explorer] PHASE="

# Bounds for the short host calls. status/log return in well under a second;
# stop has to wait out SIGTERM plus the host's SIGKILL escalation.
CALL_TIMEOUT_S = 25.0
STOP_TIMEOUT_S = 60.0

# Serialises start/stop. Two rapid button presses would otherwise race: both
# could see "not running" and both ask the host to start, and the second would
# collide with the first for /map and foxglove:8765.
_lock = threading.Lock()

# Reap the streaming `start` child if it has already exited. A finished ssh
# client stays visible as a zombie until reaped, and it holds a pid; dropping
# the handle would leak one per Start/Stop cycle for the life of the portal.
_child = None

# Cache for the host status probe. The 2s status tick must not spawn an ssh
# process every two seconds for the life of the dashboard, but "not running"
# is exactly the state that must not be trusted -- a portal restart loses the
# local view of an explorer that is still going, and acting on that would
# invite the user to press Start and collide with it.
_REMOTE_TTL_S = 8.0
_remote_cache = {"at": 0.0, "running": False, "robot": None}


# ----------------------- ssh plumbing ----------------------------------------
def _ssh_argv(request):
    """argv for one forced-command request.

    The request is a fixed literal built from ROBOT_NAMESPACES, never from raw
    user input. The host ignores it anyway -- sshd replaces whatever is asked
    for with explorer_host.sh -- but keeping it in the fixed grammar is what
    makes the host's own validation meaningful rather than decorative.
    """
    return ["ssh", "-T", *SSH_OPTS, SSH_TARGET, request]


def _ssh_call(request, timeout=CALL_TIMEOUT_S):
    """Run a short request to completion.

    Returns (returncode, output). returncode is None on a local failure (ssh
    missing, key unreadable, timed out) with the reason in output, because
    those are the cases the dashboard most needs to explain.
    """
    try:
        done = subprocess.run(
            _ssh_argv(request),
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return None, f"no response from {SSH_TARGET} after {timeout:.0f}s"
    except OSError as exc:
        return None, f"could not run ssh: {exc}"
    return done.returncode, (done.stdout or "") + (done.stderr or "")


def _reap():
    global _child
    if _child is not None and _child.poll() is not None:
        _child = None


def _remote_status(max_age=_REMOTE_TTL_S):
    """(running, robot) as the host sees it, rate-limited."""
    now = time.time()
    if now - _remote_cache["at"] < max_age:
        return _remote_cache["running"], _remote_cache["robot"]

    rc, out = _ssh_call("status")
    if rc is None:
        # Treat an unreachable host as "unknown", which callers read as not
        # running: refusing to start is the safe direction to fail.
        running, robot = False, None
    elif rc == 0 and out.strip().startswith("running"):
        running = True
        found = re.search(r"robot=(\S+)", out)
        robot = found.group(1) if found else None
    else:
        running, robot = False, None

    _remote_cache.update({"at": now, "running": running, "robot": robot})
    return running, robot


def _invalidate_remote():
    _remote_cache.update({"at": 0.0, "running": False, "robot": None})


# ----------------------- local bookkeeping -----------------------------------
def _read_state():
    """Return (namespace, started_at) from disk, or (None, None) if absent."""
    try:
        with open(META_FILE) as fh:
            namespace, started = fh.read().split("|", 1)
        return namespace, float(started)
    except (OSError, ValueError):
        return None, None


def _write_state(namespace):
    tmp = f"{META_FILE}.tmp"
    with open(tmp, "w") as fh:
        fh.write(f"{namespace}|{time.time()}")
    os.replace(tmp, META_FILE)


def _clear_state():
    for path in (PID_FILE, META_FILE):
        try:
            os.remove(path)
        except OSError:
            pass


# ----------------------- log helpers -----------------------------------------
def _read_tail(max_bytes=LOG_TAIL_BYTES):
    try:
        size = os.path.getsize(LOG_FILE)
    except OSError:
        return "(no log yet -- press \"Start Exploring\")"
    try:
        with open(LOG_FILE, "rb") as fh:
            if size > max_bytes:
                fh.seek(size - max_bytes)
                fh.readline()  # drop the partial first line
            data = fh.read()
    except OSError as exc:
        return f"(cannot read log: {exc})"
    return data.decode("utf-8", "replace").rstrip() or "(log is empty)"


def _read_phase():
    """Last phase marker in the streamed log, or None."""
    phase = None
    for line in _read_tail().splitlines():
        if line.startswith(PHASE_PREFIX):
            phase = line[len(PHASE_PREFIX):].strip()
    return phase


def _log_view():
    """What the dashboard's log box should show right now.

    While the start call is streaming, the host is writing to our log file, so
    read it directly -- that is the live build. Once start has returned the
    explorer is detached and writing on the host, so the only way to see it is
    to ask for the tail. Local log first, so a failed ssh call still leaves a
    readable build log on screen instead of an empty box.
    """
    if _child is not None and _child.poll() is None:
        return _read_tail()
    rc, out = _ssh_call("log")
    if rc == 0 and out.strip() and "no launch log" not in out:
        return out.rstrip()
    return _read_tail()


# ----------------------- public API ------------------------------------------
def start(robot_label):
    """Start (bridge + build + launch) the explorer for `robot_label`.

    Returns (status_line, log_tail) for the dashboard.
    """
    namespace = ROBOT_NAMESPACES.get(robot_label)
    if namespace is None:
        return (f"❌ Unknown robot {robot_label!r}.", _read_tail())

    with _lock:
        _reap()
        _invalidate_remote()

        # The host is the only authority on what is running: a previous start
        # may have been issued by an older portal process, or the launch may
        # outlive the dashboard entirely.
        running, running_robot = _remote_status(max_age=0)
        if running:
            label = _label_for(running_robot) if running_robot else "An explorer"
            if running_robot == namespace:
                return (
                    f"❌ {robot_label} is already exploring. Stop it first.",
                    _log_view(),
                )
            return (
                f"❌ {label} is already exploring. Only one explorer can run "
                f"at a time -- stop it before starting {robot_label}.",
                _log_view(),
            )

        if not os.path.exists(SSH_KEY):
            return (
                f"❌ {SSH_KEY} is missing from the dashboard container, so the "
                "server cannot be reached. Ask an admin to re-run "
                "scripts/dashboard/install_explorer_key.sh on the host.",
                _read_tail(),
            )

        # 'wb' truncates, so each start begins with a log that is about this
        # run only -- otherwise the tail is dominated by the previous session.
        try:
            log_fh = open(LOG_FILE, "wb")
        except OSError as exc:
            return (f"❌ Cannot open {LOG_FILE}: {exc}", _read_tail())

        try:
            proc = subprocess.Popen(
                _ssh_argv(f"start {namespace}"),
                stdin=subprocess.DEVNULL,
                stdout=log_fh,
                stderr=subprocess.STDOUT,
                # Own session => pgid == pid, so the local handle can be
                # signalled as a group if the host call wedges.
                start_new_session=True,
            )
        except OSError as exc:
            return (f"❌ Failed to reach the server: {exc}", _read_tail())
        finally:
            # The child holds its own dup of the fd; the parent's copy is dead
            # weight and would otherwise leak on every start.
            log_fh.close()

        try:
            with open(PID_FILE, "w") as fh:
                fh.write(f"{proc.pid}\n")
        except OSError:
            pass
        _write_state(namespace)

        global _child
        _child = proc

    return (
        f"⏳ Starting {robot_label}: starting the Zenoh bridge, then building "
        f"{' + '.join(BUILD_PACKAGES)}. A cold build takes a few minutes and "
        "the log below updates as it goes; the launch starts on its own.",
        _read_tail(),
    )


def stop():
    """Stop the running explorer and all of its nodes. Returns (status, log)."""
    with _lock:
        _reap()
        namespace, _ = _read_state()

        # Ask the host unconditionally, even with no local state: the explorer
        # is detached from us, so the local files are a hint, not a fact.
        rc, out = _ssh_call("stop", timeout=STOP_TIMEOUT_S)
        _invalidate_remote()

        if rc is None:
            return (
                f"❌ Could not reach the server to stop the explorer: {out}",
                _log_view(),
            )
        if rc != 0:
            detail = out.strip().splitlines()[-1] if out.strip() else f"exit {rc}"
            return (f"❌ Stop failed on the server: {detail}", _log_view())

        # The host has released the launch. Reap the streaming start call so it
        # does not linger as a zombie holding a pid.
        if _child is not None:
            try:
                _child.wait(timeout=5)
            except (subprocess.TimeoutExpired, OSError):
                pass
        _clear_state()

        label = _label_for(namespace) if namespace else "The explorer"
        message = out.strip().splitlines()[-1] if out.strip() else "stopped"
        if "no explorer" in message:
            return (f"ℹ️ {label} was not running.", _log_view())
        return (f"🛑 Stopped {label}.", _log_view())


def _label_for(namespace):
    return next((k for k, v in ROBOT_NAMESPACES.items() if v == namespace),
                namespace or "?")


def status():
    """Live state for the dashboard's status box. Returns (status, log)."""
    _reap()
    namespace, started_at = _read_state()
    streaming = _child is not None and _child.poll() is None
    phase = _read_phase() if streaming else None
    log = _log_view()

    if streaming:
        return (_streaming_status(namespace, started_at, phase), log)

    # Nothing streaming here. The host decides what is actually running.
    running, remote_robot = _remote_status()
    if running:
        label = _label_for(remote_robot or namespace)
        if namespace and remote_robot and remote_robot != namespace:
            label = f"{label} (started from another dashboard session)"
        return (f"🟢 {label} is exploring.", log)

    # A failed build is the single most likely thing to go wrong, and it takes
    # the whole process down with it -- so by the time we look, the start call
    # is already gone and "Not exploring" would hide the only useful clue.
    # Read it back from the phase marker left in the log.
    if phase and phase.startswith("build-failed"):
        rc = phase.partition(" ")[2].strip()
        if rc.startswith("rc="):
            rc = rc[3:]
        return (
            f"❌ Build FAILED (exit {rc or '?'}) -- the launch never ran. Fix "
            "the error in the log below, then press Start Exploring.",
            log,
        )
    if phase and phase.startswith("launch-failed"):
        return (
            "❌ The launch exited immediately -- see the log below. The build "
            "succeeded, so this is the launch itself failing.",
            log,
        )
    return ("⚪ Not exploring.", log)


def _streaming_status(namespace, started_at, phase):
    label = _label_for(namespace)
    mins = (time.time() - started_at) / 60.0 if started_at else 0.0

    if phase == "bridge":
        head = f"🌉 Starting the Zenoh bridge for {label} ({mins:.0f}m elapsed)"
        tail = "the build follows as soon as the bridge is up."
    elif phase == "build":
        head = f"🔨 Building the {label} explorer ({mins:.0f}m elapsed)"
        tail = "the launch starts as soon as the build finishes."
    elif phase and phase.startswith("build-failed"):
        rc = phase.partition(" ")[2].strip()
        if rc.startswith("rc="):
            rc = rc[3:]
        return (f"❌ {label} build FAILED (exit {rc or '?'}) -- the launch "
                "never ran. See the log below.")
    elif phase == "launch":
        head = f"🚀 Launching {label} ({mins:.0f}m elapsed)"
        tail = "bringing up the map, lidar slam and frontier explorer nodes."
    elif phase == "launched":
        head = f"✅ {label} launched"
        tail = "the nodes are up and the map above should start filling in."
    else:
        # Started, but no phase marker yet: still connecting to the host.
        head = f"⏳ Starting {label} ({mins:.0f}m elapsed)"
        tail = "reaching the server..."
    return f"{head} -- {tail}"
