"""
foxglove_viewer.py - Foxglove-style 3D view for the QOD lab dashboard.

A standalone side-car process (deliberately NOT part of the lab_portal.py
process). It owns its own rclpy node, subscribes to the same ROS_DOMAIN_ID=70
graph the portal uses, and serves a Three.js viewer over a single HTTP +
WebSocket port. The portal embeds it in an iframe, exactly the way it already
embeds the patrol planner on port 8766.

Why a separate process:
  The dashboard sits behind a boot.sh watchdog because it dies often enough to
  matter. The 3D view is a nice-to-have; if it crashes it must not take the map
  and camera down with it. Out-of-process, a malformed cloud or a WebGL failure
  can only ever break the iframe.

What it renders, and why it is not just a point-cloud dump:
  * /<robot>/go2/restamped/cloud_base  PointCloud2, 10 Hz, ~1.2k points/msg
  * /<robot>/amcl_pose                 map-frame pose (authoritative)
  * /<robot>/go2/restamped/robot_odom  odom-frame pose (10x the rate)
  * /<robot>/tf_static                 base_link -> <robot>/utlidar_lidar
  * /<robot>/lidar_slam_2d_map         occupancy grid, drawn as the floor
  * /<robot>/modified_map               lidar_slam's graph-optimised map cloud
  * /<robot>/plan                       Nav2 path

The lidar publishes in its own frame (<robot>/utlidar_lidar), so a raw feed
swims around with the robot and is useless as a map. Foxglove does not do that,
and neither do we: every incoming cloud is transformed into the map frame on
arrival and kept for a short rolling window (see FOXGLOVE_WINDOW_SECS). That is
what makes the world appear still while the robot drives through it, without the
register slip a long accumulation suffers as SLAM's correction drifts.

Frames:
  ROS is Z-up, three.js is Y-up, so a map-frame ROS point (x, y, z) is displayed
  as (x, z, -y). Negating one axis converts handedness, which keeps x east and
  y north reading the right way round on screen. The floor plane's -90 deg X
  rotation performs the same conversion for the map texture, so the point cloud
  and the floor agree without any per-frame fudge factor.

Bandwidth:
  This is reached over an SSH tunnel or a 5G link as often as over the LAN, so
  frames go out as binary WebSocket messages -- no base64, no per-point JSON --
  and the accumulated buffer is stride-sampled to a fixed budget first.

Usage:
    python3 -m web_backend.foxglove_viewer
Environment:
    FOXGLOVE_PORT         port to serve on                   (default 8767)
    FOXGLOVE_MAX_POINTS   points per streamed frame          (default 150000)
    FOXGLOVE_FPS          streamed frames per second         (default 10)
    FOXGLOVE_WINDOW_SECS  seconds of scans kept in the buffer (default 2.0)
    FOXGLOVE_ACCUM_MAX    hard point cap for the buffer       (default 2000000)
    FOXGLOVE_CLOUD_TOPIC  point-cloud topic to view           (default /go2/accumulated/cloud_base)
    FOXGLOVE_MODIFIED_MAP_MAX  cap on the /modified_map layer (default 400000)
"""

import asyncio
import base64
from collections import deque
import json
import math
import os
import signal
import struct
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC_ROOT = os.path.join(os.path.dirname(HERE), "web_frontend")


def _env_int(name, default):
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


PORT = _env_int("FOXGLOVE_PORT", 8767)
MAX_POINTS_PER_FRAME = _env_int("FOXGLOVE_MAX_POINTS", 150000)
TARGET_FPS = float(_env_int("FOXGLOVE_FPS", 10))
# Rolling window over which scans are kept in the map frame. Foxglove shows a
# live cloud, so only recent scans may be drawn: each scan is frozen into map
# coordinates as it arrives, and SLAM's map<-odom correction keeps drifting, so
# a long accumulation slowly slides out of register with the current grid.
WINDOW_SECS = _env_float("FOXGLOVE_WINDOW_SECS", 2.0)
# Hard memory guard for the rolling buffer; only reached if the window is set
# extremely large.
ACCUM_MAX_POINTS = _env_int("FOXGLOVE_ACCUM_MAX", 2_000_000)
# The modified map is a whole optimised map, not a rolling window of scans, so
# it is sampled once to this budget and then held until SLAM republishes it.
MODIFIED_MAP_MAX_POINTS = _env_int("FOXGLOVE_MODIFIED_MAP_MAX", 400000)

# Shared by the page, the portal iframe and the handshake interceptor below, so
# the socket path exists in exactly one place.
WS_PATH = "/ws/foxglove"

ROBOT_NAMESPACES = ("luna", "astro")
ROBOT_LABELS = {"luna": "Luna", "astro": "Astro"}

LIDAR_TOPIC = "/{ns}/go2/restamped/cloud_base"
# Optional cloud-topic override. Defaults to the motion-compensated scan
# accumulator output ("/go2/accumulated/cloud_base", already in base_link) so
# the map view can be trialled against it. Set FOXGLOVE_CLOUD_TOPIC to
# "/{ns}/go2/restamped/cloud_base" (or any other topic) to switch, and to ""
# to fall back to the raw restamped cloud. A value without a "{ns}" placeholder
# is used verbatim for every robot.
CLOUD_TOPIC = os.environ.get(
    "FOXGLOVE_CLOUD_TOPIC", "/go2/accumulated/cloud_base"
).strip() or LIDAR_TOPIC


def cloud_topic(ns):
    return CLOUD_TOPIC.format(ns=ns)


MAP_TOPIC = "/{ns}/lidar_slam_2d_map"
AMCL_TOPIC = "/{ns}/amcl_pose"
ODOM_TOPIC = "/{ns}/go2/restamped/robot_odom"
TF_STATIC_TOPIC = "/{ns}/tf_static"
PLAN_TOPIC = "/{ns}/plan"
# SLAM's map <- odom correction (TransformStamped, frame_id="map",
# child_frame_id="<ns>/odom"). The lidar SLAM stack publishes this so
# consumers can lift the drift-prone odom frame into the map frame that the
# occupancy grid and the real Foxglove app both use. Without it the viewer
# drew odom as if it were map, which offset the cloud and the robot body from
# the occupancy grid by a fixed translation + yaw.
MAP_ODOM_CORRECTION_TOPIC = "/{ns}/go2/split_compute/map_odom_correction"

# lidar_slam's graph-optimised map: what the real Foxglove app shows as a dense,
# static cloud that stays put while the dog drives. Published by
# graph_based_slam as a sensor_msgs/PointCloud2 (NOT a nav_msgs/OccupancyGrid)
# already stamped in the "map" frame, so unlike the live cloud it needs no
# transform into the map frame -- only the ROS->three.js axis swap. It is the
# pose-graph output of /map_array, i.e. the same geometry the 2D grid is
# projected from, so it registers with the floor instead of needing to be
# aligned to it.
#
# GLOBAL, not per-robot, and deliberately so: graph_based_slam_node is not
# namespaced and the explorer launch remaps only /modified_map_array, leaving
# this one as /modified_map. Verified on the robot --
# `ros2 topic info /modified_map -v` shows publisher graph_based_slam and
# subscriber foxglove_bridge. There is no /luna/modified_map or
# /astro/modified_map publisher, so a "/{ns}/modified_map" guess subscribes to
# nothing at all.
#
# It fires on every pose-graph optimisation (seconds apart while loop closures
# are being accepted, much longer when the dog is only driving) and is
# RELIABLE/VOLATILE, so a late-joining viewer waits for the next optimisation
# rather than being handed a latched copy.
MODIFIED_MAP_TOPIC = os.environ.get(
    "FOXGLOVE_MODIFIED_MAP_TOPIC", "/modified_map").strip() or "/modified_map"


def modified_map_topic(ns):
    return MODIFIED_MAP_TOPIC.format(ns=ns)

MSG_CLOUD = 1
MSG_PATH = 2
MSG_MAP_CLOUD = 3

# The Hivemind mission manager's own state, which is what RViz was displaying as
# "Exploring". It publishes a JSON string at ~5 Hz on a single global topic from
# a single publisher (node `hivemind_mission_manager`) --
#
#   {"completion_status": "Complete", "explorer_robot": "luna",
#    "mission_state": "MAP_COMPLETE", "selected_namespace": "/luna"}
#
# Subscribed purely to be displayed. Nothing here publishes, commands or
# otherwise influences the mission -- this is a read-only mirror of a topic the
# operator UI already surfaces.
MISSION_TOPIC = os.environ.get(
    "FOXGLOVE_MISSION_TOPIC", "/hivemind/mission_state"
).strip() or "/hivemind/mission_state"

# The full enum from go2_hardware_autonomy.hivemind_mission.MissionState.
# Unknown values are passed through rather than dropped, so a state added
# upstream shows up instead of silently reading as "idle".
MISSION_STATES = ("IDLE", "EXPLORING", "MAP_COMPLETE", "READY_FOR_PATROL")

# The mission topic is ~5 Hz, so this is ~50 missed messages before the readout is
# called stale. Without it the last state ever seen is displayed forever as if it
# were live: when the explorer is stopped the publisher simply disappears, the
# value never changes again, and a change-gated push sends nothing further -- so
# the HUD would keep claiming "Exploring" hours after exploration ended.
MISSION_STALE_SECONDS = 10.0

# Even while stale the frame is refreshed on this interval, so the "no update
# for Ns" figure on screen keeps counting instead of freezing at the moment the
# feed died. One small JSON frame every couple of seconds is nothing next to the
# cloud at 10 fps.
MISSION_PUSH_INTERVAL = 2.0

# Wire header: u8 type, u8 flags, u16 reserved, u32 count, f32 x, y, z, yaw.
# Defined once and reused by both frame encoders so the two can never disagree
# about the layout the browser decodes.
_HEADER_FMT = "<BBHIffff"
HEADER_BYTES = struct.calcsize(_HEADER_FMT)

# An odom reading older than this is not extrapolated; AMCL alone is used, so a
# stalled odom topic freezes the marker instead of sending it off on its own.
ODOM_MAX_AGE = 1.5
PATH_MAX_POINTS = 4000


def _log(msg):
    print(f"[foxglove] {msg}", flush=True)


def _wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def pose_matrix(x, y, yaw, z=0.0):
    """2D SE(2) pose as a 4x4: maps a point in the local frame to the map frame.

    z defaults to 0 because a ground robot on a flat floor reports it as 0, but
    it is kept as a real parameter rather than dropped: dropping it shifts the
    whole cloud to the wrong height the moment the robot rides a ramp or the
    odom origin sits below the map origin, and that shows up as a cloud that is
    visibly buried in or floating over the floor.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    m = np.eye(4, dtype=np.float64)
    m[0, 0], m[0, 1] = c, -s
    m[1, 0], m[1, 1] = s, c
    m[0, 3], m[1, 3], m[2, 3] = x, y, z
    return m


def quaternion_matrix(x, y, z, w, tx, ty, tz):
    """geometry_msgs Quaternion + Vector3 -> 4x4 transform."""
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    m = np.eye(4, dtype=np.float64)
    m[0, 0] = 1 - 2 * (y * y + z * z)
    m[0, 1] = 2 * (x * y - z * w)
    m[0, 2] = 2 * (x * z + y * w)
    m[1, 0] = 2 * (x * y + z * w)
    m[1, 1] = 1 - 2 * (x * x + z * z)
    m[1, 2] = 2 * (y * z - x * w)
    m[2, 0] = 2 * (x * z - y * w)
    m[2, 1] = 2 * (y * z + x * w)
    m[2, 2] = 1 - 2 * (x * x + y * y)
    m[0, 3], m[1, 3], m[2, 3] = tx, ty, tz
    return m


# ===========================================================================
# PointCloud2 decoding
# ===========================================================================
# sensor_msgs/msg/PointField datatype -> numpy dtype
_PC_DTYPES = {
    1: np.dtype("<i1"),    # INT8
    2: np.dtype("<u1"),    # UINT8
    3: np.dtype("<i2"),    # INT16
    4: np.dtype("<u2"),    # UINT16
    5: np.dtype("<i4"),    # INT32
    6: np.dtype("<u4"),    # UINT32
    7: np.dtype("<f4"),    # FLOAT32
    8: np.dtype("<f8"),    # FLOAT64
}
_PC_INTEGER = {1, 2, 3, 4, 5, 6}


def _apply_colormap(t, colormap):
    """Map normalised intensity t in [0,1] onto 0-255 RGB.

    Neither of these is real colour -- see decode_point_cloud's docstring. They
    only differ in how the single intensity scalar is stretched across the RGB
    cube, and "jet" exists so the map matches what the Foxglove app draws.
    """
    if colormap == "jet":
        # Piecewise-linear jet: dark blue -> blue -> cyan -> green -> yellow ->
        # orange -> dark red. Written as clamped triangles around the 0.25 /
        # 0.5 / 0.75 stops, which is cheaper than a lookup table and smooth
        # enough at these point sizes.
        rgb = np.empty((t.shape[0], 3), dtype=np.uint8)
        rgb[:, 0] = (np.clip(1.5 - np.abs(4.0 * t - 3.0), 0.0, 1.0) * 255.0
                     ).astype(np.uint8)
        rgb[:, 1] = (np.clip(1.5 - np.abs(4.0 * t - 2.0), 0.0, 1.0) * 255.0
                     ).astype(np.uint8)
        rgb[:, 2] = (np.clip(1.5 - np.abs(4.0 * t - 1.0), 0.0, 1.0) * 255.0
                     ).astype(np.uint8)
        return rgb

    # "warm": blue (weak return) -> yellow (strong), reads well on a dark scene
    # and keeps the live cloud's established look.
    rgb = np.empty((t.shape[0], 3), dtype=np.uint8)
    rgb[:, 0] = (t * 255.0).astype(np.uint8)
    rgb[:, 1] = (t * 210.0).astype(np.uint8)
    rgb[:, 2] = ((1.0 - t) * 255.0).astype(np.uint8)
    return rgb


def decode_point_cloud(msg, colormap="warm"):
    """PointCloud2 -> (Nx3 float32 xyz, Nx3 uint8 rgb), or None.

    Not pc2.read_points: that builds a Python tuple per point and cannot keep up
    with a 10 Hz stream on its own. A PointCloud2 buffer is a flat array of
    fixed-size records, so a strided numpy view reads it in one pass.

    The Go2's cloud_base is float32 x/y/z with a float32 intensity and a uint16
    ring index. Other drivers publish the same fields as uint16 millimetres, so
    integer x/y/z are scaled by 1e-3 rather than trusted as metres.

    ``colormap`` picks how intensity becomes colour, and it matters because the
    hardware has no RGB to preserve: both clouds carry xyz+intensity only, and
    lidar_slam's modified map is PointXYZI by construction, so every colour on
    screen is a function of intensity. "warm" is the default live-cloud look
    (blue->yellow); "jet" is the classic blue->cyan->green->yellow->red ramp
    that Foxglove's own intensity colouring approximates, so a map rendered with
    it reads like the Foxglove app. There is deliberately no "rgb" mode: no
    publisher on this robot carries an rgb field to read.
    """
    data = msg.data
    if data is None or len(data) == 0:
        return None

    n = int(msg.width) * int(msg.height)
    if n <= 0:
        return None

    fields = {}
    for f in msg.fields:
        # A duplicate name cannot occur in a valid cloud, but a later field
        # shadowing the real one would silently misread every point.
        fields.setdefault(f.name, f)
    if not all(k in fields for k in ("x", "y", "z")):
        return None

    buf = memoryview(bytes(data))
    stride = int(msg.point_step)
    if stride <= 0:
        return None

    def field(name):
        f = fields[name]
        dt = _PC_DTYPES.get(f.datatype)
        if dt is None:
            return None, False
        offset = int(f.offset)
        if offset + dt.itemsize > stride:
            return None, False
        try:
            arr = np.ndarray(shape=(n,), dtype=dt, buffer=buf,
                             offset=offset, strides=(stride,))
        except (ValueError, BufferError):
            return None, False
        return np.array(arr, dtype=np.float32), f.datatype in _PC_INTEGER

    xs, x_is_int = field("x")
    ys, _ = field("y")
    zs, _ = field("z")
    if xs is None or ys is None or zs is None:
        return None
    if x_is_int:
        xs, ys, zs = xs * 1e-3, ys * 1e-3, zs * 1e-3

    xyz = np.stack([xs, ys, zs], axis=1)
    keep = np.isfinite(xyz).all(axis=1)
    xyz = xyz[keep]
    if xyz.shape[0] == 0:
        return None

    rgb = None
    for name in ("intensity", "reflectivity", "i", "ring"):
        if name not in fields:
            continue
        vals, is_int = field(name)
        if vals is None:
            continue
        vals = vals[keep]
        if is_int:
            vals = vals * 1e-3
        peak = float(np.percentile(np.abs(vals), 98))
        if peak <= 0:
            continue
        t = np.clip(np.abs(vals) / peak, 0.0, 1.0)
        rgb = _apply_colormap(t, colormap)
        break

    if rgb is None:
        # No intensity field: colour by height so walls, clutter and the floor
        # separate visually instead of a uniform grey blob.
        z = xyz[:, 2]
        lo, hi = np.percentile(z, 4), np.percentile(z, 97)
        span = (hi - lo) or 1.0
        t = np.clip((z - lo) / span, 0.0, 1.0)
        rgb = np.empty((t.shape[0], 3), dtype=np.uint8)
        rgb[:, 0] = (t * 255.0).astype(np.uint8)
        rgb[:, 1] = (80.0 + t * 100.0).astype(np.uint8)
        rgb[:, 2] = ((1.0 - t) * 255.0).astype(np.uint8)

    return xyz.astype(np.float32), rgb


# ===========================================================================
# Map-frame accumulation
# ===========================================================================
class MapFrameBuffer:
    """Lidar frames held in the map frame over a short rolling time window.

    Each incoming frame is stored with a wall-clock stamp and frames older than
    ``window_secs`` are dropped. This mirrors how the Foxglove app renders a
    live point cloud: only recent scans are shown, so the cloud always tracks
    the current occupancy grid.

    Why not a persistent, ever-growing map: every scan is frozen into map
    coordinates at the moment it arrives, using SLAM's map<-odom correction as
    it stands then. That correction keeps moving as odom drifts and the graph is
    optimised, so points captured minutes ago slowly slide out of register with
    the current grid -- the cloud drifts even though the robot marker, which is
    drawn with the live pose, stays put. A rolling window sidesteps the whole
    problem and matches Foxglove's behaviour.

    ``capacity`` is only a hard memory guard for pathological windows.
    """

    def __init__(self, window_secs, capacity):
        self.window_secs = max(0.1, float(window_secs))
        self.capacity = max(1000, int(capacity))
        self._frames = deque()      # (stamp, xyz, rgb)
        self._n = 0
        self.dropped_frames = 0

    def __len__(self):
        return self._n

    # Retained for the /health shape. A rolling window never decimates or
    # compacts the way the old unbounded buffer did.
    @property
    def stride(self):
        return 1

    @property
    def compactions(self):
        return 0

    @property
    def span_secs(self):
        if len(self._frames) < 2:
            return 0.0
        return self._frames[-1][0] - self._frames[0][0]

    def _evict(self):
        cutoff = time.time() - self.window_secs
        while self._frames and self._frames[0][0] < cutoff:
            self._n -= len(self._frames.popleft()[1])
            self.dropped_frames += 1
        while self._n > self.capacity and len(self._frames) > 1:
            self._n -= len(self._frames.popleft()[1])
            self.dropped_frames += 1

    def extend(self, xyz, rgb):
        n = int(xyz.shape[0])
        if n == 0:
            return
        self._frames.append((time.time(), xyz, rgb))
        self._n += n
        self._evict()

    def sample(self, budget):
        """Concatenate the live window and stride it down to ~budget points."""
        if self._n == 0:
            return None, None
        xyz = np.concatenate([f[1] for f in self._frames], axis=0)
        rgb = np.concatenate([f[2] for f in self._frames], axis=0)
        total = xyz.shape[0]
        if total <= budget:
            return xyz, rgb
        step = int(math.ceil(total / budget))
        return xyz[::step], rgb[::step]


class ModifiedMapLayer:
    """The graph-optimised map, owned once for the whole bridge.

    graph_based_slam_node is a single un-namespaced node publishing one
    /modified_map, so it describes the same world for both dogs. One
    subscription and one copy therefore beat one per robot, which would decode
    a multi-megabyte cloud twice and hold it twice in memory for no gain.

    Separate from RobotState on purpose: everything in RobotState is placed
    through a robot pose, and this layer must not be -- it is already in the map
    frame, so feeding it through map_from_lidar() would translate and rotate the
    entire map every time the dog moved a centimetre.
    """

    def __init__(self, node):
        self.topic = MODIFIED_MAP_TOPIC
        self.xyz = None
        self.rgb = None
        self.seq = 0
        self.msgs = 0
        self.undecodable = 0
        self.last_wall = 0.0
        self.last_error = None
        self._subscribe(node)

    def _subscribe(self, node):
        from sensor_msgs.msg import PointCloud2
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        # Upstream is RELIABLE/VOLATILE (rclcpp::QoS(10)), so the volatile
        # subscription is the one that actually matches today. The latched one
        # costs nothing and covers a future relay that republishes this as
        # TRANSIENT_LOCAL, which is how /lidar_slam_2d_map is served.
        volatile = QoSProfile(depth=1)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(PointCloud2, self.topic, self._on_map, volatile)
        node.create_subscription(PointCloud2, self.topic, self._on_map, latched)

    def _on_map(self, msg):
        """One pose-graph optimisation result, drawn as-is.

        Replace rather than accumulate: each message is the complete optimised
        map, so merging successive ones would only smear the previous
        optimisation's geometry over the current one.
        """
        try:
            # "jet" so the map reads like the Foxglove app. It has no RGB to
            # preserve either -- PointXYZI -- so this is intensity through a
            # colormap, exactly as Foxglove does it.
            decoded = decode_point_cloud(msg, colormap="jet")
            if decoded is None:
                self.undecodable += 1
                return
            xyz, rgb = decoded

            # Z-up -> Y-up only. No translation, no rotation: the publisher
            # already stamped this in "map".
            out = np.empty_like(xyz)
            out[:, 0] = xyz[:, 0]
            out[:, 1] = xyz[:, 2]
            out[:, 2] = -xyz[:, 1]

            total = out.shape[0]
            if total > MODIFIED_MAP_MAX_POINTS:
                step = int(math.ceil(total / MODIFIED_MAP_MAX_POINTS))
                out, rgb = out[::step], rgb[::step]

            self.xyz = out
            self.rgb = rgb
            self.msgs += 1
            self.seq += 1
            self.last_wall = time.time()
        except Exception as exc:  # noqa: BLE001 - one bad map must not kill the node
            self.last_error = repr(exc)

    def health(self):
        return {
            "modified_map_topic": self.topic,
            "modified_map_msgs": self.msgs,
            "modified_map_points": 0 if self.xyz is None
                                  else int(self.xyz.shape[0]),
            # Null while nothing has ever arrived, so "never seen" stays
            # distinguishable from "seen long ago".
            "last_modified_map_age": (
                None if not self.last_wall
                else round(time.time() - self.last_wall, 2)),
            "modified_map_undecodable": self.undecodable,
            "modified_map_error": self.last_error,
        }


# ===========================================================================
# Hivemind mission state (read-only status mirror)
# ===========================================================================
class MissionStateView:
    """Mirrors the mission manager's state so the viewer can show it.

    Display only. The viewer has no publisher on any mission topic and no
    service client; it never sends a goal or a cancel. That is deliberate --
    an operator-facing status readout should not be able to steer the robot.

    The payload is a std_msgs/String holding JSON. It is parsed rather than
    pattern-matched so that fields added upstream (completion_status,
    selected_namespace) show up without a code change here.
    """

    def __init__(self, node):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.topic = MISSION_TOPIC
        self.state = None
        self.explorer_robot = None
        self.selected_namespace = None
        self.completion_status = None
        self.msgs = 0
        self.last_wall = None
        self.last_error = None

        # Reliable plus TRANSIENT_LOCAL as well as volatile: the publisher is
        # only ~5 Hz but is otherwise well behaved, and a latched copy means a
        # viewer opened mid-mission shows the current state immediately rather
        # than "unknown" until the next tick.
        volatile = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, self.topic, self._on_state, volatile)
        node.create_subscription(String, self.topic, self._on_state, latched)

    def _on_state(self, msg):
        try:
            data = json.loads(msg.data)
            state = str(data.get("mission_state") or "").strip().upper()
            if not state:
                raise ValueError("no mission_state in payload")
            # Not validated against MISSION_STATES on purpose: an unrecognised
            # state is news, and hiding it would look identical to "idle".
            self.state = state
            self.explorer_robot = data.get("explorer_robot")
            self.selected_namespace = data.get("selected_namespace")
            self.completion_status = data.get("completion_status")
            self.msgs += 1
            self.last_wall = time.time()
            self.last_error = None
        except Exception as exc:  # noqa: BLE001 - a bad payload must not kill the node
            self.last_error = repr(exc)

    def snapshot(self):
        """(key, payload) for change-gating the websocket push."""
        return (self.state, self.explorer_robot, self.completion_status)

    def health(self):
        return {
            "mission_topic": self.topic,
            "mission_state": self.state,
            "mission_known_states": list(MISSION_STATES),
            "explorer_robot": self.explorer_robot,
            "selected_namespace": self.selected_namespace,
            "completion_status": self.completion_status,
            "mission_msgs": self.msgs,
            # Null until the first message, so "never heard from it" stays
            # distinguishable from "stale".
            "last_mission_age": (None if not self.last_wall
                                 else round(time.time() - self.last_wall, 2)),
            "mission_error": self.last_error,
        }


# ===========================================================================
# Per-robot state
# ===========================================================================
class RobotState:
    def __init__(self, ns, node, modmap=None):
        self.ns = ns
        self.label = ROBOT_LABELS[ns]
        # Shared across robots (see ModifiedMapLayer); may be None in tests.
        self.modmap = modmap

        self.buffer = MapFrameBuffer(WINDOW_SECS, ACCUM_MAX_POINTS)
        self.map_png = None
        self.map_meta = None
        self.map_seq = 0
        self.plan = []
        self.plan_stamp = 0.0

        self.pose = None            # map frame, from AMCL
        self.pose_stamp = 0.0
        self.odom_pose = None       # odom frame
        self.odom_stamp = 0.0
        # map <- odom from SLAM (/…/split_compute/map_odom_correction). This is
        # the transform that lifts odom-frame data into the map frame the
        # occupancy grid lives in; composed with the odom pose it yields a live
        # map-frame pose even while AMCL is silent.
        self.map_from_odom = None
        self.map_from_odom_stamp = 0.0
        # Odom reading captured at the moment the last AMCL fix arrived. Odom is
        # 10x faster but drifts; anchoring it to that instant is what lets the
        # robot move smoothly between (slow) AMCL updates without ever leaving
        # the map frame. Set in _on_amcl, never on a timer -- a timer would
        # re-anchor every sample and the delta would always be zero.
        self.odom_anchor = None

        # base_link -> <ns>/utlidar_lidar from tf_static. Identity by default,
        # which is what the Go2 publishes anyway, so a missing tf_static costs
        # a small fixed offset instead of dropping every cloud.
        self.base_from_lidar = np.eye(4, dtype=np.float64)

        self.cloud_msgs = 0
        self.last_cloud_wall = 0.0
        # Points discarded because no pose was available to place them. A
        # cumulative counter is useless here: a robot that never gets a pose
        # (astro, right now) increments it on every cloud forever, and it was
        # reporting tens of millions. The streak start lets health() publish a
        # rate instead, which stays readable however long the outage lasts.
        self.dropped_no_pose = 0
        self._drop_streak_start = None
        self.last_error = None

        self._subscribe(node)

    def _subscribe(self, node):
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from tf2_msgs.msg import TFMessage

        # Lidar and odom are republished by zenoh_ros2dds as BEST_EFFORT sensor
        # data. A RELIABLE subscription would be QoS-incompatible and would
        # silently receive nothing -- the same trap the portal's odom
        # subscription already documents.
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # The map relay publishes latched, so the viewer must ask for
        # TRANSIENT_LOCAL too or it joins late and never sees the map.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # AMCL and the Nav2 plan are RELIABLE + VOLATILE; the default profile
        # matches.
        default_qos = QoSProfile(depth=10)

        node.create_subscription(PointCloud2, cloud_topic(self.ns),
                                 self._on_cloud, sensor_qos)
        node.create_subscription(OccupancyGrid, MAP_TOPIC.format(ns=self.ns),
                                 self._on_map, map_qos)
        node.create_subscription(PoseWithCovarianceStamped,
                                 AMCL_TOPIC.format(ns=self.ns),
                                 self._on_amcl, default_qos)
        node.create_subscription(Odometry, ODOM_TOPIC.format(ns=self.ns),
                                 self._on_odom, sensor_qos)
        node.create_subscription(Path, PLAN_TOPIC.format(ns=self.ns),
                                 self._on_plan, default_qos)
        # tf_static is subscribed twice on purpose. Per REP 105 it is published
        # TRANSIENT_LOCAL, which is what map_qos asks for, and that is also what
        # gets the latched transform when the viewer joins after the robot
        # started. But '/astro/tf_static' is published VOLATILE, and QoS
        # compatibility runs one way only: a TRANSIENT_LOCAL subscriber can
        # never hear a VOLATILE publisher (observed in this container as an
        # "incompatible QoS ... DURABILITY" warning with no transforms ever
        # arriving). A VOLATILE subscription does hear both, so the pair covers
        # the standard publisher and the non-conformant one.
        node.create_subscription(TFMessage, TF_STATIC_TOPIC.format(ns=self.ns),
                                 self._on_tf_static, map_qos)
        node.create_subscription(TFMessage, TF_STATIC_TOPIC.format(ns=self.ns),
                                 self._on_tf_static, sensor_qos)
        # SLAM's map<-odom correction. Published latched RELIABLE/TRANSIENT_LOCAL
        # by the lidar SLAM stack, so the same dual-subscription pattern as
        # tf_static covers both a latched publisher and a best-effort relay.
        node.create_subscription(TransformStamped,
                                 MAP_ODOM_CORRECTION_TOPIC.format(ns=self.ns),
                                 self._on_map_odom, map_qos)
        node.create_subscription(TransformStamped,
                                 MAP_ODOM_CORRECTION_TOPIC.format(ns=self.ns),
                                 self._on_map_odom, sensor_qos)

    # ------------------------------------------------------------ callbacks
    def _on_cloud(self, msg):
        try:
            decoded = decode_point_cloud(msg)
            if decoded is None:
                return
            xyz, rgb = decoded

            tf = self.map_from_lidar()
            if tf is None:
                # Discarded, not buffered: there is nowhere to put them.
                if self._drop_streak_start is None:
                    self._drop_streak_start = time.time()
                self.dropped_no_pose += int(xyz.shape[0])
                return

            r = tf[:3, :3]
            t = tf[:3, 3]
            px, py, pz = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            mx = r[0, 0] * px + r[0, 1] * py + r[0, 2] * pz + t[0]
            my = r[1, 0] * px + r[1, 1] * py + r[1, 2] * pz + t[1]
            mz = r[2, 0] * px + r[2, 1] * py + r[2, 2] * pz + t[2]

            # Map-frame ROS point (x, y, z) -> three.js (x, z, -y).
            out = np.empty_like(xyz)
            out[:, 0] = mx
            out[:, 1] = mz
            out[:, 2] = -my

            self.buffer.extend(out, rgb)
            self.cloud_msgs += 1
            self.last_cloud_wall = time.time()
            if self.dropped_no_pose:
                self.dropped_no_pose = 0
                self._drop_streak_start = None
        except Exception as exc:  # noqa: BLE001 - one bad frame must not kill the node
            self.last_error = repr(exc)

    def _on_map(self, msg):
        try:
            import cv2

            info = msg.info
            grid = np.array(msg.data, dtype=np.float32).reshape(info.height, info.width)
            grid = np.flipud(grid)

            disp = np.ones_like(grid, dtype=np.float32)
            disp[grid == -1] = 0.45      # unknown
            disp[grid == 0] = 1.0        # free
            disp[grid > 0] = 0.0         # occupied
            gray = (disp * 255).astype(np.uint8)
            img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            # Downscale for transport but keep the grid's own aspect ratio. The
            # browser sizes the floor plane from width*resolution x
            # height*resolution, so padding the image to a square canvas would
            # stretch the map off the point cloud along whichever axis is
            # shorter (and drag the robot body off its own scans with it).
            max_px = 512
            scale = max_px / max(info.height, info.width)
            img = cv2.resize(img,
                             (max(1, int(info.width * scale)),
                              max(1, int(info.height * scale))),
                             interpolation=cv2.INTER_NEAREST)

            ok, buf = cv2.imencode(".png", img)
            if not ok:
                return
            self.map_png = base64.b64encode(buf.tobytes()).decode("ascii")
            self.map_meta = {
                "origin_x": float(info.origin.position.x),
                "origin_y": float(info.origin.position.y),
                "width": int(info.width),
                "height": int(info.height),
                "resolution": float(info.resolution),
            }
            self.map_seq += 1
        except Exception as exc:  # noqa: BLE001
            self.last_error = repr(exc)

    def _on_amcl(self, msg):
        try:
            p = getattr(msg, "pose", None)
            if p is None:
                return
            p = getattr(p, "pose", p)
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose = {"x": float(p.position.x), "y": float(p.position.y),
                         "z": float(p.position.z), "yaw": float(yaw)}
            self.pose_stamp = time.time()
            # Re-anchor odom to this fix. Done here rather than on a timer: a
            # timer would latch the current odom every call and the delta
            # between the two would never be non-zero.
            self.odom_anchor = dict(self.odom_pose) if self.odom_pose else None
        except Exception as exc:  # noqa: BLE001
            self.last_error = repr(exc)

    def _on_odom(self, msg):
        try:
            p = msg.pose.pose
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.odom_pose = {"x": float(p.position.x), "y": float(p.position.y),
                              "z": float(p.position.z), "yaw": float(yaw)}
            self.odom_stamp = time.time()
        except Exception as exc:  # noqa: BLE001
            self.last_error = repr(exc)

    def _on_plan(self, msg):
        try:
            self.plan = [(float(p.pose.position.x), float(p.pose.position.y))
                         for p in msg.poses]
            self.plan_stamp = time.time()
        except Exception as exc:  # noqa: BLE001
            self.last_error = repr(exc)

    def _on_tf_static(self, msg):
        try:
            for t in msg.transforms:
                if not t.child_frame_id.endswith("utlidar_lidar"):
                    continue
                tr = t.transform.translation
                q = t.transform.rotation
                self.base_from_lidar = quaternion_matrix(
                    q.x, q.y, q.z, q.w, tr.x, tr.y, tr.z)
                _log(f"[{self.ns}] base_link -> {t.child_frame_id} via tf_static")
                return
        except Exception as exc:  # noqa: BLE001
            self.last_error = repr(exc)

    def _on_map_odom(self, msg):
        """SLAM's map <- odom correction (geometry_msgs/TransformStamped)."""
        try:
            tr = msg.transform.translation
            q = msg.transform.rotation
            self.map_from_odom = quaternion_matrix(
                q.x, q.y, q.z, q.w, tr.x, tr.y, tr.z)
            self.map_from_odom_stamp = time.time()
        except Exception as exc:  # noqa: BLE001
            self.last_error = repr(exc)

    # ----------------------------------------------------------- transforms
    def _odom_in_map(self, odom):
        """Lift an odom-frame pose into the map frame using SLAM's correction.

        Returns a map-frame pose dict, or the raw odom pose when the correction
        is missing (or odom has gone stale). This is what lines the cloud and
        the robot body up with the occupancy grid, which lives in 'map'.
        """
        if odom is None:
            return None
        fresh = (time.time() - self.odom_stamp) <= ODOM_MAX_AGE
        if self.map_from_odom is None or not fresh:
            return odom
        m = self.map_from_odom @ pose_matrix(
            odom["x"], odom["y"], odom["yaw"], odom.get("z", 0.0))
        return {
            "x": float(m[0, 3]),
            "y": float(m[1, 3]),
            "z": float(m[2, 3]),
            "yaw": math.atan2(float(m[1, 0]), float(m[0, 0])),
        }

    def map_pose(self):
        """Map-frame (x, y, yaw), or None.

        Order of preference:
          1. AMCL, carried forward with the odom delta captured at its last fix
             (accurate, and smooth between its slow updates).
          2. Odom lifted into the map frame by SLAM's map<-odom correction.
          3. Raw odom (unreliable frame, last resort before any map data).
        """
        amcl, odom = self.pose, self.odom_pose
        if amcl is None:
            return self._odom_in_map(odom)
        odom_fresh = (odom is not None
                      and (time.time() - self.odom_stamp) <= ODOM_MAX_AGE
                      and self.odom_anchor is not None)
        if not odom_fresh:
            return amcl
        return {
            "x": amcl["x"] + (odom["x"] - self.odom_anchor["x"]),
            "y": amcl["y"] + (odom["y"] - self.odom_anchor["y"]),
            # z comes straight from the AMCL fix rather than being integrated
            # from odom: AMCL's z is what the map is built against, and mixing
            # the two would drift the cloud away from the floor it sits on.
            "z": amcl.get("z", 0.0),
            "yaw": amcl["yaw"] + _wrap(odom["yaw"] - self.odom_anchor["yaw"]),
        }

    def map_from_lidar(self):
        """4x4 taking a lidar-frame point into the map frame, or None."""
        pose = self.map_pose()
        if pose is None:
            return None
        return pose_matrix(pose["x"], pose["y"], pose["yaw"],
                           pose.get("z", 0.0)) @ self.base_from_lidar

    # ------------------------------------------------------------- snapshot
    def dropped_no_pose_rate(self):
        """Points/s being discarded right now, or None when nothing is dropped.

        Averaged over the current no-pose streak. Reported alongside the raw
        counter because the counter alone is unreadable: for a robot that never
        gets a pose it just grows forever (astro reached ~39M).
        """
        if not self.dropped_no_pose or self._drop_streak_start is None:
            return None
        elapsed = time.time() - self._drop_streak_start
        if elapsed <= 0:
            return None
        return int(self.dropped_no_pose / elapsed)

    def snapshot(self, budget):
        xyz, rgb = self.buffer.sample(budget)
        # The graph-optimised map is shared, not per-robot; an absent layer
        # (unit tests) behaves as "not published yet".
        mm = self.modmap
        return {
            "xyz": xyz,
            "rgb": rgb,
            "pose": self.map_pose(),
            "map_png": self.map_png,
            "map_meta": self.map_meta,
            "map_seq": self.map_seq,
            "plan": self.plan,
            "plan_stamp": self.plan_stamp,
            "modmap_xyz": mm.xyz,
            "modmap_rgb": mm.rgb,
            "modmap_seq": mm.seq,
"stats": {
                "cloud_msgs": self.cloud_msgs,
                "buffered": len(self.buffer),
                "dropped_no_pose": self.dropped_no_pose,
                "dropped_no_pose_rate": self.dropped_no_pose_rate(),
                "last_cloud_age": None if not self.last_cloud_wall
                                  else time.time() - self.last_cloud_wall,
            },
        }

    def health(self):
        return {
            "cloud_msgs": self.cloud_msgs,
            "buffered": len(self.buffer),
            # Rolling window: the cloud shows only scans from the last
            # `window_secs`, so it stays registered to the live occupancy grid.
            "window_secs": round(self.buffer.window_secs, 2),
            "window_span_secs": round(self.buffer.span_secs, 2),
            "dropped_no_pose": self.dropped_no_pose,
            "dropped_no_pose_rate": self.dropped_no_pose_rate(),
            "last_cloud_age": None if not self.last_cloud_wall
                              else round(time.time() - self.last_cloud_wall, 2),
            "has_pose": self.map_pose() is not None,
            "pose_source": ("amcl" if self.pose is not None
                            else ("map_odom"
                                  if (self.map_from_odom is not None
                                      and self.odom_pose is not None)
                                  else ("odom" if self.odom_pose is not None
                                        else None))),
            "has_map": self.map_png is not None,
            "plan_points": len(self.plan),
            "last_error": self.last_error,
            **(self.modmap.health() if self.modmap is not None else {}),
        }


# ===========================================================================
# Bridge
# ===========================================================================
class Bridge:
    def __init__(self):
        self.robots = {}
        self.modmap = None
        self.mission = None
        self.error = None
        self.ready = threading.Event()
        self._node = None
        self._stop = threading.Event()
        self._spin_cycles = 0
        self._spin_logged = time.time()

    def stop(self):
        self._stop.set()

    def start(self):
        threading.Thread(target=self._run, name="foxglove-ros", daemon=True).start()

    def _run(self):
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node
        except Exception as exc:  # noqa: BLE001
            self.error = f"rclpy unavailable: {exc!r}"
            _log(self.error)
            self.ready.set()
            return

        try:
            rclpy.init(args=None)
        except Exception as exc:  # noqa: BLE001
            _log(f"rclpy.init reported: {exc!r}")

        node = Node("foxglove_viewer_node")
        # One shared graph-optimised map for both robots, subscribed before the
        # per-robot states so those can be handed a reference to it.
        try:
            self.modmap = ModifiedMapLayer(node)
            _log(f"subscribed to {self.modmap.topic} "
                 "(graph-optimised map, shared by all robots)")
        except Exception as exc:  # noqa: BLE001
            _log(f"modified_map subscription setup failed: {exc!r}")
        for ns in ROBOT_NAMESPACES:
            try:
                self.robots[ns] = RobotState(ns, node, self.modmap)
                _log(f"[{ns}] subscribed to {cloud_topic(ns)}")
            except Exception as exc:  # noqa: BLE001
                _log(f"[{ns}] subscription setup failed: {exc!r}")
        # Status only. Wrapped separately so a missing mission_manager on some
        # deployment degrades to "unknown" instead of taking the data path down.
        try:
            self.mission = MissionStateView(node)
            _log(f"subscribed to {self.mission.topic} (mission status, read-only)")
        except Exception as exc:  # noqa: BLE001
            _log(f"mission_state subscription setup failed: {exc!r}")
        self._node = node

        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        self.ready.set()
        _log("ROS bridge spinning")
        # Deliberately spin_once in a loop rather than ex.spin(). MultiThreaded
        # Executor.spin() blocks on the wait-set guard condition and can lose a
        # wakeup: the guard never fires again, every worker thread parks in
        # futex_wait and the node silently stops receiving -- while the process
        # stays alive and looks healthy. Reproduced here three times, stopping
        # after 476 / 98 / 241 messages while a plain spin_once subscriber on
        # the same topic ran indefinitely.
        #
        # spin_once re-arms the wait set on every pass, so a missed wakeup
        # costs at most one timeout instead of permanently. The heartbeat below
        # makes the loop observable through /health, since the failure it fixes
        # is otherwise indistinguishable from "the robot stopped publishing".
        interval = 0.2
        try:
            while not self._stop.is_set():
                ex.spin_once(timeout_sec=interval)
                self._spin_cycles += 1
                if time.time() - self._spin_logged > 30.0:
                    self._spin_logged = time.time()
                    # Never let diagnostics kill the data path: the heartbeat is
                    # observability, not function.
                    try:
                        self._log_spin_health()
                    except Exception as exc:  # noqa: BLE001
                        _log(f"heartbeat failed (ignored): {exc!r}")
        except Exception as exc:  # noqa: BLE001
            self.error = f"executor stopped: {exc!r}"
            _log(self.error)

    def _log_spin_health(self):
        parts = []
        for ns, robot in self.robots.items():
            age = (time.time() - robot.last_cloud_wall
                   if robot.last_cloud_wall else None)
            parts.append(f"{ns}: {robot.cloud_msgs} clouds"
                         + (f" last {age:.1f}s ago" if age is not None else " no clouds"))
        _log(f"[heartbeat] spin cycles {self._spin_cycles} | " + " | ".join(parts))

    def robot(self, ns):
        return self.robots.get(ns)

    def health(self):
        return {
            "ok": True,
            "error": self.error,
            "port": PORT,
            "max_points_per_frame": MAX_POINTS_PER_FRAME,
            "accum_max": ACCUM_MAX_POINTS,
            "target_fps": TARGET_FPS,
            "spin_cycles": self._spin_cycles,
            # Global (one publisher, one map) so it is reported once at the top
            # level rather than repeated under every robot.
            "modified_map": (self.modmap.health()
                             if self.modmap is not None else None),
            # Global too: one mission manager for the whole fleet.
            "mission": (self.mission.health()
                        if self.mission is not None else None),
            "robots": {ns: st.health() for ns, st in self.robots.items()},
        }


# ===========================================================================
# Wire encoding
# ===========================================================================
def encode_cloud_frame(xyz, rgb, pose):
    """One streaming frame.

    24-byte header then N x 3 float32 positions then N x 3 uint8 colours.
    Binary rather than JSON because base64 would add a third to every byte and
    this link is sometimes a tunnel over 5G.
    """
    n = 0 if xyz is None else int(xyz.shape[0])
    flags = 1 if pose is not None else 0
    if pose:
        px, py, pz, pyaw = pose["x"], pose["y"], pose.get("z", 0.0), pose["yaw"]
    else:
        px, py, pz, pyaw = 0.0, 0.0, 0.0, 0.0
    header = struct.pack(_HEADER_FMT, MSG_CLOUD, flags, 0, n, px, py, pz, pyaw)
    if n == 0:
        return header
    return header + xyz.astype("<f4", copy=False).tobytes() + rgb.tobytes()


def encode_path_frame(plan):
    """Nav2 path as its own frame type; same interleaved float32 layout."""
    n = min(len(plan), PATH_MAX_POINTS)
    if n == 0:
        return None
    arr = np.empty((n, 3), dtype="<f4")
    for i, (x, y) in enumerate(plan[:n]):
        # Same map-frame ROS -> three.js conversion as the cloud.
        arr[i, 0] = x
        arr[i, 1] = 0.05
        arr[i, 2] = -y
    return struct.pack(_HEADER_FMT, MSG_PATH, 0, 0, n, 0, 0, 0, 0) + arr.tobytes()


def encode_map_cloud_frame(xyz, rgb):
    """The static modified-map layer, same layout as the cloud frame.

    Reuses the cloud's header and interleaved layout rather than inventing a
    second shape, so the browser decodes both with one code path. The pose
    fields stay zero and flag bit0 is clear: this layer is fixed in the map
    frame and has no robot of its own, and leaving the pose out stops the
    viewer from mistaking it for a fresh robot reading.
    """
    if xyz is None or xyz.shape[0] == 0:
        return None
    n = int(xyz.shape[0])
    header = struct.pack(_HEADER_FMT, MSG_MAP_CLOUD, 0, 0, n, 0, 0, 0, 0)
    return header + xyz.astype("<f4", copy=False).tobytes() + rgb.tobytes()


# ===========================================================================
# HTTP + WebSocket on one port
# ===========================================================================
_STATIC_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}


def _read_static(rel):
    """Read a file under web_frontend/, or (None, None) if that is not possible."""
    clean = os.path.normpath("/" + rel.lstrip("/")).lstrip("/")
    path = os.path.normpath(os.path.join(STATIC_ROOT, clean))
    if path != STATIC_ROOT and not path.startswith(STATIC_ROOT + os.sep):
        return None, None
    if not os.path.isfile(path):
        return None, None
    with open(path, "rb") as fh:
        body = fh.read()
    ext = os.path.splitext(path)[1].lower()
    return body, _STATIC_TYPES.get(ext, "application/octet-stream")


def _http_response(status, body, ctype):
    """Build a plain HTTP response for the handshake interceptor.

    websockets' own ``connection.respond(status, text)`` only produces UTF-8
    text with no Content-Type, which is useless for a 1.3 MB JavaScript module
    and an image. ``Response`` is the same object that hook returns internally,
    so returning one directly is the supported way to serve arbitrary bytes.
    """
    from websockets.datastructures import Headers
    from websockets.http11 import Response

    reason = {200: "OK", 404: "Not Found", 500: "Internal Server Error",
              503: "Service Unavailable"}.get(status, "OK")
    headers = Headers()
    headers["Content-Type"] = ctype
    headers["Content-Length"] = str(len(body))
    headers["Connection"] = "close"
    headers["Cache-Control"] = "no-store"
    return Response(status, reason, headers, body)


def make_process_request(bridge):
    """websockets process_request hook: viewer page, static assets, /health.

    HTTP and the WebSocket share one port, so the iframe URL and the socket URL
    differ only by path and the client never has to know the port twice. This
    hook is what makes that possible: it runs on every inbound request and gets
    to answer the non-WebSocket ones itself.
    """

    async def process_request(connection, request):
        path = request.path.split("?", 1)[0]
        # Returning None lets the request continue to the opening handshake.
        # Anything not claimed here must fall through, or the socket is dead:
        # the handshake only happens for requests this hook declines to answer.
        if path == WS_PATH:
            return None
        if path == "/health":
            return _http_response(200, json.dumps(bridge.health(),
                                                  indent=2).encode(),
                                  "application/json")
        if path in ("/", "/index.html", "/foxglove"):
            body, ctype = _read_static("foxglove_view.html")
            if body is None:
                return _http_response(
                    500, b"foxglove_view.html missing from web_frontend/",
                    "text/plain; charset=utf-8")
            return _http_response(200, body, ctype)
        body, ctype = _read_static(path)
        if body is None:
            return _http_response(404, b"not found", "text/plain; charset=utf-8")
        return _http_response(200, body, ctype)

    return process_request


async def client_handler(ws, bridge):
    """Stream one robot's cloud to one browser tab.

    The robot is a plain local, not a bound argument, so the client can switch
    robots with a subscribe message instead of reconnecting -- and the query
    string in the socket URL is only a hint for the first frame.
    """
    loop = asyncio.get_running_loop()
    ns = "luna"
    interval = 1.0 / max(0.5, TARGET_FPS)
    stop = asyncio.Event()
    sent_map_seq = {}
    sent_modmap_seq = {}
    sent_plan_stamp = {}

    async def pump():
        nonlocal ns
        announced = None
        # None means "nothing sent yet", so the first real state is always
        # pushed even if it happens to be the first key we see.
        sent_mission = None
        last_mission_push = 0.0
        # Grace period before reporting "no mission manager at all" rather than
        # waiting silently; measured per connection.
        pump_started = time.time()
        while not stop.is_set():
            t0 = time.time()
            try:
                state = bridge.robot(ns)
                if state is None:
                    await ws.send(json.dumps({
                        "type": "error",
                        "message": f"robot '{ns}' is not available",
                    }))
                    await asyncio.sleep(interval)
                    continue

                snap = await loop.run_in_executor(None, state.snapshot,
                                                  MAX_POINTS_PER_FRAME)
                await ws.send(encode_cloud_frame(snap["xyz"], snap["rgb"],
                                                 snap["pose"]))

                # The map is latched, so send it once per connection (and again
                # only if the relay republished) rather than every frame.
                if snap["map_png"] and snap["map_meta"] \
                        and sent_map_seq.get(ns) != snap["map_seq"]:
                    sent_map_seq[ns] = snap["map_seq"]
                    await ws.send(json.dumps({"type": "map",
                                              "png": snap["map_png"],
                                              **snap["map_meta"]}))

                # The graph-optimised map only changes when SLAM re-optimises,
                # and it is hundreds of thousands of points, so gate it on the
                # sequence number -- once per connection, then only on a
                # genuine re-optimisation.
                if snap["modmap_xyz"] is not None \
                        and sent_modmap_seq.get(ns) != snap["modmap_seq"]:
                    sent_modmap_seq[ns] = snap["modmap_seq"]
                    frame = encode_map_cloud_frame(snap["modmap_xyz"],
                                                   snap["modmap_rgb"])
                    if frame:
                        await ws.send(frame)

                # The plan only changes when Nav2 replans, so gate on its stamp.
                if snap["plan"] and sent_plan_stamp.get(ns) != snap["plan_stamp"]:
                    sent_plan_stamp[ns] = snap["plan_stamp"]
                    frame = encode_path_frame(snap["plan"])
                    if frame:
                        await ws.send(frame)

                # Mission state republishes at ~5 Hz whether or not it moved. Gate on the
                # value so the socket carries one small JSON frame per actual
                # transition instead of one per tick.
                #
                # bridge.mission is read here, per iteration, rather than captured
                # into a local when this closure is built. make_process_request runs
                # once at websockets.serve() time, which is before bridge.start() has
                # finished its thread -- capturing it then would latch None forever
                # and the status would silently never appear.
                mission = bridge.mission
                now_m = time.time()
                # Two cases must reach the browser:
                #   - we have a state: report it, and report staleness once it
                #     ages out;
                #   - we have none at all, because the mission manager was never
                #     up or has already died. Waiting for a first message before
                #     saying anything leaves the HUD on "connecting..." forever,
                #     which reads as slow start-up rather than a missing
                #     publisher. So once the grace period passes, report the
                #     absence explicitly.
                have = mission is not None and mission.state is not None
                grace_over = now_m - pump_started >= MISSION_STALE_SECONDS
                if have or grace_over:
                    age = (None if mission is None or not mission.last_wall
                           else now_m - mission.last_wall)
                    stale = age is None or age > MISSION_STALE_SECONDS
                    key = None if mission is None else mission.snapshot()
                    # Push on a real transition, or on the slow keepalive tick
                    # once stale so the age keeps counting. Pure value-gating
                    # would go silent forever when the publisher dies, which is
                    # exactly the case where the operator most needs to be told.
                    if (key, stale) != sent_mission \
                            or now_m - last_mission_push >= MISSION_PUSH_INTERVAL:
                        sent_mission = (key, stale)
                        last_mission_push = now_m
                        await ws.send(json.dumps({
                            "type": "mission",
                            "state": None if key is None else key[0],
                            "stale": stale,
                            "age": None if age is None else round(age, 1),
                            "explorer_robot": None if mission is None
                                              else mission.explorer_robot,
                            "completion_status": None if mission is None
                                                else mission.completion_status,
                            "known_states": list(MISSION_STATES),
                        }))

                # Say something rather than going quiet, so the HUD can tell
                # "no data yet" apart from "connection frozen".
                stats = snap["stats"]
                if snap["xyz"] is None or stats["dropped_no_pose"]:
                    if stats["dropped_no_pose"]:
                        note = "waiting for robot pose (amcl/odom) before cloud can be placed..."
                    elif announced != "cloud":
                        note = f"waiting for {cloud_topic(ns)} ..."
                    else:
                        note = None
                    if note:
                        announced = "cloud"
                        await ws.send(json.dumps({"type": "status",
                                                  "message": note}))
                else:
                    announced = None
            except Exception:
                break
            await asyncio.sleep(max(0.0, interval - (time.time() - t0)))

    pump_task = asyncio.create_task(pump())
    try:
        await ws.send(json.dumps({"type": "status",
                                  "message": "connected"}))
        await ws.send(json.dumps({"type": "topic",
                                  "topic": cloud_topic(ns),
                                  "modmap_topic": modified_map_topic(ns)}))
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            if msg.get("type") != "subscribe":
                continue
            wanted = str(msg.get("robot", "")).strip().lower()
            if wanted in ROBOT_NAMESPACES:
                ns = wanted
                _log(f"client subscribed to {ns}")
                await ws.send(json.dumps({"type": "topic",
                                          "topic": cloud_topic(ns),
                                          "modmap_topic": modified_map_topic(ns)}))
            elif wanted:
                await ws.send(json.dumps({
                    "type": "error",
                    "message": f"unknown robot '{msg.get('robot')}'",
                    "detail": "known robots: " + ", ".join(ROBOT_LABELS[n] for n in ROBOT_NAMESPACES),
                }))
    finally:
        stop.set()
        pump_task.cancel()


async def amain(bridge):
    import websockets

    loop = asyncio.get_running_loop()
    stop = loop.create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(
                sig, lambda: stop.done() or stop.set_result(None))
        except NotImplementedError:
            pass

    async with websockets.serve(
        lambda ws: client_handler(ws, bridge),
        "0.0.0.0", PORT,
        process_request=make_process_request(bridge),
        max_size=8 * 1024 * 1024,
        ping_interval=20,
        ping_timeout=60,
    ):
        _log(f"viewer page  http://0.0.0.0:{PORT}/")
        _log(f"websocket     ws://0.0.0.0:{PORT}{WS_PATH}")
        _log(f"frame budget  {MAX_POINTS_PER_FRAME} pts @ {TARGET_FPS} fps"
             f", buffer {ACCUM_MAX_POINTS} pts")
        try:
            await stop
        finally:
            pass


def main():
    try:
        import websockets  # noqa: F401
    except ImportError:
        _log("ERROR: this viewer needs the 'websockets' package")
        return 1

    bridge = Bridge()
    bridge.start()
    bridge.ready.wait(timeout=20)

    try:
        asyncio.run(amain(bridge))
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        _log(f"ERROR: cannot serve on port {PORT}: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())