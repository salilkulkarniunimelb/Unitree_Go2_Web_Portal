"""
lab_portal.py - Resilient live dashboard for the QOD lab Go2.

Runs inside the robot_hivemind ROS 2 (Humble) container and subscribes to the
lab's real topics (bridged from the Go2 over Zenoh / domain 70):

    MAP     /lidar_slam_2d_map            (nav_msgs/msg/OccupancyGrid)
    POSE    /go2/stabilized/current_pose  (geometry_msgs/msg/PoseStamped)
    ODOM    /go2/restamped/robot_odom     (nav_msgs/msg/Odometry)
    CAMERA  /frontvideostream             (sensor_msgs/msg/Image)
    BATTERY /lf/lowstate                  (unitree_go/msg/LowState)

The page ALWAYS loads. Every section shows a "Waiting for ..." placeholder
until its topic starts publishing, so the robot being offline never crashes
or blank-screens the dashboard. Click the map to set a nav goal (/goal_pose).
Toggle "Set Initial Pose" and drag on the map to publish an initial pose
(/initialpose) for the selected robot, like Foxglove's "2D Publish Pose".

Usage:
    source /opt/ros/humble/setup.bash
    python3 lab_portal.py            # serves on 0.0.0.0:7860
"""

import math
import queue
import threading
import time
import base64
from collections import deque

import cv2
import numpy as np

import gradio as gr

# ----------------------- Topic names (lab-real) -----------------------
# Topics actually publishing on the lab server (verified via ros2 topic list).
# Each robot publishes under its own namespace (/luna/, /astro/).
# - Map and camera are the live streams.
# - Pose is shared on the unprefixed /go2/stabilized/current_pose topic.
# - No battery topic exists for either robot yet.
LUNA_TOPICS = {
    "map":     "/luna/lidar_slam_2d_map",
    "pose":    "/go2/stabilized/current_pose",
    "odom":    "/luna/go2/restamped/robot_odom",
    "camera":  "/luna/frontvideostream",
    "battery": "/lf/lowstate",
    "goal":    "/luna/goal_pose",
    "initialpose": "/luna/initialpose",
}

ASTRO_TOPICS = {
    "map":     "/astro/lidar_slam_2d_map",
    "pose":    "/go2/stabilized/current_pose",
    "odom":    "/astro/go2/restamped/robot_odom",
    "camera":  "/astro/frontvideostream",
    "battery": "/lf/lowstate",
    "goal":    "/astro/goal_pose",
    "initialpose": "/astro/initialpose",
}

ROBOTS = {
    "Luna": LUNA_TOPICS,
    "Astro": ASTRO_TOPICS,
}

# Luna's camera is H.264-encoded Go2FrontVideoData. We decode it with a
# persistent ffmpeg pipeline (started once per robot) fed via stdin and read
# Camera frames arrive as raw H.264 Go2FrontVideoData, fragmented across many
# small ROS messages. A persistent ffmpeg pipe cannot stay in sync on this
# stream, so we buffer bytes and decode the newest SPS-keyframe window from a
# file on a background thread. ffmpeg must be installed in the container.
CAM_W, CAM_H, CAM_BUFFER_MAX = 640, 360, 6 * 1024 * 1024
# Ignore the buffer until it holds at least this much data (enough for a
# keyframe sequence) -- avoids decoding a nearly-empty, undecodable buffer.
CAM_MIN_BYTES = 512 * 1024
# Background decode cadence (s). Governs how often a new batch is decoded.
CAM_DECODE_INTERVAL = 0.4
# Max frames to cache for smooth playback (one shot of motion per refill).
CAM_FRAMES_MAX = 20
# Cap the decoded segment to this many bytes -- we only need the most recent
# keyframe onward, not the whole (up to several-MB) rolling buffer.
# Sized generously so a full GOP (which may include one keyframe + many P-frames)
# fits in the window even as the buffer scrolls, keeping the camera continuous.
CAM_DECODE_WINDOW = 4 * 1024 * 1024

FT_FRAME = "lidar_map"

SERV_NAME = "0.0.0.0"
SERV_PORT = 7860

# ----------------------- import rclpy + msgs (defensive) -------------
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - cv_bridge usually present in Humble
    CvBridge = None

try:
    from unitree_go.msg import LowState
except Exception:
    LowState = None

try:
    from unitree_go.msg import Go2FrontVideoData
except Exception:
    Go2FrontVideoData = None


def _placeholder(w, h, text, color=(56, 189, 248)):
    """Dark placeholder image with centred text."""
    ph = np.ones((h, w, 3), dtype=np.uint8) * 12
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(text, font, 0.6, 2)
    cv2.putText(ph, text, ((w - tw) // 2, (h + th) // 2), font, 0.6, color, 2)
    return ph


_LOGO_URI = None


def logo_data_uri(path="assets/logo.png"):
    """Return the logo as a base64 data URI so it renders regardless of how
    Gradio serves static files (works over SSH tunnels / containers).
    Returns "" if the file is missing so the page never crashes."""
    global _LOGO_URI
    if _LOGO_URI is None:
        try:
            with open(path, "rb") as f:
                b64 = base64.b64encode(f.read()).decode()
            _LOGO_URI = f"data:image/png;base64,{b64}"
        except FileNotFoundError:
            _LOGO_URI = ""
    return _LOGO_URI


DASHBOARD_CSS = """
    html, body, .gradio-container, .gradio-container * {
        font-family: Helvetica, 'Helvetica Neue', Arial, sans-serif !important;
    }
    .portal-header{display:flex;align-items:center;gap:14px;
        padding:10px 12px;border-radius:12px;
        background:linear-gradient(90deg,#0f2450,#000f46);
        color:#fff;margin-bottom:4px;}
    .portal-header img.logo{height:44px;width:auto;border-radius:8px;
        box-shadow:0 2px 8px rgba(0,0,0,.35);}
    .portal-header .title{font-size:20px;font-weight:700;letter-spacing:.3px;}
    .portal-header .subtitle{font-size:13px;opacity:.85;margin-top:2px;}
    .robot-selector{margin-top:10px;}
    #initpose_bridge{pointer-events:none;opacity:0;height:0;overflow:hidden;}
    #initpose_bridge textarea{opacity:0;height:0;min-height:0!important;}
"""

# Foxglove-style click-and-drag to set the initial pose. The drag endpoints are
# captured (in image-pixel coords) and forwarded to a Python handler through a
# small hidden Textbox bridge (initpose_bridge) by dispatching a DOM 'input'
# event, which Gradio's `.input` listener picks up. When the "Set Initial Pose"
# toggle is off, the backend ignores the payload, so goal-setting is unaffected.
INIT_POSE_JS = """<script>
(function () {
    var ACTIVE = true;
    var dragging = false;
    var sX = 0, sY = 0, sPx = 0, sPy = 0;
    var svg = null, svgLine = null, svgHead = null, svgDot = null;

    // Gradio mounts its UI inside a shadow root under <gradio-app>. Resolve an
    // element against that shadow root (falling back to the light DOM).
    function resolveEl(sel) {
        var root = document.querySelector('gradio-app');
        if (root && root.shadowRoot) {
            var el = root.shadowRoot.querySelector(sel);
            if (el) return el;
        }
        return document.querySelector(sel);
    }

    function mapImg() {
        var img = resolveEl('#map_image');
        if (!img) return null;
        return img.tagName === 'IMG' ? img : img.querySelector('img');
    }

    function inImg(e, img) {
        var r = img.getBoundingClientRect();
        return e.clientX >= r.left && e.clientX <= r.right &&
               e.clientY >= r.top  && e.clientY <= r.bottom;
    }

    function toImgPts(e, img) {
        var r = img.getBoundingClientRect();
        if (!r.width || !r.height) return [0, 0];
        var x = Math.round((e.clientX - r.left) / r.width  * (img.naturalWidth  || 1));
        var y = Math.round((e.clientY - r.top)  / r.height * (img.naturalHeight || 1));
        return [x, y];
    }

    // A full-viewport SVG overlay that draws a Foxglove-style pose arrow while
    // dragging: a dot at the press point + a line with an arrowhead at the tip.
    function makeArrow() {
        if (svg) return svg;
        var NS = 'http://www.w3.org/2000/svg';
        svg = document.createElementNS(NS, 'svg');
        svg.setAttribute('style',
            'position:fixed;left:0;top:0;width:100vw;height:100vh;' +
            'z-index:99999;pointer-events:none;overflow:visible;');
        svgDot = document.createElementNS(NS, 'circle');
        svgDot.setAttribute('r', '7');
        svgDot.setAttribute('fill', 'rgba(34,197,94,.95)');
        svgDot.setAttribute('stroke', 'rgba(0,80,40,.6)');
        svgDot.setAttribute('stroke-width', '2');
        svgLine = document.createElementNS(NS, 'line');
        svgLine.setAttribute('stroke', 'rgba(34,197,94,.95)');
        svgLine.setAttribute('stroke-width', '3.5');
        svgLine.setAttribute('stroke-linecap', 'round');
        svgHead = document.createElementNS(NS, 'polygon');
        svgHead.setAttribute('fill', 'rgba(34,197,94,.95)');
        svgHead.setAttribute('stroke', 'rgba(0,80,40,.6)');
        svgHead.setAttribute('stroke-width', '1');
        svg.appendChild(svgDot);
        svg.appendChild(svgLine);
        svg.appendChild(svgHead);
        document.body.appendChild(svg);
        return svg;
    }

    function drawArrow(e) {
        var dx = e.clientX - sX, dy = e.clientY - sY;
        var len = Math.hypot(dx, dy);
        var ang = Math.atan2(dy, dx);
        svgDot.setAttribute('cx', sX);
        svgDot.setAttribute('cy', sY);
        svgLine.setAttribute('x1', sX);
        svgLine.setAttribute('y1', sY);
        // If the drag is essentially a click (too short), just show the dot.
        if (len < 4) {
            svgLine.setAttribute('x2', sX);
            svgLine.setAttribute('y2', sY);
            svgHead.setAttribute('points', sX + ',' + sY + ' ' + sX + ',' + sY + ' ' + sX + ',' + sY);
            return;
        }
        var tipX = e.clientX, tipY = e.clientY;
        svgLine.setAttribute('x2', tipX);
        svgLine.setAttribute('y2', tipY);
        // arrowhead: a triangle whose base is behind the tip, perp to the line
        var hs = 15, hw = 7.5;
        var bx = tipX - hs * Math.cos(ang), by = tipY - hs * Math.sin(ang);
        var cp = Math.PI / 2;
        var q1x = bx + hw * Math.cos(ang + cp), q1y = by + hw * Math.sin(ang + cp);
        var q2x = bx + hw * Math.cos(ang - cp), q2y = by + hw * Math.sin(ang - cp);
        svgHead.setAttribute('points',
            tipX + ',' + tipY + ' ' + q1x + ',' + q1y + ' ' + q2x + ',' + q2y);
    }

    document.addEventListener('pointerdown', function (e) {
        var img = mapImg();
        if (!img || !ACTIVE || !inImg(e, img)) return;
        // Only draw the pose arrow when the "Set Initial Pose" toggle is ON.
        var toggle = resolveEl('#initpose_toggle input');
        if (!toggle || !toggle.checked) return;
        var p = toImgPts(e, img);
        sPx = p[0]; sPy = p[1];
        sX = e.clientX; sY = e.clientY;
        dragging = true;
        makeArrow();
        svg.style.display = '';
        drawArrow(e);
    });

    document.addEventListener('pointermove', function (e) {
        if (!dragging || !svg) return;
        drawArrow(e);
    });

    document.addEventListener('pointerup', function (e) {
        if (!dragging) return;
        dragging = false;
        if (svg) svg.style.display = 'none';
        var img = mapImg();
        if (!img || !inImg(e, img)) return;
        var p2 = toImgPts(e, img);
        var payload = sPx + ',' + sPy + ',' + p2[0] + ',' + p2[1];
        // Only publish when the "Set Initial Pose" toggle is checked.
        var toggle = resolveEl('#initpose_toggle input');
        var mode = toggle ? !!toggle.checked : false;
        // Deterministic bridge: POST the drag directly to the Gradio API endpoint
        // instead of depending on Gradio's component-event dispatch semantics.
        try {
            fetch('/gradio_api/call/drag_initial', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ data: [payload, mode] })
            }).catch(function () {});
        } catch (err) {}
    });
})();
</script>"""


class RobotState:
    """Per-robot data + callbacks + rendering.

    One instance per robot (Luna, Astro). Each owns its own topic names and
    live data, so the dashboard can switch between robots without ever
    tearing down or recreating subscriptions.
    """

    def __init__(self, node, name, topics):
        self.node = node
        self.name = name
        self.topics = topics
        self._bridge = CvBridge() if CvBridge else None

        # --- map ---
        self.map_info = None
        self.map = None
        self.cached_map_img = None
        self.cached_scale = None
        self.cached_meta = None
        self.map_stamp = 0.0
        # --- pose / odom ---
        self.robot_pose = None
        self.yaw = 0.0
        self.odom_path = deque(maxlen=2000)
        self.pose_latest = "Waiting for pose..."
        self.last_goal = None
        # --- initial pose (set via map drag) ---
        self.initial_pose = None          # (wx, wy, yaw) pending/confirmed initial pose
        self.last_initial = None
        # --- camera ---
        self.color_frame = None
        self.cam_stamp = 0.0
        self.cam_frame_ts = 0.0
        self._cam_buf = bytearray()
        self._cam_lock = threading.Lock()
        self._cam_frames = deque(maxlen=CAM_FRAMES_MAX)  # smooth-playback buffer
        self._last_decode = 0.0
        self._stop_cam = False
        # Camera decode strategy: the Go2 H.264 stream is fragmented across many
        # small ROS messages, so a persistent ffmpeg pipe can never stay in sync
        # (constant "no frame!" / "non-existing PPS"). Instead we buffer bytes,
        # trim the tail to the latest SPS keyframe sequence, and decode that
        # window from a file on a background thread. Reliable, keeps the icon
        # green, and delivers a live multi-fps camera.
        threading.Thread(target=self._camera_decode_loop, daemon=True).start()
        # --- battery / imu ---
        self.battery = None
        self.voltage = None
        self.current = None
        self.power = None
        self.roll = None
        self.pitch = None
        self.imu_yaw = None
        self.motor_status = None
        self.bat_stamp = 0.0

        self._goal_queue = queue.Queue()
        self._init_queue = queue.Queue()

        # Camera playback FPS diagnostic (rough): count frames popped vs wall time.
        self._cam_played = 0
        self._cam_played_ts = time.time()
        self.cam_fps = 0.0

        # Cached H.264 parameter sets (SPS + PPS). The Go2 stream only emits these
        # once at startup, then continuous P-frames. The rolling buffer eventually
        # scrolls past the SPS, leaving only reference P-frames that cannot decode
        # on their own -> the camera "works a few moments then goes stale". We cache
        # the SPS/PPS and prepend them to every decode window so the stream stays
        # decodable forever.
        self._sps = b""
        self._pps = b""

    # ---------------------- callbacks ----------------------
    def map_cb(self, msg):
        self.map_info = msg.info
        mp = np.array(msg.data, dtype=np.float32).reshape(msg.info.height, msg.info.width)
        self.map = np.flipud(mp)
        h, w = self.map.shape
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        res = msg.info.resolution

        disp = np.ones((h, w), dtype=np.float32)
        disp[self.map == -1] = 0.5
        disp[self.map == 0] = 1.0
        disp[self.map > 0] = 0.0
        gray = (disp * 255).astype(np.uint8)
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        canvas = 800
        scale = canvas / max(h, w)
        img = cv2.resize(img, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_NEAREST)
        out = np.ones((canvas, canvas, 3), dtype=np.uint8) * 255
        ox = (canvas - img.shape[1]) // 2
        oy = (canvas - img.shape[0]) // 2
        out[oy:oy + img.shape[0], ox:ox + img.shape[1]] = img

        self.cached_map_img = out
        self.cached_scale = scale
        self.cached_meta = (origin_x, origin_y, h, ox, oy, res)
        self.map_stamp = self.node.get_clock().now().nanoseconds / 1e9

    def pose_cb(self, msg):
        self.robot_pose = msg.pose
        q = self.robot_pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose_latest = (
            f"x: {self.robot_pose.position.x:.3f}   "
            f"y: {self.robot_pose.position.y:.3f}   "
            f"yaw: {self.yaw:.3f} rad"
        )

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.odom_path.append(
            (p.position.x, p.position.y)
        )
        # The dedicated pose topic (/go2/stabilized/current_pose) has no
        # publisher on this server, so derive pose from odometry instead.
        q = p.orientation
        try:
            self.robot_pose = p
            self.yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self.pose_latest = (
                f"x: {p.position.x:.3f}   "
                f"y: {p.position.y:.3f}   "
                f"yaw: {self.yaw:.3f} rad"
            )
        except Exception:
            pass

    @staticmethod
    def _nal_units(raw):
        """Yield (offset, start_code_size) for each Annex-B start code in raw."""
        i = 0
        n = len(raw) - 4
        while i < n:
            if raw[i] == 0 and raw[i + 1] == 0:
                if raw[i + 2] == 1:
                    yield i, 3
                    i += 3
                elif raw[i + 2] == 0 and raw[i + 3] == 1:
                    yield i, 4
                    i += 4
                else:
                    i += 1
            else:
                i += 1

    def _capture_headers(self, raw):
        """Cache any SPS(7)/PPS(8) NAL units found in raw (they are rare, so we
        only re-scan for them when they appear near a keyframe / on startup)."""
        units = list(self._nal_units(raw))
        for k, (pos, sz) in enumerate(units):
            hdr = pos + sz
            if hdr >= len(raw):
                continue
            nal_type = raw[hdr] & 0x1F
            end = units[k + 1][0] if k + 1 < len(units) else len(raw)
            if nal_type == 7:
                self._sps = raw[pos:end]
            elif nal_type == 8:
                self._pps = raw[pos:end]

    def camera_cb(self, msg):
        """Append H.264 bytes from Go2FrontVideoData to the rolling buffer and
        cache SPS/PPS so the stream stays decodable as the buffer scrolls."""
        try:
            data = bytes(msg.data)
            with self._cam_lock:
                self._cam_buf.extend(data)
                # keep a bounded trailing window (enough for a full IDR+frames)
                if len(self._cam_buf) > CAM_BUFFER_MAX:
                    del self._cam_buf[: len(self._cam_buf) - CAM_BUFFER_MAX]
                # Occasionally scan for SPS/PPS (cheap: only the newest bytes).
                if len(self._cam_buf) < 1024 * 1024 or len(self._cam_buf) % (512 * 1024) < len(data):
                    self._capture_headers(data)
            self.cam_stamp = self.node.get_clock().now().nanoseconds / 1e9
        except Exception as e:
            self.node.get_logger().error(f"Camera buffer failed: {e}")

    @staticmethod
    def _trim_to_keyframe(raw):
        """Trim a raw H.264 Annex-B buffer so it starts at the most recent
        COMPLETE SPS->PPS->IDR sequence (NAL types 7 -> 8 -> 5) and ends just
        before the NEXT SPS. Starting at a bare SPS with no following PPS/IDR
        (or mid-GOP) is what produces the ffmpeg 'top block unavailable for
        requested intra mode' errors and a near-zero frame yield, which makes
        the camera look frozen. Scanning the whole buffer for the last clean
        GOP start gives the decoder a perfect SPS+PPS+IDR+P.. sequence to sync."""
        def start_codes(data):
            i = 0
            n = len(data) - 4
            pos = []
            while i < n:
                if data[i] == 0 and data[i + 1] == 0:
                    if data[i + 2] == 1:
                        pos.append((i, 3))
                        i += 3
                    elif data[i + 2] == 0 and data[i + 3] == 1:
                        pos.append((i, 4))
                        i += 4
                    else:
                        i += 1
                else:
                    i += 1
            return pos

        codes = start_codes(raw)
        if not codes:
            return None

        def nal_type_at(idx):
            pos, sz = codes[idx]
            hdr = pos + sz
            if hdr >= len(raw):
                return None
            return raw[hdr] & 0x1F

        # Find the newest REFERENCE frame (NAL 5 = IDR, or NAL 1 slice) to cut at.
        # We need a real I/mixed slice so the decoder has a reference to start.
        # Prefer an IDR(5); else any NAL-1 slice after the newest SPS->PPS pair.
        ref = None
        for i in range(len(codes)):
            t = nal_type_at(i)
            if t == 5:  # IDR - guaranteed clean reference
                ref = i
        if ref is None:
            # Fall back: last NAL-1 slice that could start a new frame.
            for i in range(len(codes)):
                if nal_type_at(i) == 1:
                    ref = i
        if ref is None:
            # Fall back to the newest SPS->PPS pair (stream may have no I slice yet)
            for i in range(len(codes) - 1):
                if nal_type_at(i) == 7 and nal_type_at(i + 1) == 8:
                    ref = i
        if ref is None:
            return None

        start = codes[ref][0]
        # Cut before the NEXT SPS (a new parameter-set = a new clean access unit),
        # so the returned segment is a single self-contained GOP.
        end = len(raw)
        for j in range(ref + 1, len(codes)):
            if nal_type_at(j) == 7:
                end = codes[j][0]
                break
        return raw[start:end]

    def _camera_decode_loop(self):
        """Background decode thread: repeatedly decode the newest H.264 keyframe
        segment and fill a buffer of frames. draw_camera() plays those frames at
        full speed for smooth video while this thread refills in the background."""
        while not self._stop_cam:
            try:
                if self.cam_stamp:
                    batch = self._decode_camera_batch()
                    if batch:
                        with self._cam_lock:
                            self._cam_frames.extend(batch)
                        self.cam_frame_ts = time.time()
                    else:
                        # Diagnostic: surface WHY no frames decoded (throttled).
                        now = time.time()
                        if now - getattr(self, "_diag_last", 0) > 5.0 and self.cam_fps < 0.5:
                            self._diag_last = now
                            with self._cam_lock:
                                buf = len(self._cam_buf)
                            raw = bytes(self._cam_buf[-CAM_DECODE_WINDOW:]) if buf else b""
                            trim = self._trim_to_keyframe(raw) if raw else None
                            self.node.get_logger().warning(
                                f"[{self.name}] cam decode empty: buf={buf}B "
                                f"window={len(raw)}B trim={'yes' if trim else 'NO-KEYFRAME'}"
                            )
            except Exception as e:
                self.node.get_logger().error(f"Camera decode failed: {e}")
            threading.Event().wait(CAM_DECODE_INTERVAL)

    def _decode_camera_batch(self):
        """Decode the newest bytes into a list of frames, staying continuous.

        Strategy: instead of requiring a keyframe to be present in the rolling
        window (which fails once the buffer scrolls past the one-time SPS/IDR,
        causing the camera to freeze permanently), we prepend the cached SPS+PPS
        parameter sets and, when available, split at the newest IDR/I-slice so the
        decoder always has a valid reference to start from.
        """
        if not self._cam_buf:
            return []
        with self._cam_lock:
            sps, pps = self._sps, self._pps
            raw = bytes(self._cam_buf[-CAM_DECODE_WINDOW:])
        if len(raw) < CAM_MIN_BYTES:
            return []
        if sps:
            raw = self._trim_to_keyframe(raw) or raw
            # Prepend the cached header so ffmpeg always has its parameter sets,
            # even after the keyframe has scrolled out of the rolling buffer.
            raw = sps + pps + raw
        else:
            raw = self._trim_to_keyframe(raw)
            if not raw:
                return []
        tmp = f"/tmp/{self.name}_cam.h264"
        with open(tmp, "wb") as f:
            f.write(raw)
        cap = cv2.VideoCapture(tmp)
        frames = []
        while len(frames) < CAM_FRAMES_MAX:
            ret, f = cap.read()
            if not ret:
                break
            h, w = f.shape[:2]
            if w > CAM_W or h > CAM_H:
                f = cv2.resize(f, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)
            frames.append(f)
        cap.release()
        return frames

    def decode_camera(self):
        """Return the latest decoded frame instantly (decode runs in a thread).
        Never blocks the UI: there is always a cached frame or the placeholder."""
        return self.color_frame

    def battery_cb(self, msg):
        self.battery = msg.bms_state.soc
        self.motor_status = [m.temperature for m in msg.motor_state if m.mode == 1]
        self.roll = msg.imu_state.rpy[0]
        self.pitch = msg.imu_state.rpy[1]
        self.imu_yaw = msg.imu_state.rpy[2]
        self.voltage = msg.power_v
        self.current = msg.power_a
        self.power = self.voltage * self.current
        self.bat_stamp = self.node.get_clock().now().nanoseconds / 1e9

    # ---------------------- rendering ----------------------
    def draw_map(self):
        if self.cached_map_img is None:
            return _placeholder(640, 480, f"WAITING FOR {self.name.upper()} MAP...")

        canvas = self.cached_map_img.copy()
        scale = self.cached_scale
        origin_x, origin_y, h, ox, oy, res = self.cached_meta

        def to_px(wx, wy):
            mx = (wx - origin_x) / res
            my = (wy - origin_y) / res
            my = h - 1 - my
            return int(mx * scale) + ox, int(my * scale) + oy

        if self.robot_pose is not None:
            px, py = to_px(self.robot_pose.position.x, self.robot_pose.position.y)
            cv2.circle(canvas, (px, py), 10, (255, 0, 0), -1)
            ex = int(px + 30 * math.cos(self.yaw))
            ey = int(py - 30 * math.sin(self.yaw))
            cv2.arrowedLine(canvas, (px, py), (ex, ey), (0, 0, 0), 3)

            # Pending / confirmed initial pose (green) so it is shown on the map
            if self.initial_pose is not None:
                ipx, ipy = to_px(self.initial_pose[0], self.initial_pose[1])
                cv2.circle(canvas, (ipx, ipy), 8, (34, 197, 94), -1)
                cv2.circle(canvas, (ipx, ipy), 10, (0, 120, 60), 2)
                iex = int(ipx + 28 * math.cos(self.initial_pose[2]))
                iey = int(ipy - 28 * math.sin(self.initial_pose[2]))
                cv2.arrowedLine(canvas, (ipx, ipy), (iex, iey), (0, 150, 0), 3)

            canvas = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)
            hc, wc = canvas.shape[:2]
            cv2.putText(canvas, "N", (wc // 2, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
            cv2.putText(canvas, "S", (wc // 2, hc - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
        return canvas

    def draw_camera(self):
        # Play smoothly from the decoded frame queue (refilled in the
        # background). When it runs dry, hold the last frame until the next
        # batch is decoded -- gives fluid motion instead of one stamp per tick.
        frame = None
        now = time.time()
        if now - self._cam_played_ts >= 1.0:
            self.cam_fps = self._cam_played / (now - self._cam_played_ts)
            self._cam_played = 0
            self._cam_played_ts = now
        with self._cam_lock:
            if self._cam_frames:
                frame = self._cam_frames.popleft()
                self.color_frame = frame
                self._cam_played += 1
        if frame is None:
            frame = self.color_frame
        if frame is None:
            return _placeholder(640, 360, f"WAITING FOR {self.name.upper()} CAMERA...")
        h, w = frame.shape[:2]
        if w > 640 or h > 360:
            frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
        return frame

    # ---------------------- text getters ----------------------
    def get_battery_data(self):
        if self.battery is None:
            return f"Waiting for {self.name} battery..."
        if self.battery >= 70:
            col = "🟢"
        elif self.battery >= 30:
            col = "🟡"
        elif self.battery >= 20:
            col = "🔴"
        else:
            col = "🚨"
        text = f"🔋 {col} {self.battery} %"
        if self.power is not None:
            text += f"  🔌 {self.power:.1f} W ({self.voltage:.1f}V, {self.current:.1f}A)"
        return text

    def get_motor_data(self):
        if self.motor_status is None:
            return f"Waiting for {self.name} motor temps..."
        legs = ["FL", "FR", "RL", "RR"]
        out = []
        for i, leg in enumerate(legs):
            base = i * 3
            out.append(
                f"{leg} hip {self.motor_status[base]:.0f}°  "
                f"thigh {self.motor_status[base+1]:.0f}°  "
                f"calf {self.motor_status[base+2]:.0f}°"
            )
        return "\n".join(out)

    def get_orientation_data(self):
        if self.roll is None:
            return f"Waiting for {self.name} IMU..."
        text = (f"Roll {self.roll:.3f}   Pitch {self.pitch:.3f}   "
                f"Yaw {self.imu_yaw:.3f} rad")
        if abs(self.roll) > 0.5 or abs(self.pitch) > 0.5:
            text += "\n🚨 Robot UNSTABLE!"
        return text

    def get_pose_data(self):
        pos = self.pose_latest
        if self.last_goal:
            return (f"{pos}\n🎯 last goal: "
                    f"({self.last_goal[0]:.2f}, {self.last_goal[1]:.2f})")
        return pos

    def get_connection_status(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        parts = []
        for name, ts in (
            ("MAP", self.map_stamp), ("CAM", self.cam_stamp),
            ("BAT", self.bat_stamp),
        ):
            if ts > 0 and now - ts < 5.0:
                parts.append(f"🟢 {name}")
            elif ts > 0:
                parts.append(f"🟡 {name} (stale)")
            else:
                parts.append(f"⚪ {name} (waiting)")
        cam = f"camera {self.cam_fps:.1f} fps" if self.cam_fps > 0.5 else "camera no-frames"
        return "   ".join(parts) + f"   ·  {cam}"

    # ---------------------- initial pose ----------------------
    def _display_to_world(self, x, y):
        """Convert displayed-map image pixel coords (x = horizontal, y = vertical,
        matching Gradio's select evt.index = [x, y]) to world (wx, wy).
        Mirrors the inverse of draw_map() used by click_to_goal()."""
        if self.cached_meta is None:
            return None, None
        origin_x, origin_y, h, ox, oy, res = self.cached_meta
        canvas = self.cached_map_img.shape[0]
        # undo the 90deg CW display rotation (same as click_to_goal)
        px = y
        py = canvas - 1 - x
        # undo scaling / centering, then the y-flip
        mx = (px - ox) / self.cached_scale
        myp = (py - oy) / self.cached_scale
        wx = mx * res + origin_x
        wy = (h - 1 - myp) * res + origin_y
        return wx, wy

    def set_initial_pose(self, wx, wy, yaw):
        """Queue a PoseWithCovarianceStamped for the robot and remember it so it
        is drawn on the map. Publishing happens on the dedicated node thread."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = FT_FRAME
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(wx)
        msg.pose.pose.position.y = float(wy)
        msg.pose.pose.position.z = 0.0
        # yaw about Z -> quaternion
        half = yaw / 2.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        # heuristic initial covariance so Nav2/AMCL accepts it
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = (30.0 * math.pi / 180.0) ** 2
        self._init_queue.put(msg)
        self.initial_pose = (float(wx), float(wy), float(yaw))
        self.last_initial = (float(wx), float(wy), float(yaw))
        return (f"{self.name}: Initial pose set -> "
                f"({wx:.3f}, {wy:.3f}) yaw {yaw:.3f} rad")

    def drag_to_initial(self, press, release):
        """press/release = (x, y) displayed-pixel coords of a click-drag.
        Position = press point, yaw = direction of the drag. Mirrors Foxglove's
        "2D Publish Pose" tool."""
        if self.cached_meta is None:
            return f"No {self.name} map yet (drag on the map once it loads)"
        x1, y1 = press
        x2, y2 = release
        wx, wy = self._display_to_world(x1, y1)
        if wx is None:
            return f"No {self.name} map yet"
        # world-frame yaw from the drag vector (see transform derivation)
        yaw = math.atan2(x2 - x1, y2 - y1)
        return self.set_initial_pose(wx, wy, yaw)

    # ---------------------- goal click ----------------------
    def click_to_goal(self, evt: gr.SelectData):
        if self.cached_meta is None:
            return f"No {self.name} map yet"
        origin_x, origin_y, h, ox, oy, res = self.cached_meta
        idx = evt.index
        row, col = idx[0], idx[1]
        canvas = self.cached_map_img.shape[0]
        # Invert the display pipeline (y-flip, then 90deg CW rotation):
        #   py = canvas - 1 - row ; px = col
        px = col
        py = canvas - 1 - row
        mx = (px - ox) / self.cached_scale
        myp = (py - oy) / self.cached_scale
        wx = mx * res + origin_x
        wy = (h - 1 - myp) * res + origin_y
        goal = PoseStamped()
        goal.header.frame_id = FT_FRAME
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = float(wx)
        goal.pose.position.y = float(wy)
        goal.pose.orientation.w = 1.0
        self._goal_queue.put(goal)
        self.last_goal = (wx, wy)
        return f"{self.name}: Goal set -> ({wx:.3f}, {wy:.3f})"


class LabRobotNode(Node):
    """Subscribes to map/pose/odom/camera/battery for every robot and exposes
    the currently-selected robot for rendering."""

    def __init__(self):
        super().__init__("lab_portal_node")
        self.robots = {}
        for name, topics in ROBOTS.items():
            robot = RobotState(self, name, topics)

            # All subscriptions are optional: ROS waits silently for any topic.
            # The map publishers (map_server / completed-map relays) use
            # TRANSIENT_LOCAL durability, so we MUST subscribe with the same
            # durability, otherwise no map message is ever delivered.
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, topics["map"], robot.map_cb,
                                     qos_profile=map_qos)
            self.create_subscription(PoseStamped, topics["pose"], robot.pose_cb, 10)
            self.create_subscription(Odometry, topics["odom"], robot.odom_cb, 10)
            if Go2FrontVideoData is not None:
                # Camera arrives as high-rate (~250Hz+) fragmented H.264. A
                # RELIABLE depth-10 subscription drops messages under the burst
                # load, which slices gaps into the H.264 stream and makes it
                # impossible to decode any frame. Use BEST_EFFORT + a deep
                # history so the newest contiguous bytes arrive without gap.
                cam_qos = QoSProfile(
                    depth=100,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                )
                self.create_subscription(
                    Go2FrontVideoData, topics["camera"], robot.camera_cb,
                    qos_profile=cam_qos,
                )
            if LowState is not None:
                self.create_subscription(LowState, topics["battery"], robot.battery_cb, 10)

            robot.goal_pub = self.create_publisher(PoseStamped, topics["goal"], 10)
            robot.init_pub = self.create_publisher(
                PoseWithCovarianceStamped, topics["initialpose"], 10)
            self.robots[name] = robot
            self.get_logger().info(
                f"[{name}] Subscribed to {topics['map']}, {topics['pose']}, "
                f"{topics['odom']}, {topics['camera']}, {topics['battery']}"
            )

        self.current = "Luna"

        # Publish goals from a dedicated thread. rclpy publish() must NOT be
        # called from a Gradio/anyio worker thread (it can corrupt the rclpy
        # context and crash the whole process).
        threading.Thread(target=self._goal_publish_loop, daemon=True).start()
        threading.Thread(target=self._diag_loop, daemon=True).start()

    def _diag_loop(self):
        """Periodic heartbeat: log what each robot is actually receiving so we
        can confirm map/pose/camera delivery. Diagnostic only."""
        while True:
            try:
                for name, robot in self.robots.items():
                    self.get_logger().info(
                        f"[DIAG] {name}: map={robot.cached_map_img is not None} "
                        f"pose={'y' if robot.robot_pose is not None else 'n'} "
                        f"pose_txt={robot.pose_latest!r} cam_frame={'y' if robot.color_frame is not None else 'n'} "
                        f"cam_ts={robot.cam_frame_ts:.1f} cam_fps={robot.cam_fps:.2f}"
                    )
            except Exception as e:
                self.get_logger().error(f"[DIAG] failed: {e}")
            threading.Event().wait(5.0)

    def _goal_publish_loop(self):
        while True:
            for name, robot in self.robots.items():
                try:
                    init = robot._init_queue.get(timeout=0.0)
                except queue.Empty:
                    init = None
                if init is not None:
                    try:
                        robot.init_pub.publish(init)
                        self.get_logger().info(
                            f"[{name}] Initial pose published -> "
                            f"({init.pose.pose.position.x:.3f}, "
                            f"{init.pose.pose.position.y:.3f})"
                        )
                    except Exception as e:
                        self.get_logger().error(f"[{name}] Failed to publish initial pose: {e}")
                try:
                    goal = robot._goal_queue.get(timeout=0.0)
                except queue.Empty:
                    goal = None
                if goal is not None:
                    try:
                        robot.goal_pub.publish(goal)
                        self.get_logger().info(
                            f"[{name}] Goal published -> "
                            f"({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f})"
                        )
                    except Exception as e:
                        self.get_logger().error(f"[{name}] Failed to publish goal: {e}")
            threading.Event().wait(0.1)

    # ---- the active robot's data ----
    @property
    def current_robot(self):
        return self.robots[self.current]

    def set_robot(self, name):
        if name in self.robots:
            self.current = name
            return f"Showing: {name}"
        return f"Unknown robot: {name}"

    def draw_map(self):
        return self.current_robot.draw_map()

    def draw_camera(self):
        return self.current_robot.draw_camera()

    def get_battery_data(self):
        return self.current_robot.get_battery_data()

    def get_motor_data(self):
        return self.current_robot.get_motor_data()

    def get_orientation_data(self):
        return self.current_robot.get_orientation_data()

    def get_pose_data(self):
        return self.current_robot.get_pose_data()

    def get_connection_status(self):
        return self.current_robot.get_connection_status()

    def click_to_goal(self, evt: gr.SelectData):
        return self.current_robot.click_to_goal(evt)

    def drag_to_initial(self, press, release):
        return self.current_robot.drag_to_initial(press, release)


def main():
    rclpy.init(args=None)
    node = LabRobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    theme = gr.themes.Soft(
        primary_hue=gr.themes.colors.blue,
        secondary_hue=gr.themes.colors.blue,
        neutral_hue=gr.themes.colors.slate,
        font="Helvetica, 'Helvetica Neue', Arial, sans-serif",
    ).set(
        body_background_fill="#f4f6fb",
        block_background_fill="#ffffff",
        block_border_color="#e3e8f2",
    )

    logo_uri = logo_data_uri()
    logo_tag = f'<img class="logo" src="{logo_uri}" alt="logo"/>' if logo_uri else ""
    header_html = f"""
    <div class="portal-header">
        {logo_tag}
        <div>
            <div class="title">QOD Lab · Go2 Live Dashboard</div>
            <div class="subtitle">Map · Camera · Battery · Pose — live from the lab</div>
        </div>
    </div>
    """

    with gr.Blocks(title="QOD Lab - Go2 Live Dashboard") as demo:
        gr.HTML(header_html)
        with gr.Row():
            robot_dd = gr.Dropdown(
                choices=list(ROBOTS.keys()),
                value=node.current,
                label="🤖 Select Robot",
                interactive=True,
                elem_classes=["robot-selector"],
            )
            conn_out = gr.Textbox(label="Connection", lines=1, interactive=False, scale=3)

        with gr.Row():
            with gr.Column(scale=3):
                init_toggle = gr.Checkbox(
                    label="✏️ Set Initial Pose — drag on the map (position = start, "
                          "orientation = drag direction)",
                    value=False,
                    elem_id="initpose_toggle",
                )
                map_img = gr.Image(label="Robot Map (click to set goal)",
                                   type="numpy", elem_id="map_image")
                goal_out = gr.Textbox(label="Goal Status", lines=1)
                init_status = gr.Textbox(label="🟢 Initial Pose Status", lines=1,
                                         interactive=False)
                init_bridge = gr.Textbox(visible=True, show_label=False,
                                         elem_id="initpose_bridge")
            with gr.Column(scale=2):
                cam_img = gr.Image(label="Camera", type="numpy")

        with gr.Row():
            with gr.Column():
                battery_out = gr.Textbox(label="🔋 Battery", lines=1)
                motor_out = gr.Textbox(label="🌡️ Motor Temps", lines=4)
                orient_out = gr.Textbox(label="📐 Orientation", lines=2)
            with gr.Column():
                pose_out = gr.Textbox(label="📍 Pose", lines=2)

        # --- tickers (each independently safe; missing data => placeholder) ---
        map_timer = gr.Timer(0.1)
        map_timer.tick(lambda: node.draw_map(), outputs=map_img)

        cam_timer = gr.Timer(0.2)
        cam_timer.tick(lambda: node.draw_camera(), outputs=cam_img)

        status_timer = gr.Timer(1.0)
        status_timer.tick(
            lambda: (
                node.get_connection_status(),
                node.get_battery_data(),
                node.get_motor_data(),
                node.get_orientation_data(),
                node.get_pose_data(),
            ),
            outputs=[conn_out, battery_out, motor_out, orient_out, pose_out],
        )

        def on_map_select(mode, evt: gr.SelectData):
            if mode:
                return ("✏️ Initial Pose mode ON — press and drag on the map to "
                        "set the initial pose (position + orientation).")
            return node.click_to_goal(evt)

        def on_drag_payload(payload, mode):
            if not mode:
                return "Initial pose drag ignored — toggle \"Set Initial Pose\" ON."
            if not payload or "," not in payload:
                return "No drag data received."
            try:
                x1, y1, x2, y2 = [int(x) for x in payload.split(",")]
            except Exception:
                return f"Bad drag payload: {payload!r}"
            return node.drag_to_initial((x1, y1), (x2, y2))

        map_img.select(on_map_select, inputs=init_toggle, outputs=goal_out)
        # Exposed as a public Gradio API endpoint ("drag_initial") so the browser
        # drag JS can POST the payload directly (POST /gradio_api/call/drag_initial
        # with {"data": [payload, mode]}). This is a deterministic bridge that works
        # regardless of Gradio's component-event semantics. The endpoint is bound to
        # init_bridge.input so it also works on the legacy event path.
        init_bridge.input(on_drag_payload, inputs=[init_bridge, init_toggle],
                          outputs=init_status, api_name="drag_initial")

        robot_dd.change(node.set_robot, robot_dd, None)

    demo.launch(
        server_name=SERV_NAME,
        server_port=SERV_PORT,
        theme=theme,
        css=DASHBOARD_CSS,
        head=INIT_POSE_JS,
    )


if __name__ == "__main__":
    main()