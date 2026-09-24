"""
lab_portal.py - 5G-Enabled Multi-Agent Mission Dashboard (QOD Lab).

A Hivemind Mission Dashboard-style UI for the QOD lab Go2 robots (Luna & Astro).

Runs inside the robot_hivemind ROS 2 (Humble) container and subscribes to the
lab's real topics (bridged from the Go2 over Zenoh / domain 70):

    MAP     /lidar_slam_2d_map            (nav_msgs/msg/OccupancyGrid)
    POSE    /luna/amcl_pose, /astro/amcl_pose  (geometry_msgs/msg/PoseWithCovarianceStamped)
    ODOM    /go2/restamped/robot_odom     (nav_msgs/msg/Odometry)
    CAMERA  /frontvideostream             (sensor_msgs/msg/Image)
    BATTERY /lf/lowstate                  (unitree_go/msg/LowState)

    Camera (default): the front camera is pulled from the QOD streaming
    platform's WebRTC SFU (the same channel + decode path the qod website uses),
    so the dashboard keeps showing the dog's camera even when the ROS
    /frontvideostream bridge is not publishing. Set LAB_CAMERA_SOURCE=ros to
    fall back to the legacy Go2FrontVideoData ROS-topic decoder.

Layout mirrors the Hivemind mission dashboard:
    - Workflow stepper (mapping -> scenario -> localisation -> operations)
    - Robot fleet panel (Luna / Astro cards with live status)
    - Live occupancy map (center) + live camera (right)
    - Telemetry + status footer

Working features stream live (map, camera, battery, pose). Features not yet
implemented (scenario/operations steps, 3D Foxglove view, ROS gateway, 5G
link, RealSense/YOLO streams) have been removed from the layout until they are
wired up.

The page ALWAYS loads. Every section shows a "Waiting for ..." placeholder
until its topic starts publishing. Click the map to publish a navigation goal
for the selected robot on its goal topic (e.g. /astro/goal_pose) in the "map"
frame. This drives the robot only when a navigation stack on the robot consumes
that goal topic. Toggle "Set Initial Pose" and drag on the map to publish an
initial pose (/initialpose) for the selected robot, like Foxglove's
"2D Publish Pose".

Usage:
    source /opt/ros/humble/setup.bash
    python3 lab_portal.py            # serves on 0.0.0.0:7860
"""

import math
import os
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
# - AMCL pose is per-robot (/luna/amcl_pose, /astro/amcl_pose) and is the
#   accurate localization source for drawing each robot on the map.
# - Odom is per-robot (/luna|astro/go2/restamped/robot_odom) and is used as a
#   fallback pose source when AMCL is not publishing on a robot.
# - No battery topic exists for either robot yet.
LUNA_TOPICS = {
    "map":     "/luna/lidar_slam_2d_map",
    "pose":    "/luna/amcl_pose",
    "odom":    "/luna/go2/restamped/robot_odom",
    "camera":  "/luna/frontvideostream",
    "source":  "luna",
    "battery": "/lf/lowstate",
    "goal":    "/luna/goal_pose",
    "initialpose": "/luna/initialpose",
    "plan":    "/luna/plan",
}

ASTRO_TOPICS = {
    "map":     "/astro/lidar_slam_2d_map",
    "pose":    "/astro/amcl_pose",
    "odom":    "/astro/go2/restamped/robot_odom",
    "camera":  "/astro/frontvideostream",
    "source":  "astro",
    "battery": "/lf/lowstate",
    "goal":    "/astro/goal_pose",
    "initialpose": "/astro/initialpose",
    "plan":    "/astro/plan",
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
# The Go2 source streams ~260 tiny H.264 fragments/s of only ~7 distinct display
# frames/s, so decoding faster than 0.4s only re-produces duplicate frames and
# costs CPU without raising the visible fps (which is source-limited).
CAM_DECODE_INTERVAL = 0.4
# Max frames to cache for smooth playback (one shot of motion per refill).
CAM_FRAMES_MAX = 20
# Cap the decoded segment to this many bytes -- we only need the most recent
# keyframe onward, not the whole (up to several-MB) rolling buffer.
# Sized generously so a full GOP (which may include one keyframe + many P-frames)
# fits in the window even as the buffer scrolls, keeping the camera continuous.
CAM_DECODE_WINDOW = 4 * 1024 * 1024

# ----------------------- Camera source (QOD WebRTC vs ROS) ------------
# "webrtc" (default): consume the dog's front camera from the QOD platform's
# WebRTC SFU on 127.0.0.1:8000 - the same channel and decode path the qod
# streaming platform website uses. This keeps the camera live even when the ROS
# /frontvideostream bridge is not publishing (identical frames to the website).
# "ros": legacy Go2FrontVideoData ROS-topic subscription + file decode above.
CAMERA_SOURCE = os.getenv("LAB_CAMERA_SOURCE", "webrtc").strip().lower()
QOD_SIGNALING_URL = os.getenv(
    "QOD_SIGNALING_URL", "ws://127.0.0.1:8000/ws/signaling"
)

# -------------------- QOD detection overlay -------------------------
# The qod cv-worker runs YOLO on each robot's WebRTC feed and publishes the
# boxes to Redis channel "detections:<source>"; the qod website draws them on a
# canvas. Subscribing to the same channel lets us draw the identical overlay on
# the dashboard camera. Set LAB_DETECTIONS=0 to disable.
DETECTIONS_ENABLED = os.getenv("LAB_DETECTIONS", "1").strip().lower() != "0"
REDIS_URL = os.getenv("QOD_DETECTIONS_REDIS_URL", "redis://127.0.0.1:6379")
# Only draw boxes the cv-worker already published (it filters at 0.25 -> 0.5);
# this guards the overlay against noise-tolerant thresholds.
DETECTIONS_MIN_CONF = float(os.getenv("LAB_DETECTIONS_MIN_CONF", "0.25"))
# Same class->style mapping as frontend/src/Services/detection-class-style.ts.
# NOTE: tuples are RGB, matching the (BGR-in-cv2-displayed-as-RGB) convention
# used by ROBOT_COLORS above (channel 0 = red on screen).
DETECTION_CLASS_STYLES = {
    "jerry":  dict(alias="Jerry can",      color=(255, 59, 48)),   # #ff3b30
    "spill":  dict(alias="Chemical spill", color=(34, 197, 94)),   # #22c55e
    "wire":   dict(alias="Exposed wire",   color=(168, 85, 247)),  # #a855f7
    "rubble": dict(alias="Debris",         color=(59, 130, 246)),  # #3b82f6
}
DEFAULT_DETECTION_STYLE = dict(alias="Object", color=(53, 255, 101))  # #35ff65

FT_FRAME = "map"

# Once the robot gets this close (meters) to the active goal, consider it
# reached and clear the plotted path/goal marker from the map. Widen enough to
# tolerate AMCL noise and Nav2 stopping a little short of the exact point.
GOAL_REACHED_TOLERANCE = 1.0

# ROS heading sources can stall (AMCL silent after start, or a one-shot /plan
# that is only re-published per goal). AMCL pose/yaw is trusted only while
# fresh; after this many seconds without a sample we fall back to the live
# odometry heading re-anchored into the map frame.
AMCL_FRESH_SECS = 3.0

# Per-robot visual identity. NOTE: cv2 draws BGR but Gradio displays RGB, so
# these tuples equal the browser colours (channel 0 = red on screen).
# Red = Luna, blue = Astro.
ROBOT_COLORS = {
    "Luna":  dict(fill=(255, 0, 0),  outline=(180, 0, 0),
                  path_dark=(160, 0, 0),  path_bright=(255, 0, 0)),
    "Astro": dict(fill=(0, 200, 255), outline=(0, 140, 200),
                  path_dark=(0, 0, 160), path_bright=(0, 0, 255)),
}


def _norm_angle(a):
    """Wrap an angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def _draw_plan(canvas, pts, dark, bright):
    """Draw the robot's planned route so it stands out against both the light
    explored map and the dark walls: light casing -> black under-stroke ->
    darker coloured outline -> bright coloured core (Luna red, Astro blue)."""
    cv2.polylines(canvas, [pts], False, (250, 250, 250), 14, cv2.LINE_AA)
    cv2.polylines(canvas, [pts], False, (0, 0, 0), 10, cv2.LINE_AA)
    cv2.polylines(canvas, [pts], False, dark, 7, cv2.LINE_AA)
    cv2.polylines(canvas, [pts], False, bright, 5, cv2.LINE_AA)


SERV_NAME = "0.0.0.0"
SERV_PORT = 7860

# ----------------------- import rclpy + msgs (defensive) -------------
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
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
    /* ---------- Hivemind Mission Dashboard style ----------
       Colors are driven by Gradio theme CSS variables so switching the
       theme in Settings applies everywhere (no hardcoded dark colors). */
    .gradio-container{background:var(--background-fill-primary) !important;
        color:var(--body-text-color) !important;max-width:100% !important;}
    .portal-header{display:flex;align-items:center;gap:14px;
        padding:14px 18px;border-radius:12px;
        background:linear-gradient(90deg,var(--block-background-fill),var(--block-background-fill));
        border:1px solid var(--block-border-color);
        color:var(--block-title-text-color);margin-bottom:10px;}
    .portal-header img.logo{height:44px;width:auto;border-radius:8px;
        box-shadow:0 2px 8px rgba(0,0,0,.35);}
    .portal-header .title{font-size:22px;font-weight:700;letter-spacing:.3px;
        color:var(--color-accent);}
    .portal-header .subtitle{font-size:13px;opacity:.85;margin-top:2px;
        color:var(--body-text-color-subdued);}
    .robot-selector{margin-top:10px;}
    #initpose_bridge{pointer-events:none;opacity:0;height:0;overflow:hidden;}
    #initpose_bridge textarea{opacity:0;height:0;min-height:0!important;}

    /* Blocks / cards */
    .gr-block,.gr-box,.gr-form{background:transparent !important;}
    .gr-group,.gr-gallery{background:var(--block-background-fill) !important;
        border:1px solid var(--block-border-color) !important;
        border-radius:12px !important;padding:12px !important;}

    

    /* Tab navigation — style gradio tabs to look like nav buttons */
    .tab-nav{display:flex;gap:6px !important;margin-bottom:10px !important;padding:6px !important;
        background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;}
    .tab-nav button{border-radius:10px !important;font-weight:600 !important;
        padding:10px 24px !important;font-size:14px !important;
        transition:all 0.2s ease !important;}
    .tab-nav button.selected{background:var(--color-accent) !important;
        color:var(--button-primary-text-color, #fff) !important;
        border-color:var(--color-accent) !important;}

    /* Robot fleet */
    .fleet-panel{display:flex;flex-direction:column;gap:10px;}
    .robot-card{background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;padding:12px;margin-bottom:10px;}
    .robot-card.selected{border-color:var(--color-accent);box-shadow:0 0 0 1px var(--color-accent);}
    .robot-name{font-size:15px;font-weight:600;color:var(--block-title-text-color);
        display:flex;align-items:center;gap:8px;margin-bottom:6px;}
    .robot-status-dot{width:8px;height:8px;border-radius:50%;display:inline-block;}
    .robot-status-dot.connected{background:#22c55e;box-shadow:0 0 6px #22c55e;}
    .robot-status-dot.waiting{background:#f59e0b;box-shadow:0 0 6px #f59e0b;}
    .telem-text{font-size:12px;color:var(--body-text-color-subdued);margin-top:4px;}

    /* Live 3D mapping panel */
    .map-panel{background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;padding:10px;}
    .map-panel-title{font-size:15px;font-weight:600;color:var(--block-title-text-color);
        display:flex;align-items:center;gap:8px;margin-bottom:6px;}

    /* Camera panel */
    .camera-panel{background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;padding:10px;}

    /* Status footer */
    .status-footer{display:flex;gap:14px;flex-wrap:wrap;
        background:var(--block-background-fill);border:1px solid var(--block-border-color);
        border-radius:12px;padding:10px 14px;margin-top:10px;font-size:12px;
        color:var(--body-text-color-subdued);}
    .status-item{display:flex;align-items:center;gap:6px;}
    .status-item .lv{width:8px;height:8px;border-radius:50%;display:inline-block;}
    .status-item .lv.green{background:#22c55e;}
    .status-item .lv.yellow{background:#f59e0b;}
    .status-item .lv.gray{background:#475569;}
    .status-item .lv.blue{background:#3b82f6;}

    .grid-2col{display:grid;grid-template-columns:1fr 1fr;gap:10px;}
    .grid-3col{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;}

    /* Future placeholder buttons */
    .future-placeholder{display:flex;align-items:center;justify-content:center;
        background:var(--input-background-fill);border:1px dashed var(--input-border-color);
        border-radius:8px;color:var(--body-text-color-subdued);font-size:12px;
        padding:8px 12px;min-height:36px;}

    .section-label{font-size:13px;color:var(--color-accent);font-weight:600;
        text-transform:uppercase;letter-spacing:.5px;margin:12px 0 8px;}
    .section-label:first-child{margin-top:0;}

    /* Mission buttons (future) */
    .mission-buttons{display:flex;gap:8px;flex-wrap:wrap;}
    .mission-buttons .gr-button{background:var(--input-background-fill);color:var(--color-accent);
        border:1px solid var(--input-border-color);border-radius:8px !important;}
    .mission-buttons .gr-button:hover{background:var(--border-color-primary);color:var(--body-text-color);}
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
            'z-index:99999;pointer-events:none;overflow:visible;' +
            'touch-action:none;');
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
        // Only the primary (left) mouse button — and only when the toggle is ON.
        if (e.pointerType === 'mouse' && e.button !== 0) return;
        var toggle = resolveEl('#initpose_toggle input');
        if (!toggle || !toggle.checked) return;
        // Stop the browser/Gradio default (map pan / text-select) that would
        // otherwise hijack a left-button drag, so left-click drag works.
        e.preventDefault();
        var p = toImgPts(e, img);
        sPx = p[0]; sPy = p[1];
        sX = e.clientX; sY = e.clientY;
        dragging = true;
        makeArrow();
        // Capture pointer events on the overlay so the drag does NOT pan the
        // underlying map (Gradio's image pan must not fire while drawing).
        svg.style.pointerEvents = 'auto';
        svg.style.display = '';
        drawArrow(e);
    });

    document.addEventListener('pointermove', function (e) {
        if (!dragging || !svg) return;
        e.preventDefault();
        drawArrow(e);
    });

    document.addEventListener('pointerup', function (e) {
        if (!dragging) return;
        dragging = false;
        if (svg) {
            svg.style.display = 'none';
            // Release the overlay so normal map panning/clicking works again.
            svg.style.pointerEvents = 'none';
        }
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
        self.pose_from_amcl = False   # True once AMCL (accurate) pose received
        self.yaw = 0.0
        self.odom_path = deque(maxlen=2000)
        self.pose_latest = "Waiting for pose..."
        self.last_goal = None
        # --- Nav2 planned path (published by the robot's global planner on
        #     /luna|astro/plan when a navigation goal is being executed) ---
        self.planner_path = None          # list of (wx, wy) in the "map" frame
        self.planner_path_stamp = 0.0
        # --- map-frame body anchoring ------------------------------------
        # The lab's AMCL (/luna|astro/amcl_pose) is NOT publishing, so the
        # odom fallback is used for the robot body. Odom lives in the odom
        # frame while the map + Nav2 plan are in the "map" frame, so drawing
        # the odom body with the map transform misplaces it (body off the
        # route). The Nav2 plan starts at the robot's current map-frame pose,
        # so we derive an odom->map offset from the plan's first point at each
        # replan and re-anchor the odom body into the map frame.
        self.last_odom = None             # latest (x, y) in the odom frame
        self.odom_to_map = None           # (dx, dy) odom -> map offset
        self.odom_yaw = None              # latest odom-frame heading (always kept)
        self.odom_to_map_yaw = None       # learned odom -> map rotation offset
        self.plan_start_yaw = None        # heading at the plan start (map frame)
        self._last_amcl_wall = 0.0        # last AMCL sample (wall clock)
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
        self._qod_detections = None  # QodDetectionsSubscriber (webrtc mode only)
        # Camera decode strategy: the Go2 H.264 stream is fragmented across many
        # small ROS messages, so a persistent ffmpeg pipe can never stay in sync
        # (constant "no frame!" / "non-existing PPS"). Instead we buffer bytes,
        # trim the tail to the latest SPS keyframe sequence, and decode that
        # window from a file on a background thread. Reliable, keeps the icon
        # green, and delivers a live multi-fps camera.
        if CAMERA_SOURCE == "webrtc":
            # Pull the camera from the QOD SFU instead of ROS (same feed the qod
            # website shows). Decoded BGR frames arrive via _on_qod_frame and are
            # queued into the same _cam_frames deck draw_camera() plays back.
            self._qod_consumer = None
            self._qod_detections = None
            qod_source = self.topics.get("source")
            if qod_source:
                try:
                    from web_backend.qod_consumer import QodCameraConsumer
                    self._qod_consumer = QodCameraConsumer(
                        source_id=qod_source,
                        name=self.name,
                        signaling_url=QOD_SIGNALING_URL,
                        on_frame=self._on_qod_frame,
                    )
                    self._qod_consumer.start()
                    self.node.get_logger().info(
                        f"[{self.name}] camera via QOD WebRTC (source={qod_source}, "
                        f"{QOD_SIGNALING_URL})"
                    )
                except Exception as e:
                    self.node.get_logger().error(
                        f"[{self.name}] failed to start QOD camera consumer: {e}"
                    )
                if DETECTIONS_ENABLED:
                    try:
                        from web_backend.qod_detections import (
                            QodDetectionsSubscriber,
                        )
                        self._qod_detections = QodDetectionsSubscriber(
                            source_id=qod_source,
                            name=self.name,
                            redis_url=REDIS_URL,
                        )
                        self._qod_detections.start()
                    except Exception as e:
                        self.node.get_logger().error(
                            f"[{self.name}] failed to start QOD detections "
                            f"subscriber: {e}"
                        )
        else:
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
        # The per-robot AMCL pose topics (/luna/amcl_pose, /astro/amcl_pose)
        # are geometry_msgs/msg/PoseWithCovarianceStamped, where the pose lives
        # at msg.pose.pose. The older PoseStamped path keeps msg.pose at the
        # top level. Normalize so both feed the same drawing/telemetry code.
        pose = getattr(msg, "pose", None)
        if pose is None:
            return
        p = getattr(pose, "pose", pose)  # PoseWithCovarianceStamped -> .pose.pose
        self.robot_pose = p
        self.pose_from_amcl = True       # AMCL is the accurate per-robot source
        self._last_amcl_wall = time.time()
        q = p.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        # Learn the odom -> map rotation from AMCL (map-frame heading minus the
        # near-simultaneous odom-frame heading) too, so a live re-anchored
        # heading stays available after AMCL goes quiet / between plans.
        if self.odom_yaw is not None:
            self.odom_to_map_yaw = _norm_angle(self.yaw - self.odom_yaw)
        self.pose_latest = (
            f"x: {p.position.x:.3f}   "
            f"y: {p.position.y:.3f}   "
            f"yaw: {self.yaw:.3f} rad"
        )

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.last_odom = (p.position.x, p.position.y)
        self.odom_path.append(
            (p.position.x, p.position.y)
        )
        q = p.orientation
        try:
            # Always keep the latest odom-frame heading so we can re-anchor it
            # into the map frame with the learned rotation offset (this is what
            # keeps the heading arrow live while travelling).
            self.odom_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
        except Exception:
            pass
        # The per-robot odom topic is a fallback pose source. Once the accurate
        # AMCL pose has been received, keep using AMCL (odom would otherwise
        # drift / overwrite the localized position and misplace the robot dot).
        if self.pose_from_amcl:
            return
        try:
            self.robot_pose = p
            self.yaw = self.odom_yaw
            self.pose_latest = (
                f"x: {p.position.x:.3f}   "
                f"y: {p.position.y:.3f}   "
                f"yaw: {self.yaw:.3f} rad"
            )
        except Exception:
            pass

    def plan_cb(self, msg):
        """Store the latest Nav2 global plan (/luna|astro/plan, nav_msgs/msg/Path).
        This is the path the robot is about to take; it is re-published each time
        a new navigation goal is planned, so we just remember the newest one."""
        try:
            self.planner_path = [
                (p.pose.position.x, p.pose.position.y) for p in msg.poses
            ]
            self.planner_path_stamp = self.node.get_clock().now().nanoseconds / 1e9
            # Re-anchor the odom body into the map frame: the first plan pose is
            # the robot's current map-frame position, so the difference against
            # the (nearly simultaneous) odom reading is the odom->map offset.
            if msg.poses:
                fp = msg.poses[0].pose
                fx, fy = fp.position.x, fp.position.y
                if self.last_odom is not None:
                    self.odom_to_map = (fx - self.last_odom[0],
                                        fy - self.last_odom[1])
                # Prefer the plan's stated heading (map frame) for the body arrow.
                try:
                    q = fp.orientation
                    self.plan_start_yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                    )
                except Exception:
                    pass
                # Learn the odom -> map rotation: the plan's first pose heading
                # is the robot's map-frame heading at plan time, while odom_yaw
                # is the same physical heading in the odom frame (odom and map
                # axes differ by a fixed rotation). This offset lets us keep a
                # live map-frame heading between plans / while AMCL is silent.
                if (self.plan_start_yaw is not None
                        and self.odom_yaw is not None):
                    self.odom_to_map_yaw = _norm_angle(
                        self.plan_start_yaw - self.odom_yaw
                    )
                if self.pose_from_amcl:
                    self.pose_latest = (
                        f"x: {self.robot_pose.position.x:.3f}   "
                        f"y: {self.robot_pose.position.y:.3f}   "
                        f"yaw: {self.yaw:.3f} rad"
                    )
                else:
                    self.pose_latest = (
                        f"x: {fx:.3f}   "
                        f"y: {fy:.3f}   "
                        f"yaw: {self.plan_start_yaw:.3f} rad "
                        f"(from plan)"
                    )
            self.node.get_logger().info(
                f"[{self.name}] /plan received: {len(self.planner_path)} pts "
                f"frame={getattr(msg.header, 'frame_id', '?')!r}"
                f" odom->map={self.odom_to_map}"
            )
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] plan_cb failed: {e}")

    def _live_map_yaw(self):
        """Best continuously-updating map-frame heading.

        Re-anchor the latest odom heading into the map frame using the learned
        rotation offset (from a plan start pose or AMCL while it was fresh), so
        the heading arrow turns in real time as the robot drives - even between
        /plan re-publishes or while AMCL is silent."""
        if self.odom_to_map_yaw is not None and self.odom_yaw is not None:
            return _norm_angle(self.odom_yaw + self.odom_to_map_yaw)
        if self.plan_start_yaw is not None:
            return self.plan_start_yaw
        return self.yaw

    def map_pose(self):
        """Best available robot position expressed in the 'map' frame.

        Returns (x, y, yaw) or None. Order of preference:
          1. AMCL pose, as long as it is still publishing (fresh samples).
          2. Odom re-anchored into the map frame via the offset taken from the
             Nav2 plan's start pose (used while navigating without AMCL / when
             AMCL stalls, and kept after arrival so the body does not jump
             frames). The heading here is LIVE (re-anchored odom yaw).
          3. Nav2 plan start pose (map frame).
          4. Raw odom (unreliable frame, last resort before any map data)."""
        amcl_fresh = (self.pose_from_amcl
                      and time.time() - self._last_amcl_wall <= AMCL_FRESH_SECS)
        if amcl_fresh and self.robot_pose is not None:
            return (self.robot_pose.position.x,
                    self.robot_pose.position.y, self.yaw)
        if (self.odom_to_map is not None and self.last_odom is not None):
            yaw = self._live_map_yaw()
            return (self.last_odom[0] + self.odom_to_map[0],
                    self.last_odom[1] + self.odom_to_map[1], yaw)
        if self.planner_path:
            fx, fy = self.planner_path[0]
            yaw = self._live_map_yaw()
            return (fx, fy, yaw)
        if self.robot_pose is not None:
            return (self.robot_pose.position.x,
                    self.robot_pose.position.y, self.yaw)
        return None

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

    def _on_qod_frame(self, frame):
        """Queue a decoded frame from the QOD WebRTC consumer for smooth
        playback. draw_camera() pops from the same _cam_frames deck used by the
        ROS decode path, so the UI code is shared. The consumer hands us BGR;
        Gradio's gr.Image component displays RGB, so convert here once (this is
        what caused the blue tint)."""
        try:
            if frame.ndim == 3 and frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w = frame.shape[:2]
            if w > CAM_W or h > CAM_H:
                frame = cv2.resize(frame, (CAM_W, CAM_H),
                                   interpolation=cv2.INTER_AREA)
            with self._cam_lock:
                self._cam_frames.append(frame)
                self.color_frame = frame
            self.cam_frame_ts = time.time()
            self.cam_stamp = time.time()
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] QOD frame failed: {e}")

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

        # Prefer starting at the newest COMPLETE SPS->PPS->IDR(5) sequence so the
        # returned segment is self-contained and does not rely on a cached
        # parameter set that the camera may have renegotiated (a stale SPS+PPS
        # prepended to newer slices is what makes cv2 yield zero frames forever).
        best = codes[ref][0]
        for i in range(len(codes) - 2):
            if (nal_type_at(i) == 7 and nal_type_at(i + 1) == 8
                    and nal_type_at(i + 2) == 5 and codes[i][0] <= best):
                best = codes[i][0]
                break

        start = best
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
        causing the camera to freeze permanently), we refresh the cached SPS+PPS
        from the newest bytes each decode and, when available, split at the
        newest SPS->PPS->IDR sequence so the decoder always has a valid reference
        to start from. If a decode yields zero frames, the cached parameter sets
        may be stale (the camera renegotiated its encoder) -- we clear them and
        retry once against a freshly assembled segment so the stream self-heals.
        """
        if not self._cam_buf:
            return []
        with self._cam_lock:
            sps, pps = self._sps, self._pps
            raw = bytes(self._cam_buf[-CAM_DECODE_WINDOW:])
        if len(raw) < CAM_MIN_BYTES:
            return []

        def assemble(s, p):
            r = self._trim_to_keyframe(raw)
            if r is None:
                return None
            if s:
                # Prepend the (refreshed) parameter sets so the decoder always
                # has them, even after the keyframe scrolled out of the buffer.
                return s + p + r
            return r

        seg = assemble(sps, pps)
        if seg is None:
            return []
        frames = self._decode_segment(seg)
        if frames:
            return frames

        # Zero frames from a valid keyframe segment -> cached params are stale.
        # Refresh SPS/PPS from the newest bytes and retry once.
        with self._cam_lock:
            self._capture_headers(bytes(self._cam_buf[-CAM_DECODE_WINDOW:]))
            sps, pps = self._sps, self._pps
        seg = assemble(sps, pps)
        if seg is None:
            return []
        return self._decode_segment(seg)

    def _decode_segment(self, raw):
        """Write raw H.264 bytes to a temp file and decode up to CAM_FRAMES_MAX
        frames. Returns [] on failure so callers can trigger self-healing."""
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

        mp = self.map_pose()
        if mp is not None:
            body_x, body_y, body_yaw = mp
            # Once the robot arrives at the goal, clear the plan + goal marker
            # so the path disappears from the map. "Arrived" = the current
            # map-frame pose is within tolerance of the goal.
            if self.last_goal is not None:
                gx, gy = self.last_goal
                d_now = math.hypot(body_x - gx, body_y - gy)
                if d_now < GOAL_REACHED_TOLERANCE:
                    self.planner_path = None
                    self.last_goal = None

            # Nav2 planned path (the route the robot is about to take). Color
            # matches the robot: Luna = red, Astro = blue (same as its dot).
            colors = ROBOT_COLORS[self.name]
            if self.planner_path and len(self.planner_path) >= 2:
                pts = np.array(
                    [to_px(wx, wy) for wx, wy in self.planner_path], dtype=np.int32
                )
                _draw_plan(canvas, pts, colors["path_dark"], colors["path_bright"])

            # Goal marker (orange crosshair + dot) at the last clicked point.
            if self.last_goal is not None:
                gx, gy = to_px(*self.last_goal)
                cv2.circle(canvas, (gx, gy), 9, (0, 165, 255), 3, cv2.LINE_AA)
                cv2.circle(canvas, (gx, gy), 3, (0, 165, 255), -1, cv2.LINE_AA)
                cv2.line(canvas, (gx - 14, gy), (gx + 14, gy), (0, 165, 255), 2, cv2.LINE_AA)
                cv2.line(canvas, (gx, gy - 14), (gx, gy + 14), (0, 165, 255), 2, cv2.LINE_AA)

            px, py = to_px(body_x, body_y)
            dot = colors["fill"]
            cv2.circle(canvas, (px, py), 10, dot, -1)
            ex = int(px + 30 * math.cos(body_yaw))
            ey = int(py - 30 * math.sin(body_yaw))
            cv2.arrowedLine(canvas, (px, py), (ex, ey), (0, 0, 0), 3)

            canvas = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)
            hc, wc = canvas.shape[:2]
            cv2.putText(canvas, "N", (wc // 2, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
            cv2.putText(canvas, "S", (wc // 2, hc - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
        return canvas

    def _draw_detections(self, frame):
        """Draw the qod cv-worker's object detections on the camera frame -
        the exact same stream (Redis channel detections:<source>) and class
        styling the qod website uses, so both UIs match. Boxes arrive
        normalized 0..1 relative to the original 1280x720 feed; the displayed
        frame keeps the same aspect ratio, so we scale linearly to its size."""
        sub = getattr(self, "_qod_detections", None)
        payload = sub.detections() if sub is not None else []
        if not payload:
            return frame
        h, w = frame.shape[:2]
        outer = max(2, w // 170)   # dark under-stroke
        inner = max(1, outer - 1)  # coloured core - same look as the site
        font = cv2.FONT_HERSHEY_SIMPLEX
        for det in payload:
            try:
                cls = str(det.get("class") or "").strip().lower()
                conf = float(det.get("confidence") or 0.0)
                if conf < DETECTIONS_MIN_CONF:
                    continue
                bbox = det.get("bbox") or {}
                x = float(bbox["x"])
                y = float(bbox["y"])
                bw = float(bbox["width"])
                bh = float(bbox["height"])
                # The site accepts normalized (0..1) boxes and also raw pixel
                # boxes; we only draw normalized ones, like the YOLO publisher.
                if not (0 <= x <= 1.1 and 0 <= y <= 1.1 and
                        0 <= bw <= 1.1 and 0 <= bh <= 1.1):
                    continue
                style = DETECTION_CLASS_STYLES.get(
                    cls, DEFAULT_DETECTION_STYLE
                )
                label = f"{style['alias']} {conf * 100:.0f}%"
                px = int(x * w)
                py = int(y * h)
                pw = max(1, int(bw * w))
                ph = max(1, int(bh * h))
                pt1 = (px, py)
                pt2 = (px + pw, py + ph)
                cv2.rectangle(frame, pt1, pt2, (0, 0, 0), outer)
                cv2.rectangle(frame, pt1, pt2, style["color"], inner)
                (tw, th), baseline = cv2.getTextSize(
                    label, font, 0.45, 1
                )
                lx = px
                ly = max(0, py - (th + baseline + 4))
                cv2.rectangle(
                    frame, (lx, ly), (lx + tw + 6, ly + th + 6),
                    (0, 0, 0), -1,
                )
                cv2.putText(
                    frame, label, (lx + 3, ly + th + 2), font, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA,
                )
            except Exception:
                continue
        return frame

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
        return self._draw_detections(frame)

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
        mp = self.map_pose()
        if mp is None:
            return self.pose_latest
        bx, by, byaw = mp
        pos = (f"x: {bx:.3f}   y: {by:.3f}   yaw: {byaw:.3f} rad")
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
        # Goal orientation: keep the robot's current heading so it drives to the
        # clicked point (no heading requirement). Published on the goal topic
        # (e.g. /astro/goal_pose) in the "map" frame.
        goal = PoseStamped()
        goal.header.frame_id = FT_FRAME          # "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = float(wx)
        goal.pose.position.y = float(wy)
        goal.pose.position.z = 0.0
        half = self.yaw / 2.0
        goal.pose.orientation.z = math.sin(half)
        goal.pose.orientation.w = math.cos(half)
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
            # Per-robot AMCL pose (/luna/amcl_pose, /astro/amcl_pose) is
            # geometry_msgs/msg/PoseWithCovarianceStamped and gives the accurate
            # localized position for drawing each robot on the map.
            self.create_subscription(PoseWithCovarianceStamped, topics["pose"],
                                     robot.pose_cb, 10)
            # Live odometry. The robot odom topics here are bridged by
            # zenoh_bridge_ros2dds, which publishes BEST_EFFORT; a RELIABLE
            # subscription is QoS-incompatible and silently receives nothing
            # (dead robot body/heading on the map). Use BEST_EFFORT + deep
            # history so odom - and therefore the live heading - always flows.
            odom_qos = QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(Odometry, topics["odom"], robot.odom_cb,
                                     qos_profile=odom_qos)
            # Nav2's global planner publishes the path currently being followed
            # on /luna|astro/plan; draw it on the map so the operator can see the
            # route the robot is about to take before/while it drives.
            self.create_subscription(Path, topics["plan"], robot.plan_cb, 10)
            if Go2FrontVideoData is not None and CAMERA_SOURCE != "webrtc":
                # Camera arrives as high-rate (~250Hz+) fragmented H.264. A
                # RELIABLE depth-10 subscription drops messages under the burst
                # load, which slices gaps into the H.264 stream and makes it
                # impossible to decode any frame. Use BEST_EFFORT + a deep
                # history so the newest contiguous bytes arrive without gap.
                # (Skipped in "webrtc" mode: the camera comes from the QOD SFU.)
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
        self.patrol_robot = "Luna"

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
                    live_yaw = robot._live_map_yaw()
                    if (robot.pose_from_amcl
                            and time.time() - robot._last_amcl_wall
                            <= AMCL_FRESH_SECS):
                        src = "amcl"
                    elif robot.odom_to_map_yaw is not None:
                        src = "odom+off"
                    elif robot.plan_start_yaw is not None:
                        src = "plan"
                    else:
                        src = "odom"
                    self.get_logger().info(
                        f"[DIAG] {name}: map={robot.cached_map_img is not None} "
                        f"pose={'y' if robot.robot_pose is not None else 'n'} "
                        f"plan={'y' if robot.planner_path and len(robot.planner_path) >= 2 else 'n'} "
                        f"plan_pts={len(robot.planner_path) if robot.planner_path else 0} "
                        f"pose_txt={robot.pose_latest!r} "
                        f"hdg={src} "
                        f"map_yaw={(live_yaw if live_yaw is not None else float('nan')):.2f} "
                        f"cam_frame={'y' if robot.color_frame is not None else 'n'} "
                        f"cam_ts={robot.cam_frame_ts:.1f} cam_fps={robot.cam_fps:.2f} "
                        f"det={len(robot._qod_detections.detections()) if getattr(robot, '_qod_detections', None) else 'n/a'}"
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

    # ---- patrolling page ----
    def set_patrol_robot(self, name):
        if name in self.robots:
            self.patrol_robot = name
            return f"Patrol goal target: {name}"
        return f"Unknown robot: {name}"

    def draw_patrol_map(self):
        """Render both robots, their paths and goals on a single occupancy
        grid.  The base map is taken from whichever robot has a live occupancy
        grid; every robot's pose, Nav2 plan and goal marker are overlaid."""
        # Pick the base map from whichever robot has one.
        base = None
        label_positions = []
        for r in self.robots.values():
            if r.cached_map_img is not None:
                base = r
                break
        if base is None:
            return _placeholder(640, 480, "WAITING FOR MAP DATA...")

        canvas = base.cached_map_img.copy()
        scale = base.cached_scale
        ox, oy, h, pox, poy, res = base.cached_meta

        def to_px(wx, wy):
            mx = (wx - ox) / res
            my = (wy - oy) / res
            my = h - 1 - my
            return int(mx * scale) + pox, int(my * scale) + poy

        # Draw each robot's plan + goal (same logic as single-robot draw_map).
        for name, robot in self.robots.items():
            mp = robot.map_pose()
            if mp is None:
                continue
            body_x, body_y, body_yaw = mp
            colors = ROBOT_COLORS.get(name, ROBOT_COLORS["Luna"])
            # Goal-reached check: clears the plan so the path disappears. Only
            # the current map-frame pose is compared to the goal.
            if robot.last_goal is not None:
                gx, gy = robot.last_goal
                d_now = math.hypot(body_x - gx, body_y - gy)
                if d_now < GOAL_REACHED_TOLERANCE:
                    robot.planner_path = None
                    robot.last_goal = None

            # Nav2 planned path (the route the robot is about to take), drawn in
            # the robot's colour with a light casing + black under-stroke.
            if robot.planner_path and len(robot.planner_path) >= 2:
                pts = np.array(
                    [to_px(wx, wy) for wx, wy in robot.planner_path],
                    dtype=np.int32,
                )
                _draw_plan(canvas, pts, colors["path_dark"], colors["path_bright"])

            # Goal marker (orange crosshair) at the last clicked point.
            if robot.last_goal is not None:
                gx, gy = to_px(*robot.last_goal)
                cv2.circle(canvas, (gx, gy), 9, (0, 165, 255), 3, cv2.LINE_AA)
                cv2.circle(canvas, (gx, gy), 3, (0, 165, 255), -1, cv2.LINE_AA)
                cv2.line(canvas, (gx - 14, gy), (gx + 14, gy),
                         (0, 165, 255), 2, cv2.LINE_AA)
                cv2.line(canvas, (gx, gy - 14), (gx, gy + 14),
                         (0, 165, 255), 2, cv2.LINE_AA)

            fill = colors.get("fill", (255, 0, 0))
            outline = colors.get("outline", (180, 0, 0))
            px, py = to_px(body_x, body_y)
            cv2.circle(canvas, (px, py), 10, fill, -1)
            cv2.circle(canvas, (px, py), 10, outline, 2)
            ex = int(px + 30 * math.cos(body_yaw))
            ey = int(py - 30 * math.sin(body_yaw))
            cv2.arrowedLine(canvas, (px, py), (ex, ey), (0, 0, 0), 3)
            label_positions.append((px, py, name))

        canvas = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)
        hc, wc = canvas.shape[:2]

        # Robot labels.
        for px, py, name in label_positions:
            # 90-degree CW rotation: dst(row = src_col, col = H-1-src_row).
            # Source row = py, col = px -> dst col = H-1-py, dst row = px.
            # cv2.putText's (x, y) is (col, baseline-row).
            lx = wc - 1 - py + 5
            ly = px - 5
            cv2.putText(canvas, name, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(canvas, name, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.putText(canvas, "N", (wc // 2, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 2)
        cv2.putText(canvas, "S", (wc // 2, hc - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 2)

        return canvas

    def patrol_click_to_goal(self, evt: gr.SelectData):
        """Forward a map click on the patrol page to the patrol-selected robot."""
        robot = self.robots.get(self.patrol_robot)
        if robot is None:
            return "No robot selected for patrol"
        return robot.click_to_goal(evt)

    def patrol_draw_cameras(self):
        """Return both robots' camera streams for the fleet camera panel."""
        luna = self.robots.get("Luna")
        astro = self.robots.get("Astro")
        return (
            luna.draw_camera() if luna else _placeholder(640, 360, "LUNA OFFLINE"),
            astro.draw_camera() if astro else _placeholder(640, 360, "ASTRO OFFLINE"),
        )

    def patrol_get_status(self):
        """Per-robot telemetry (connection + pose) for the fleet footer."""
        luna = self.robots.get("Luna")
        astro = self.robots.get("Astro")
        if luna is None or astro is None:
            return ("LUNA unavailable", "—", "ASTRO unavailable", "—")
        return (
            luna.get_connection_status(),
            luna.get_pose_data(),
            astro.get_connection_status(),
            astro.get_pose_data(),
        )


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
    )

    logo_uri = logo_data_uri()
    logo_tag = f'<img class="logo" src="{logo_uri}" alt="logo"/>' if logo_uri else ""
    header_html = f"""
    <div class="portal-header">
        {logo_tag}
        <div>
            <div class="title">5G-Enabled Multi-Agent Mission Dashboard</div>
            <div class="subtitle">E-12-DMAT-025 · Luna &amp; Astro · live telemetry from the lab</div>
        </div>
    </div>
    """

    with gr.Blocks(title="5G-Enabled Multi-Agent Mission Dashboard", theme=theme, css=DASHBOARD_CSS, head=INIT_POSE_JS) as demo:
        gr.HTML(header_html)

        with gr.Tabs() as page_tabs:
            # ================================================================
            # TAB 1: MAPPING (existing single-robot dashboard)
            # ================================================================
            with gr.Tab("Mapping", id="mapping"):

                # ================================================================
                # TOP ROW: ROBOT FLEET (LEFT) + MAP (CENTER) + CAMERA (RIGHT)
                # ================================================================
                with gr.Row():
                    # ---------- LEFT: ROBOT FLEET ----------
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        gr.Markdown("#### ROBOT FLEET")
                        robot_dd = gr.Dropdown(
                            choices=list(ROBOTS.keys()),
                            value=node.current,
                            label="Select Robot",
                            interactive=True,
                            elem_classes=["robot-selector"],
                        )

                        # Luna card
                        with gr.Group(elem_classes=["robot-card"]):
                            with gr.Row():
                                gr.Markdown("### Luna")
                                gr.Markdown("🟢 Connected")
                            luna_map_status = gr.Textbox(label="Map", lines=1, interactive=False)
                            luna_pose_status = gr.Textbox(label="Pose", lines=1, interactive=False)
                            luna_cam_status = gr.Textbox(label="Camera", lines=1, interactive=False)

                        # Astro card
                        with gr.Group(elem_classes=["robot-card"]):
                            with gr.Row():
                                gr.Markdown("### Astro")
                                gr.Markdown("🟢 Connected")
                            astro_map_status = gr.Textbox(label="Map", lines=1, interactive=False)
                            astro_pose_status = gr.Textbox(label="Pose", lines=1, interactive=False)
                            astro_cam_status = gr.Textbox(label="Camera", lines=1, interactive=False)

                        gr.Markdown("#### FUTURE: MISSION CONTROLS")
                        with gr.Group(elem_classes=["mission-buttons"]):
                            with gr.Row():
                                gr.Button("🎯 Start Mapping", interactive=False)
                            with gr.Row():
                                gr.Button("🚀 Start Mission", interactive=False)
                                gr.Button("⏹ Stop", interactive=False)

                    # ---------- CENTER: LIVE 3D MAPPING ----------
                    with gr.Column(scale=2, elem_classes=["map-panel"]):
                        gr.Markdown("## Live Mapping")

                        init_toggle = gr.Checkbox(
                            label="Set Initial Pose — drag on the map (position = start, "
                                  "orientation = drag direction)",
                            value=False,
                            elem_id="initpose_toggle",
                        )

                        map_img = gr.Image(label="Occupancy Map — click to set a nav goal",
                                           type="numpy", elem_id="map_image", height=520)
                        goal_out = gr.Textbox(label="Goal Status", lines=1)

                        with gr.Row():
                            init_status = gr.Textbox(label="🟢 Initial Pose Status", lines=1,
                                                     interactive=False, scale=2)
                            init_bridge = gr.Textbox(visible=True, show_label=False,
                                                     elem_id="initpose_bridge", scale=0)

                        gr.Markdown("**Legend:** 🔴 **Luna** (red dot · red route) · 🟦 **Astro** (blue dot · blue route) · black arrow = heading · **orange target = goal (route clears on arrival)** · blue = explored · dark = walls")

                    # ---------- RIGHT: LIVE CAMERA ----------
                    with gr.Column(scale=1, elem_classes=["camera-panel"]):
                        gr.Markdown("## Live Camera")
                        cam_img = gr.Image(label="Forward Camera — selected robot",
                                           type="numpy", height=300)
                        cam_fps_out = gr.Textbox(label="Stream FPS", lines=1, interactive=False)

                # ================================================================
                # BOTTOM ROW: TELEMETRY + STATUS FOOTER
                # ================================================================
                with gr.Row():
                    with gr.Column():
                        battery_out = gr.Textbox(label="🔋 Battery", lines=2)
                        orient_out = gr.Textbox(label="📐 Orientation", lines=2)
                    with gr.Column():
                        pose_out = gr.Textbox(label="📍 Pose", lines=2)
                        motor_out = gr.Textbox(label="🌡️ Motor Temps", lines=3)

                conn_out = gr.Textbox(label="Connection Status", lines=2, interactive=False)

                # --- tickers (each independently safe; missing data => placeholder) ---
                # Map is redrawn at 2 Hz: a full occupancy-grid image shipped to the
                # browser on every tick is the largest CPU/bandwidth driver. Slowing it
                # to 0.5s frees CPU so the camera stream (below) can refresh smoothly
                # instead of being starved by a 10 Hz map redraw.
                map_timer = gr.Timer(0.5)
                map_timer.tick(lambda: node.draw_map(), outputs=map_img)

                # Camera pulls decoded frames faster than before. Frames are produced by
                # a background decode thread, so a quick 0.1s pull yields fluid playback
                # without adding decode work -- it just drains the ready frame queue.
                cam_timer = gr.Timer(0.1)
                cam_timer.tick(lambda: node.draw_camera(), outputs=cam_img)

                status_timer = gr.Timer(1.0)
                status_timer.tick(
                    lambda: (
                        node.get_connection_status(),
                        node.get_battery_data(),
                        node.get_motor_data(),
                        node.get_orientation_data(),
                        node.get_pose_data(),
                        f"{node.current_robot.cam_fps:.1f} fps",
                    ),
                    outputs=[conn_out, battery_out, motor_out, orient_out, pose_out, cam_fps_out],
                )

                # Populate per-robot status cards
                robot_status_timer = gr.Timer(2.0)

                def robot_status():
                    texts = []
                    for name in node.robots:
                        robot = node.robots[name]
                        now = node.get_clock().now().nanoseconds / 1e9
                        map_ok = "🟢 live" if robot.map_stamp > 0 and now - robot.map_stamp < 5.0 else ("🟡 stale" if robot.map_stamp > 0 else "⚪ waiting")
                        pose_ok = "🟢 live" if robot.robot_pose is not None else "⚪ waiting"
                        cam_ok = "🟢 live" if robot.cam_fps > 0.5 else "⚪ waiting"
                        texts.append(map_ok)
                        texts.append(pose_ok)
                        texts.append(cam_ok)
                    return texts

                robot_status_timer.tick(
                    robot_status,
                    outputs=[luna_map_status, luna_pose_status, luna_cam_status,
                             astro_map_status, astro_pose_status, astro_cam_status],
                )

                def on_map_select(mode, evt: gr.SelectData):
                    if mode:
                        return ("Initial Pose mode ON — press and drag on the map to "
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

            # ================================================================
            # TAB 2: FLEET VIEW (both robots on one map)
            # ================================================================
            with gr.Tab("Fleet View", id="patrolling"):
                # ---------- TOP ROW: GOAL TARGET (L) + FLEET MAP (C) + CAMERA (R) ----------
                with gr.Row():
                    # LEFT: Goal Target Robot selector
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        gr.Markdown("#### GOAL TARGET")
                        patrol_robot_dd = gr.Dropdown(
                            choices=list(ROBOTS.keys()),
                            value=node.patrol_robot,
                            label="Goal Target Robot",
                            interactive=True,
                            elem_classes=["robot-selector"],
                        )

                    # CENTER: Fleet Patrolling Map (both robots)
                    with gr.Column(scale=2, elem_classes=["map-panel"]):
                        gr.Markdown("## Fleet Patrolling Map")
                        patrol_map_img = gr.Image(
                            label="Occupancy Map — both robots · click to set a nav goal",
                            type="numpy", elem_id="patrol_map_image", height=520)
                        gr.Markdown(
                            "**Legend:** 🔴 **Luna** (red dot, red route) · "
                            "🟦 **Astro** (blue dot, blue route) — black arrow = heading · "
                            "**orange target = goal (route clears on arrival)**")

                    # RIGHT: Fleet Camera — streams from BOTH robots
                    with gr.Column(scale=1, elem_classes=["camera-panel"]):
                        gr.Markdown("## Fleet Camera")
                        patrol_cam_luna = gr.Image(
                            label="Luna — Forward Camera", type="numpy", height=250)
                        patrol_cam_astro = gr.Image(
                            label="Astro — Forward Camera", type="numpy", height=250)

                # ---------- SECTION 2: FLEET OVERVIEW / GOAL ROUTING / FUTURE ----------
                with gr.Row():
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        with gr.Group(elem_classes=["robot-card"]):
                            gr.Markdown("### Fleet Overview")
                            gr.Markdown(
                                "**Luna** 🔴 (red) · **Astro** 🟦 (blue)\n\n"
                                "Both robots are shown together. Click the map to send "
                                "a nav goal to the robot selected in **Goal Target Robot**."
                            )
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        with gr.Group(elem_classes=["robot-card"]):
                            gr.Markdown("### Goal Routing")
                            patrol_goal_out = gr.Textbox(
                                label="Patrol Goal Status", lines=2, interactive=False)
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        gr.Markdown("#### FUTURE: PATROL ROUTES")
                        with gr.Group(elem_classes=["mission-buttons"]):
                            with gr.Row():
                                gr.Button("📍 Add Waypoint", interactive=False)
                                gr.Button("🚀 Start Patrol", interactive=False)

                # ---------- SECTION 3: FLEET TELEMETRY (BOTH ROBOTS) ----------
                with gr.Row():
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        patrol_conn_luna = gr.Textbox(
                            label="Luna — Connection Status", lines=2, interactive=False)
                        patrol_pose_luna = gr.Textbox(
                            label="Luna — 📍 Pose", lines=2, interactive=False)
                    with gr.Column(scale=1, elem_classes=["fleet-panel"]):
                        patrol_conn_astro = gr.Textbox(
                            label="Astro — Connection Status", lines=2, interactive=False)
                        patrol_pose_astro = gr.Textbox(
                            label="Astro — 📍 Pose", lines=2, interactive=False)

                # ---------- TICKERS (run continuously; cheap when tab hidden) ----------
                patrol_map_timer = gr.Timer(0.5)
                patrol_map_timer.tick(lambda: node.draw_patrol_map(),
                                      outputs=patrol_map_img)

                patrol_cam_timer = gr.Timer(0.1)
                patrol_cam_timer.tick(lambda: node.patrol_draw_cameras(),
                                      outputs=[patrol_cam_luna, patrol_cam_astro])

                patrol_status_timer = gr.Timer(1.0)
                patrol_status_timer.tick(
                    lambda: node.patrol_get_status(),
                    outputs=[patrol_conn_luna, patrol_pose_luna,
                             patrol_conn_astro, patrol_pose_astro],
                )

                def on_patrol_map_select(evt: gr.SelectData):
                    return node.patrol_click_to_goal(evt)

                patrol_map_img.select(on_patrol_map_select, inputs=None,
                                      outputs=patrol_goal_out)
                patrol_robot_dd.change(node.set_patrol_robot, patrol_robot_dd, None)

            # ================================================================
            # TAB 3: UI PATOL (embedded polygon-constrained patrol planner)
            # The patrol planner runs in the same robot_hivemind container on
            # port 8766 (host networking). The dashboard embeds it in an
            # iframe; the port is forwarded by the SSH tunnel just like 7860.
            # ================================================================
            with gr.Tab("Patrolling", id="patrol-ui"):
                gr.Markdown("#### POLYGON-CONSTRAINED PATROL PLANNER")
                gr.Markdown(
                    "Draw a patrol region on the map below, run the planner, "
                    "then dispatch Luna / Astro through the planned checkpoints."
                )
                gr.HTML(
                    '<iframe src="http://localhost:8766/" '
                    'style="width:100%; height:900px; border:0; '
                    'border-radius:8px; background:#0b0b12;"></iframe>'
                )

    demo.launch(
        server_name=SERV_NAME,
        server_port=SERV_PORT,
    )


if __name__ == "__main__":
    main()