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

Usage:
    source /opt/ros/humble/setup.bash
    python3 lab_portal.py            # serves on 0.0.0.0:7860
"""

import errno
import math
import os
import queue
import select
import subprocess
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
}

ASTRO_TOPICS = {
    "map":     "/astro/lidar_slam_2d_map",
    "pose":    "/go2/stabilized/current_pose",
    "odom":    "/astro/go2/restamped/robot_odom",
    "camera":  "/astro/frontvideostream",
    "battery": "/lf/lowstate",
    "goal":    "/astro/goal_pose",
}

ROBOTS = {
    "Luna": LUNA_TOPICS,
    "Astro": ASTRO_TOPICS,
}

# Luna's camera is H.264-encoded Go2FrontVideoData. We decode it with a
# persistent ffmpeg pipeline (started once per robot) fed via stdin and read
# decoded BGR frames from stdout — no repeated file-open, so it is smooth and
# cheap. ffmpeg must be installed in the container (see restore_dashboard.sh).
CAM_W, CAM_H, CAM_BUFFER_MAX = 640, 360, 6 * 1024 * 1024
# Decode only the newest slice for the file-based fallback decode.
CAM_DECODE_WINDOW = 1 * 1024 * 1024
# If the ffmpeg pipe delivers no frame for this long (s), fall back to the
# slower file-based decode so the camera never fully blanks.
CAM_PIPE_FALLBACK = 4.0

FT_FRAME = "lidar_map"

SERV_NAME = "0.0.0.0"
SERV_PORT = 7860

# ----------------------- import rclpy + msgs (defensive) -------------
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
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
"""


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
        # --- camera ---
        self.color_frame = None
        self.cam_stamp = 0.0
        self.cam_frame_ts = 0.0
        self._cam_buf = bytearray()
        self._cam_lock = threading.Lock()
        self._last_decode = 0.0
        self._ffmpeg = None
        self._ffmpeg_lock = threading.Lock()
        self._stop_cam = False
        self._feed_carry = b""
        self._synced = False
        # Primary decode: continuous ffmpeg pipe (smooth, low CPU). The
        # file-based decode stays as a rarely-used fallback thread below.
        self._start_ffmpeg_pipeline()
        threading.Thread(target=self._ffmpeg_reader, daemon=True).start()
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

    def camera_cb(self, msg):
        """Append H.264 bytes from Go2FrontVideoData to the rolling buffer and
        feed them into the persistent ffmpeg decode pipeline."""
        try:
            data = bytes(msg.data)
            with self._cam_lock:
                self._cam_buf.extend(data)
                # keep a bounded trailing window (enough for a full IDR+frames)
                if len(self._cam_buf) > CAM_BUFFER_MAX:
                    del self._cam_buf[: len(self._cam_buf) - CAM_BUFFER_MAX]
            self._pipe_feed(data)
            self.cam_stamp = self.node.get_clock().now().nanoseconds / 1e9
        except Exception as e:
            self.node.get_logger().error(f"Camera buffer failed: {e}")

    # ------------------ persistent ffmpeg pipeline ------------------
    def _start_ffmpeg_pipeline(self):
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-probesize", "1024", "-analyzeduration", "0",
            "-f", "h264", "-i", "pipe:0",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-r", "12",               # throttle output to ~12 fps (we don't
            "-flush_packets", "1",    # need the full 85fps -> far less CPU)
            "pipe:1",
        ]
        with self._ffmpeg_lock:
            try:
                self._ffmpeg = subprocess.Popen(
                    cmd, stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    bufsize=0,
                )
                os.set_blocking(self._ffmpeg.stdout.fileno(), False)
                try:
                    os.set_blocking(self._ffmpeg.stdin.fileno(), False)
                except OSError:
                    pass
                self._synced = False
            except Exception as e:
                self._ffmpeg = None
                self.node.get_logger().warn(f"[{self.name}] ffmpeg start failed: {e}")

    @staticmethod
    def _scan_idr(data):
        """Return the byte-offset of the first IDR (NAL type 5) in `data`, or -1.

        ffmpeg decoding raw H.264 only produces frames once it hits a keyframe
        (IDR). The Go2 stream is continuous, so we feed ffmpeg from the most
        recent IDR boundary to keep it perpetually in sync."""
        i = 0
        n = len(data) - 4
        while i < n:
            if data[i] == 0 and data[i + 1] == 0:
                if data[i + 2] == 1:
                    hdr = i + 3
                    sz = 3
                elif data[i + 2] == 0 and data[i + 3] == 1:
                    hdr = i + 4
                    sz = 4
                else:
                    i += 1
                    continue
                if hdr < len(data):
                    nal_type = (data[hdr] >> 5) & 0x1F
                    if nal_type == 5:  # IDR
                        return i
                i += sz
            else:
                i += 1
        return -1

    def _pipe_feed(self, data):
        """Feed H.264 into ffmpeg.

        Raw H.264 needs to start at a keyframe (IDR) or ffmpeg can never sync
        (endless 'non-existing PPS 0 referenced'). So the very first IDR we see
        after (re)starting the decoder restarts ffmpeg at that boundary; after
        that we just stream bytes continuously — ffmpeg stays in sync across
        subsequent delta frames.

        `self._feed_carry` carries buffered bytes across messages so an IDR
        split across two messages is still detected."""
        carry = self._feed_carry
        combined = carry + data
        self._feed_carry = combined[-4:]  # keep last 4 bytes for next scan
        idr = self._scan_idr(combined)
        if idr >= 0 and not self._synced:
            # First keyframe after start -> restart decoder at a clean sync point
            self._restart_ffmpeg(quiet=True)
            combined = combined[idr:]
            self._synced = True
        p = self._ffmpeg
        if p is None or p.stdin is None:
            return
        try:
            os.write(p.stdin.fileno(), combined)
        except (BlockingIOError, BrokenPipeError, OSError) as e:
            if isinstance(e, BlockingIOError):
                return  # decoder is behind live pace; drop stale bytes
            self._restart_ffmpeg()
            self._synced = False

    def _restart_ffmpeg(self, quiet=False):
        with self._ffmpeg_lock:
            old = self._ffmpeg
            self._ffmpeg = None
            if old is not None:
                try:
                    old.terminate()
                except Exception:
                    pass
        self._start_ffmpeg_pipeline()
        if not quiet:
            self.node.get_logger().info(f"[{self.name}] restarting ffmpeg pipeline")

    def _ffmpeg_reader(self):
        """Continuously read decoded BGR frames from ffmpeg stdout and publish
        them into self.color_frame."""
        size = CAM_W * CAM_H * 3
        buf = b""
        while not self._stop_cam:
            p = self._ffmpeg
            if p is None or p.stdout is None:
                if self._ffmpeg is None:
                    self._restart_ffmpeg()
                threading.Event().wait(0.5)
                continue
            try:
                r, _, _ = select.select([p.stdout], [], [], 0.2)
                if not r:
                    continue
                d = os.read(p.stdout.fileno(), 65536)
                if not d:
                    self._restart_ffmpeg()
                    buf = b""
                    continue
                buf += d
                while len(buf) >= size:
                    frame = np.frombuffer(buf[:size], dtype=np.uint8).copy()
                    frame = frame.reshape((CAM_H, CAM_W, 3))
                    buf = buf[size:]
                    self.color_frame = frame
                    self.cam_frame_ts = time.time()
            except (BrokenPipeError, OSError):
                self._restart_ffmpeg()
                buf = b""
                continue
            except Exception:
                buf = b""
                continue

    @staticmethod
    def _trim_to_keyframe(raw):
        """Trim a raw H.264 Annex-B byte buffer so it starts at the most recent
        keyframe (IDR, NAL type 5). cv2.VideoCapture needs a clean IDR at the
        start of the file or it can never sync and returns no frame."""
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
            return raw
        idr = None
        for idx, (pos, sz) in enumerate(codes):
            hdr = pos + sz
            if hdr >= len(raw):
                break
            nal_type = (raw[hdr] >> 5) & 0x1F
            if nal_type == 5:  # IDR
                idr = pos
        if idr is not None:
            return raw[idr:]
        # No IDR yet in window; fall back to start of stream (best effort)
        return raw[codes[0][0]:]

    def _camera_decode_loop(self):
        """Low-frequency fallback decode thread.

        The primary source is the continuous ffmpeg pipe. This thread only
        kicks in (file-based decode ~4s apart) if the pipe has delivered no
        frame recently, so the camera never fully blanks if ffmpeg struggles."""
        while not self._stop_cam:
            if self.cam_frame_ts and time.time() - self.cam_frame_ts < CAM_PIPE_FALLBACK:
                threading.Event().wait(2.0)
                continue
            try:
                frame = self._decode_camera_once()
                if frame is not None:
                    self.color_frame = frame
                    self.cam_frame_ts = time.time()
            except Exception as e:
                self.node.get_logger().error(f"Camera decode failed: {e}")
            threading.Event().wait(4.0)

    def _decode_camera_once(self):
        """Decode the newest H.264 window into a frame (returns numpy or None)."""
        if not self._cam_buf:
            return None
        with self._cam_lock:
            raw = bytes(self._cam_buf[-CAM_DECODE_WINDOW:])
        if not raw:
            return None
        raw = self._trim_to_keyframe(raw)
        if not raw:
            return None
        tmp = f"/tmp/{self.name}_cam.h264"
        with open(tmp, "wb") as f:
            f.write(raw)
        cap = cv2.VideoCapture(tmp)
        frame = None
        seen = 0
        while seen < 4:
            ret, f = cap.read()
            if not ret:
                break
            seen += 1
            frame = f
        cap.release()
        if frame is None:
            return None
        h, w = frame.shape[:2]
        if w > CAM_W or h > CAM_H:
            frame = cv2.resize(frame, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)
        return frame

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
            canvas = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)
            hc, wc = canvas.shape[:2]
            cv2.putText(canvas, "N", (wc // 2, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
            cv2.putText(canvas, "S", (wc // 2, hc - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
        return canvas

    def draw_camera(self):
        self.decode_camera()
        if self.color_frame is None:
            return _placeholder(640, 360, f"WAITING FOR {self.name.upper()} CAMERA...")
        frame = self.color_frame
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
        return "   ".join(parts)

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
                self.create_subscription(
                    Go2FrontVideoData, topics["camera"], robot.camera_cb, 10
                )
            if LowState is not None:
                self.create_subscription(LowState, topics["battery"], robot.battery_cb, 10)

            robot.goal_pub = self.create_publisher(PoseStamped, topics["goal"], 10)
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

    def _goal_publish_loop(self):
        while True:
            for name, robot in self.robots.items():
                try:
                    goal = robot._goal_queue.get(timeout=0.0)
                except queue.Empty:
                    continue
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
                map_img = gr.Image(label="Robot Map (click to set goal)",
                                   type="numpy")
                goal_out = gr.Textbox(label="Goal Status", lines=1)
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

        map_img.select(node.click_to_goal, None, [goal_out])

        robot_dd.change(node.set_robot, robot_dd, None)

    demo.launch(
        server_name=SERV_NAME,
        server_port=SERV_PORT,
        theme=theme,
        css=DASHBOARD_CSS,
    )


if __name__ == "__main__":
    main()