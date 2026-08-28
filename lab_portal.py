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

import math
import queue
import threading
from collections import deque

import cv2
import numpy as np

import gradio as gr

# ----------------------- Topic names (lab-real) -----------------------
MAP_TOPIC = "/lidar_slam_2d_map"
POSE_TOPIC = "/go2/stabilized/current_pose"
ODOM_TOPIC = "/go2/restamped/robot_odom"
CAMERA_TOPIC = "/frontvideostream"
BATTERY_TOPIC = "/lf/lowstate"
GOAL_TOPIC = "/goal_pose"
FT_FRAME = "lidar_map"

SERV_NAME = "0.0.0.0"
SERV_PORT = 7860

# ----------------------- import rclpy + msgs (defensive) -------------
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
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


def _placeholder(w, h, text, color=(56, 189, 248)):
    """Dark placeholder image with centred text."""
    ph = np.ones((h, w, 3), dtype=np.uint8) * 12
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(text, font, 0.6, 2)
    cv2.putText(ph, text, ((w - tw) // 2, (h + th) // 2), font, 0.6, color, 2)
    return ph


class LabRobotNode(Node):
    """Subscribes to map/pose/odom/camera/battery and renders the dashboard."""

    def __init__(self):
        super().__init__("lab_portal_node")
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
        self._bridge = CvBridge() if CvBridge else None
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

        # All subscriptions are optional: ROS waits silently for any topic.
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.map_cb, 10)
        self.create_subscription(PoseStamped, POSE_TOPIC, self.pose_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.create_subscription(Image, CAMERA_TOPIC, self.camera_cb, 10)
        if LowState is not None:
            self.create_subscription(LowState, BATTERY_TOPIC, self.battery_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self.get_logger().info(
            f"Subscribed to {MAP_TOPIC}, {POSE_TOPIC}, {ODOM_TOPIC}, "
            f"{CAMERA_TOPIC}, {BATTERY_TOPIC}"
        )

        # Publish goals from a dedicated thread. rclpy publish() must NOT be
        # called from a Gradio/anyio worker thread (it can corrupt the rclpy
        # context and crash the whole process).
        threading.Thread(target=self._goal_publish_loop, daemon=True).start()

    def _goal_publish_loop(self):
        while True:
            try:
                goal = self._goal_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.goal_pub.publish(goal)
                self.get_logger().info(
                    f"Goal published -> "
                    f"({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f})"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to publish goal: {e}")

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
        self.map_stamp = self.get_clock().now().nanoseconds / 1e9

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
        self.odom_path.append(
            (msg.pose.pose.position.x, msg.pose.pose.position.y)
        )

    def camera_cb(self, msg):
        try:
            if self._bridge is not None:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:  # manual fallback (BGR/8-bit only)
                dtype = np.uint8
                frame = np.frombuffer(msg.data, dtype=dtype)
                frame = frame.reshape((msg.height, msg.width, 3))
            self.color_frame = frame
            self.cam_stamp = self.get_clock().now().nanoseconds / 1e9
        except Exception as e:
            self.get_logger().error(f"Camera conversion failed: {e}")

    def battery_cb(self, msg):
        self.battery = msg.bms_state.soc
        self.motor_status = [m.temperature for m in msg.motor_state if m.mode == 1]
        self.roll = msg.imu_state.rpy[0]
        self.pitch = msg.imu_state.rpy[1]
        self.imu_yaw = msg.imu_state.rpy[2]
        self.voltage = msg.power_v
        self.current = msg.power_a
        self.power = self.voltage * self.current
        self.bat_stamp = self.get_clock().now().nanoseconds / 1e9

    # ---------------------- rendering ----------------------
    def draw_map(self):
        if self.cached_map_img is None:
            return _placeholder(640, 480, "WAITING FOR MAP...")

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
        if self.color_frame is None:
            return _placeholder(640, 360, "WAITING FOR CAMERA...")
        frame = self.color_frame
        h, w = frame.shape[:2]
        if w > 640 or h > 360:
            frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
        return frame

    # ---------------------- text getters ----------------------
    def get_battery_data(self):
        if self.battery is None:
            return "Waiting for battery status..."
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
            return "Waiting for motor temps..."
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
            return "Waiting for IMU..."
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
        now = self.get_clock().now().nanoseconds / 1e9
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
            return "No map yet"
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
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(wx)
        goal.pose.position.y = float(wy)
        goal.pose.orientation.w = 1.0
        self._goal_queue.put(goal)
        self.last_goal = (wx, wy)
        return f"Goal set -> ({wx:.3f}, {wy:.3f})"


def main():
    rclpy.init(args=None)
    node = LabRobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    with gr.Blocks(title="QOD Lab - Go2 Live Dashboard") as demo:
        gr.Markdown("## 🗺️ Go2 Live Dashboard (QOD Lab)")
        conn_out = gr.Textbox(label="Connection", lines=1, interactive=False)

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

    demo.launch(server_name=SERV_NAME, server_port=SERV_PORT)


if __name__ == "__main__":
    main()