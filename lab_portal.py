"""
lab_portal.py - Minimal read-only map + pose + odom portal for the QOD lab.

Runs inside the robot_hivemind ROS 2 (Humble) container and subscribes to the
lab's real topics (bridged from the Go2 over Zenoh / domain 70):

    MAP   /lidar_slam_2d_map          (nav_msgs/msg/OccupancyGrid)
    POSE  /go2/stabilized/current_pose (geometry_msgs/msg/PoseStamped)
    ODOM  /go2/restamped/robot_odom   (nav_msgs/msg/Odometry)

Click on the map to set a navigation goal (published to /goal_pose).

This is intentionally standalone (no unitree_sdk2py, no cameras, no DDS
clients) so it boots cleanly on topics that actually exist in this lab.

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
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry

import gradio as gr

# ----------------------- Topic names (lab-real) -----------------------
MAP_TOPIC = "/lidar_slam_2d_map"
POSE_TOPIC = "/go2/stabilized/current_pose"
ODOM_TOPIC = "/go2/restamped/robot_odom"
GOAL_TOPIC = "/goal_pose"
FT_FRAME = "lidar_map"

SERV_NAME = "0.0.0.0"
SERV_PORT = 7860


class LabRobotNode(Node):
    """Subscribes to map + pose + odom and renders a 2D map with the robot."""

    def __init__(self):
        super().__init__("lab_portal_node")
        self.map_info = None
        self.map = None
        self.robot_pose = None
        self.yaw = 0.0
        self.odom_path = deque(maxlen=2000)
        self.cached_map_img = None
        self.cached_scale = None
        self.cached_meta = None
        self.pose_latest = "Waiting for pose..."
        self.last_goal = None
        self._goal_queue = queue.Queue()

        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.map_cb, 10)
        self.create_subscription(PoseStamped, POSE_TOPIC, self.pose_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self.get_logger().info(
            f"Subscribed to {MAP_TOPIC}, {POSE_TOPIC}, {ODOM_TOPIC}"
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

    # ---------------------- rendering ----------------------
    def draw(self):
        if self.cached_map_img is None:
            ph = np.ones((400, 400, 3), dtype=np.uint8) * 12
            cv2.putText(ph, "WAITING FOR MAP...", (40, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (56, 189, 248), 2)
            return ph

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

    with gr.Blocks(title="QOD Lab - Go2 Map Portal") as demo:
        gr.Markdown("## 🗺️ Go2 Live Map (QOD Lab)")
        map_img = gr.Image(label="Robot Map (click to set goal)", type="numpy")
        goal_out = gr.Textbox(label="Goal Status")
        pose_out = gr.Textbox(label="📍 Pose", lines=1)

        def tick_map():
            return node.draw()

        def tick_pose():
            pos = node.pose_latest
            if node.last_goal:
                return f"{pos}\n🎯 last goal: ({node.last_goal[0]:.2f}, {node.last_goal[1]:.2f})"
            return pos

        map_timer = gr.Timer(0.1)
        map_timer.tick(tick_map, outputs=map_img)
        pose_timer = gr.Timer(0.5)
        pose_timer.tick(tick_pose, outputs=pose_out)

        map_img.select(node.click_to_goal, None, [goal_out])

    demo.launch(server_name=SERV_NAME, server_port=SERV_PORT)


if __name__ == "__main__":
    main()
