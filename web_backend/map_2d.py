from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from unitree_go.msg import SportModeState
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
from config import POSE, MAP, ODOM, SPORTS
import math
from collections import deque

class Nav2MapWithRobot(Node):
    def __init__(self):
        super().__init__('nav2_map_with_robot')

        # Map data
        self.map = None
        self.map_info = None

        # Robot state
        self.robot_pose = None
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.get_logger().info("Started 2D OccupancyMap stream")

        # Odom trail
        self.odom_path = deque(maxlen=1000)


        self.cached_map_img = None
        self.cached_scale = None
        self.cached_origin = None

        # -------------------- SUBSCRIBERS --------------------
        self.create_subscription(
            OccupancyGrid,
            MAP,
            self.map_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            POSE,
            self.pose_callback,
            10
        )

        self.create_subscription(
            Odometry,
            ODOM,
            self.odom_callback,
            10
        )

        self.create_subscription(
            SportModeState,
            SPORTS,
            self.sports_callback,
            10
        )

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )

        map_data = np.flipud(self.map)
        h, w = map_data.shape

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        display_map = np.ones((h, w), dtype=np.float32)
        display_map[map_data == -1] = 0.5
        display_map[map_data == 0] = 1.0
        display_map[map_data > 0] = 0.0

        gray = (display_map * 255).astype(np.uint8)
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        canvas_size = 800
        scale = canvas_size / max(w, h)

        img = cv2.resize(
            img,
            (int(w * scale), int(h * scale)),
            interpolation=cv2.INTER_NEAREST
        )

        canvas = np.ones((canvas_size, canvas_size, 3), dtype=np.uint8) * 255
        ox = (canvas_size - img.shape[1]) // 2
        oy = (canvas_size - img.shape[0]) // 2
        canvas[oy:oy + img.shape[0], ox:ox + img.shape[1]] = img

        self.cached_map_img = canvas
        self.cached_scale = scale
        self.cached_origin = (origin_x, origin_y, h, ox, oy)


    def pose_callback(self, msg):
        self.robot_pose = msg.pose
        q = self.robot_pose.orientation

        # Quaternion → Yaw
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_path.append((x, y))

        # self.linear_velocity = msg.twist.twist.linear.x
        # self.angular_velocity = msg.twist.twist.angular.z



    def sports_callback(self, msg):
        # Linear velocity (use forward speed)
        vx = msg.velocity[0]
        vy = msg.velocity[1]


        self.linear_velocity = math.sqrt(vx**2 + vy**2)

        self.angular_velocity = msg.yaw_speed
    # =========================================================
    # DATA FOR GRADIO TEXT BOXES
    # =========================================================

    def get_live_data(self):
        if self.robot_pose is None:
            return "Waiting...", "Waiting...", "Waiting..."

        pos = self.robot_pose.position

        position_text = f"x: {pos.x:.3f}\t y: {pos.y:.3f}"
        orientation_text = f"yaw: {self.yaw:.3f} rad"
        velocity_text = f"🏎️linear: {self.linear_velocity:.3f} m/s\n 🌀angular: {self.angular_velocity:.3f} rad/s"

        return position_text, orientation_text, velocity_text


    def draw_gradio(self):
        if self.cached_map_img is None:
            return None

        canvas = self.cached_map_img.copy()

        scale = self.cached_scale
        origin_x, origin_y, h, ox, oy = self.cached_origin


        # ---------------- ODOM TRAIL ----------------
        if self.odom_path:
            pts = list(self.odom_path)   # 🔥 FIX

            prev = None
            for wx, wy in pts:
                mx = (wx - origin_x) / self.map_info.resolution
                my = (wy - origin_y) / self.map_info.resolution
                my = h - 1 - my

                px = int(mx * scale) + ox
                py = int(my * scale) + oy

                if prev is not None:
                    cv2.line(canvas, prev, (px, py), (0, 180, 0), 2)
                prev = (px, py)

        # ---------------- ROBOT ----------------
        if self.robot_pose is not None:
            wx = self.robot_pose.position.x
            wy = self.robot_pose.position.y

            mx = (wx - origin_x) / self.map_info.resolution
            my = (wy - origin_y) / self.map_info.resolution
            my = h - 1 - my

            px = int(mx * scale) + ox
            py = int(my * scale) + oy

            radius = 10

            cv2.circle(canvas, (px, py), radius, (255, 0, 0), -1)

            end_x = int(px + 30 * math.cos(self.yaw))
            end_y = int(py - 30 * math.sin(self.yaw))

            cv2.arrowedLine(canvas, (px, py), (end_x, end_y), (0, 0, 0), 3)
            canvas = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)#ROTATE_90_COUNTERCLOCKWISE

            h_c, w_c = canvas.shape[:2]
            cv2.putText(canvas, 'S', (w_c // 2, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            cv2.putText(canvas, 'N', (w_c // 2, h_c - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        return canvas

    def click_to_world(self, px, py):
        """Convert a click in the rotated Gradio canvas image to map-frame world coords.

        The draw_gradio() canvas is 800x800 and is rotated 90deg CLOCKWISE before
        display. This mirrors the transform used in get_raw_map() (the inverse of the
        world->pixel mapping there) so a click lands on the correct map location.
        """
        if self.map_info is None or self.robot_pose is None:
            return None, None

        canvas_size = 800
        scale = self.cached_scale
        origin_x, origin_y, h, ox, oy = self.cached_origin
        res = self.map_info.resolution

        # The displayed image is the 800x800 canvas rotated 90deg CLOCKWISE.
        # Forward (draw_gradio): displayed (px,py) = (cy, canvas_size-1-cx).
        # Inverse: original canvas coords:
        cx = canvas_size - 1 - float(py)
        cy = float(px)

        # Undo canvas centering / scaling (map placed into the 800x800 canvas).
        mx = (cx - ox) / scale
        my = (cy - oy) / scale

        # Undo the Y-flip from draw_gradio (my = h - 1 - my) for map pixel row.
        my = h - 1 - my

        # Back to world coordinates.
        wx = mx * res + origin_x
        wy = my * res + origin_y
        return wx, wy

    def get_raw_map(self):
        if self.map_info is None or self.map is None:
            return None

        h = self.map_info.height
        w = self.map_info.width
        res = self.map_info.resolution
        ox_map = self.map_info.origin.position.x
        oy_map = self.map_info.origin.position.y

        def transform_pt(wx, wy):
            # 1. Convert world to map pixel coordinates
            mx = (wx - ox_map) / res
            my = (wy - oy_map) / res
            
            # 2. Apply the Y-flip seen in draw_gradio (my = h - 1 - my)
            my_flipped = h - 1 - my
            
            # 3. Apply 90-degree Clockwise Rotation
            # In a 90 CW rotation: 
            # new_x = flipped_y
            # new_y = width - 1 - mx
            rx = my_flipped
            ry = w - 1 - mx
            return rx, ry

        # Transform Robot Pose
        rx, ry = (0, 0)
        rotated_yaw = self.yaw
        if self.robot_pose:
            rx, ry = transform_pt(self.robot_pose.position.x, self.robot_pose.position.y)
            # Adjust yaw for 90-degree CW rotation (-PI/2)
            rotated_yaw = self.yaw - (math.pi / 2)

        # Transform Path
        rotated_path = []
        if self.odom_path:
            for wx, wy in self.odom_path:
                rotated_path.append(transform_pt(wx, wy))

        # Note: If you are sending the raw grid data, you must also rotate the numpy array
        # to match the coordinates, otherwise the "dots" won't line up with the "map".
        rotated_data = np.rot90(self.map, k=-1).flatten().tolist() 

        return {
            "width": h,  # Swapped because of 90deg rotation
            "height": w, # Swapped because of 90deg rotation
            "resolution": res,
            "origin_x": ox_map,
            "origin_y": oy_map,
            "data": rotated_data, 
            "robot": {
                "x": rx,
                "y": ry,
                "yaw": rotated_yaw
            },
            "path": rotated_path
        }

    # def draw_gradio(self):
    #     """
    #     Returns a processed numpy image. 
    #     Critical Fix: Returns a blank placeholder if map is None to prevent cv2.imencode error.
    #     """
    #     if self.cached_map_img is None:
    #         # Create a 'System Initializing' placeholder image
    #         placeholder = np.ones((800, 800, 3), dtype=np.uint8) * 10
    #         cv2.putText(placeholder, "INITIALIZING DDS BRIDGE...", (180, 400), 
    #                     cv2.FONT_HERSHEY_SIMPLEX, 1, (56, 189, 248), 2)
    #         return placeholder

    #     canvas = self.cached_map_img.copy()
    #     scale = self.cached_scale
    #     origin_x, origin_y, h, ox, oy = self.cached_origin

    #     # ---------------- ODOM TRAIL ----------------
    #     if self.odom_path:
    #         pts = list(self.odom_path)
    #         prev = None
    #         for wx, wy in pts:
    #             mx = (wx - origin_x) / self.map_info.resolution
    #             my = (wy - origin_y) / self.map_info.resolution
    #             my = h - 1 - my

    #             px = int(mx * scale) + ox
    #             py = int(my * scale) + oy

    #             if prev is not None:
    #                 # Blue neon trail
    #                 cv2.line(canvas, prev, (px, py), (248, 189, 56), 1, cv2.LINE_AA)
    #             prev = (px, py)

    #     # ---------------- ROBOT ----------------
    #     if self.robot_pose is not None:
    #         wx = self.robot_pose.position.x
    #         wy = self.robot_pose.position.y

    #         mx = (wx - origin_x) / self.map_info.resolution
    #         my = (wy - origin_y) / self.map_info.resolution
    #         my = h - 1 - my

    #         px = int(mx * scale) + ox
    #         py = int(my * scale) + oy

    #         # Robot Marker (Blue Circle with black border)
    #         cv2.circle(canvas, (px, py), 8, (248, 189, 56), -1, cv2.LINE_AA)
    #         cv2.circle(canvas, (px, py), 9, (0, 0, 0), 1, cv2.LINE_AA)

    #         # Direction Arrow
    #         end_x = int(px + 25 * math.cos(self.yaw))
    #         end_y = int(py - 25 * math.sin(self.yaw))
    #         cv2.arrowedLine(canvas, (px, py), (end_x, end_y), (255, 255, 255), 2, tipLength=0.3)
            
    #         # Rotate for Portal Orientation
    #         canvas = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)

    #         h_c, w_c = canvas.shape[:2]
    #         cv2.putText(canvas, 'N', (w_c // 2, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    #         cv2.putText(canvas, 'S', (w_c // 2, h_c - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    #     return canvas