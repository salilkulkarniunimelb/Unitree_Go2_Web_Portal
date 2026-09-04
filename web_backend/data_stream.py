import time
import cv2
import base64
import rclpy
import numpy as np
import torch
from geometry_msgs.msg import Twist
from web_backend.camera import RosImageSubscriber
from web_backend.camera_rs import RosRSImageSubscriber
# from web_backend.lidar import RosLidarSubscriber, check_safety_breach
from web_backend.map_2d import Nav2MapWithRobot
from web_backend.controller import Go2WirelessController
# from web_backend.chatbot import GenerateResponse
# from web_backend.events import EventManager
from web_backend.nav2 import Nav2Controller
from web_backend.bm_status import BatteryMotorStatus
from web_backend.action_sub import ActionSubscriber
from web_backend.audio_sub import AudioSubscriber
from web_backend.led_sub import LedSubscriber
from backends.dds_backend import DDSBackend
from backends.backend_factory import BackendFactory
from web_backend.llm_describer import GenerateResponse
from config import UNITREE_ROS2_SETUP_SH_PATH, CMD_VEL_PUB_TOPIC_NAME, CMD_VEL_PUB_TOPIC_TYPE, TTS_LANGUAGE, UPDATE_INTERVAL, IMAGE_HEIGHT, IMAGE_WIDTH, LFLOWCMD, WAYPOINT_FILE, MODE
import json
from gtts import gTTS
import tempfile
import os

# from web_backend.april_tag import AprilTagInspector



# ---Display Live Stream -----------------------------------------------------
class DataStream:

    def __init__(self,interface,camera_topic):
        self.system = GenerateResponse()
        # self.TF = TFGraph()
        
        #backend = DDSBackend()
        self.backend_factory = BackendFactory()
        backend = self.backend_factory.load_backend(MODE)
        backend_clients = backend.initialize()
        self.action_subscriber = ActionSubscriber(backend_clients)
        self.audio_subscriber = AudioSubscriber(backend_clients)
        self.led_subscriber = LedSubscriber(backend_clients)



        self.rs_image_subscriber = RosRSImageSubscriber() 
        self.image_subscriber = RosImageSubscriber(interface,camera_topic)
        self.bm_subscriber=BatteryMotorStatus(lf_lowcmd_topic_name=LFLOWCMD)
        #self.lidar_subscriber = RosLidarSubscriber(lidar)
        self.map_subscriber = Nav2MapWithRobot()
        self.nav2_controller = Nav2Controller() 
        self.safety_breach=None
        


        # #TODO
        # # After your existing subscribers are set up:
        # self.apriltag_inspector = AprilTagInspector(
        #     llm_system=self.system,                    # your GenerateResponse()
        #     image_subscriber=self.image_subscriber     # your RosImageSubscriber()
        # )


        # self.event_manager = EventManager()
        self.wirelesscontroller = Go2WirelessController(topic=CMD_VEL_PUB_TOPIC_NAME, msg_type=CMD_VEL_PUB_TOPIC_TYPE, setup_sh_path=UNITREE_ROS2_SETUP_SH_PATH)
        self.description_cache = "説明待ち..."
        self.desc_update_interval = UPDATE_INTERVAL  # seconds
        self.sleep_interval=0.05
        self.image_width=IMAGE_WIDTH
        self.image_height=IMAGE_HEIGHT
        self.frame=None
        self.stream=True
        self.audio=None

    def publish_commands(self):
            """Continuously publish base_vel_cmd_input to /cmd_vel"""
            rate_hz = 10
            rate = 1.0 / rate_hz
            while rclpy.ok():
                msg = Twist()
                msg.linear.x = float(self.base_vel_cmd_input[0, 0])
                msg.linear.y = float(self.base_vel_cmd_input[0, 1])
                msg.angular.z = float(self.base_vel_cmd_input[0, 2])
                self.cmd_publisher.publish(msg)
                time.sleep(rate)


    def run_action(self,action_name):
        success = self.action_subscriber.send_action_command(action_name)
        return f"Action '{action_name}' queued" if success else f"Action '{action_name}' not found"



    def get_description_from_image(self, image: np.ndarray):
        # Ensure RGB before encoding
        if image.shape[2] == 3:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            image_rgb = image
        _, buffer = cv2.imencode('.jpg', image_rgb)
        img_b64 = base64.b64encode(buffer).decode('utf-8')
        data_url = f"data:image/jpeg;base64,{img_b64}"
        user_msg="What's in this image? Tell me briefly. Also warn user of some warning signs or some dangerous situation."
        return self.system.llm_response(user_msg, data_url)

    
                    # ---- Text-to-Speech function ----
    def narrate_text(self,text):
        if not text.strip():
            return None
        tts = gTTS(text, lang=TTS_LANGUAGE)  # 'ja' works well for mixed EN/JP
        tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".mp3")
        tts.save(tmp.name)
        return tmp.name

    # def update_description(self):
    #     while rclpy.ok():
    #         frame = self.image_subscriber.color_frame

    #         if frame is not None:
    #             rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #             resized_frame = cv2.resize(rgb_frame, (self.image_width, self.image_height))
    #             self.description_cache = self.get_description_from_image(resized_frame)
    #         else:
    #             self.description_cache = self.system.llm_response(None)
            #print(self.description_cache)

            #self.audio = self.narrate_text(self.description_cache)
            # if frame is not None:
            #     # Convert to RGB for LLM description
            #     rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #     resized_frame = cv2.resize(rgb_frame, (self.image_width, self.image_height))
            #     self.description_cache = self.get_description_from_image(resized_frame)
            #     self.audio=self.narrate_text(self.description_cache)
            # time.sleep(self.desc_update_interval)

    


    def camera_status(self):
        """Return a small text snapshot of every camera source for diagnostics."""
        img = self.image_subscriber
        rs = self.rs_image_subscriber
        main_ok = img.color_frame is not None
        yolo_ok = img.detection_frame is not None
        rs_c_ok = rs.color_frame is not None
        rs_d_ok = rs.depth_frame is not None
        return (
            f"Main camera: {'OK' if main_ok else 'STALE'} "
            f"(frames={img.frame_count if main_ok else 0}) | "
            f"YOLO: {'OK' if yolo_ok else 'STALE'} | "
            f"RS color: {'OK' if rs_c_ok else 'STALE'} | "
            f"RS depth: {'OK' if rs_d_ok else 'STALE'} | "
            f"GStreamer: {self._gst_state()}"
        )

    def _gst_state(self):
        try:
            p = self.image_subscriber.gst_process
            if p is None:
                return "not-started"
            return "running" if p.poll() is None else f"EXITED(rc={p.returncode})"
        except Exception:
            return "unknown"

    def live_cam_feed(self,mode:str):
        stale_logged = False

        while True:
            if mode=="color_image":
                self.frame = self.image_subscriber.color_frame

                if self.frame is None:
                    time.sleep(self.sleep_interval)
                    continue
                else:
                    self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                    self.frame = cv2.resize(self.frame, (self.image_width, self.image_height))

            elif mode=="yolo_detection":
                self.frame = self.image_subscriber.detection_frame

                if self.frame is None:
                    time.sleep(self.sleep_interval)
                    continue
                else:
                    self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                    self.frame = cv2.resize(self.frame, (self.image_width, self.image_height))


            elif mode == "rs_color_image":
                self.frame = self.rs_image_subscriber.color_frame

                if self.frame is None:
                    time.sleep(self.sleep_interval)
                    continue
                else:
                    self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                    self.frame = cv2.resize(self.frame, (self.image_width, self.image_height))

            elif mode == "rs_depth_image":
                self.frame = self.rs_image_subscriber.depth_frame

                if self.frame is None:
                    time.sleep(self.sleep_interval)
                    continue
                else:
                    min_depth_mm = 300    # 0.3m — closer than this is noise
                    max_depth_mm = 3000   # 3m — practical D435i limit

                    self.frame = np.clip(self.frame, min_depth_mm, max_depth_mm)
                    self.frame = ((self.frame - min_depth_mm) / (max_depth_mm - min_depth_mm) * 255).astype(np.uint8)
                    self.frame = cv2.applyColorMap(self.frame, cv2.COLORMAP_JET)  # uncomment this
                    self.frame = cv2.resize(self.frame, (self.image_width, self.image_height))
            

            elif mode=="rs_yolo_detection":
                self.frame = self.rs_image_subscriber.detection_frame

                if self.frame is None:
                    time.sleep(self.sleep_interval)
                    continue
                else:
                    self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                    self.frame = cv2.resize(self.frame, (self.image_width, self.image_height))

            if self.frame is not None:
                stale_logged = False
            elif not stale_logged:
                # Log once per period when a feed dries up so we can tell whether
                # the issue is the camera pipeline vs the dashboard rendering.
                print(f"[live_cam_feed:{mode}] No frame received (camera stale). "
                      f"color_frame={self.image_subscriber.color_frame is not None}, "
                      f"detection_frame={self.image_subscriber.detection_frame is not None}, "
                      f"rs_color={self.rs_image_subscriber.color_frame is not None}, "
                      f"rs_depth={self.rs_image_subscriber.depth_frame is not None}")
                stale_logged = True

            yield self.frame, self.description_cache
            time.sleep(self.sleep_interval)


        
        # ==============================
    # 📂 WAYPOINT HELPERS
    # ==============================
    def load_data(self):
        if not os.path.exists(WAYPOINT_FILE):
            return {}
        with open(WAYPOINT_FILE, "r") as f:
            return json.load(f)

    def save_data(self,data):
        with open(WAYPOINT_FILE, "w") as f:
            json.dump(data, f, indent=4)

    def wp_choices(self):
        return list(self.load_data().keys())
    
    def get_robot_pose(self):
        try:
            x = self.map_subscriber.robot_pose.position.x
            y = self.map_subscriber.robot_pose.position.y
            yaw = self.bm_subscriber.yaw
            return x, y, yaw
        except:
            return None, None, None
        

        
    def audio_command(self, action: str, value=None):
        """
        Routes audio requests from the web portal to the AudioSubscriber node.
        """
        try:
            if action == "play":
                # value is the filename string
                self.audio_subscriber.play_sound(value)
                return {"status": "success", "message": f"Playing {value}"}

            elif action == "stop":
                self.audio_subscriber.stop_sound()
                return {"status": "success", "message": "Audio stopped"}

            elif action == "volume":
                # Convert value to int if it's coming from a slider/string
                vol = int(value)
                self.audio_subscriber.change_volume(vol)
                return {"status": "success", "message": f"Volume set to {vol}"}

            else:
                return {"status": "error", "message": f"Unknown audio action: {action}"}

        except Exception as e:
            print(f"Error in audio_command: {e}")
            return {"status": "error", "message": str(e)}
        
    def led_command(self, action: str, value=None):
        """
        Routes LED requests from the web portal to the LedSubscriber node methods.
        """
        try:
            if action == "color":
                self.led_subscriber.change_color(value)
                return {"status": "success", "message": f"Color set to {value}"}

            elif action == "on":
                self.led_subscriber.led_on()
                return {"status": "success", "message": "LED turned ON"}

            elif action == "off":
                self.led_subscriber.led_off()
                return {"status": "success", "message": "LED turned OFF"}

            elif action == "brightness":
                # Ensure value is an integer for the led_client
                brightness_val = int(value)
                self.led_subscriber.change_brightness(brightness_val)
                return {"status": "success", "message": f"Brightness set to {brightness_val}"}

            else:
                self.get_logger().warn(f"Unknown LED action received: {action}")
                return {"status": "error", "message": f"Unknown action: {action}"}

        except Exception as e:
            self.get_logger().error(f"Error executing LED command: {e}")
            return {"status": "error", "message": str(e)}
    