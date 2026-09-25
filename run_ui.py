"""Standalone launcher — runs the Gradio dashboard UI without ROS2."""
import sys, os, types, shutil, base64

# ── Stub every ROS2-only module before any project import ──────────────
_ros_stub = types.ModuleType("rclpy")
_ros_stub.init = lambda *a, **kw: None
_ros_stub.shutdown = lambda *a, **kw: None
_ros_stub.ok = lambda *a, **kw: True

_exec = types.ModuleType("rclpy.executors")
class _DummyExecutor:
    def add_node(self, *a): pass
    def spin(self):
        import time as _t
        while True:
            _t.sleep(999)
_exec.MultiThreadedExecutor = _DummyExecutor
_ros_stub.executors = _exec

for name in ("rclpy.node",):
    mod = types.ModuleType(name)
    class _Node:
        def __init__(self, *a, **kw): pass
    mod.Node = _Node
    sys.modules[name] = mod

sys.modules["rclpy"] = _ros_stub
sys.modules["rclpy.executors"] = _exec

# Stub ROS message packages
for pkg in [
    "sensor_msgs", "sensor_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg",
    "nav_msgs", "nav_msgs.msg",
    "visualization_msgs", "visualization_msgs.msg",
    "std_msgs", "std_msgs.msg",
    "tf2_msgs", "tf2_msgs.msg",
    "rosgraph_msgs", "rosgraph_msgs.msg",
    "unitree_go", "unitree_go.msg",
    "nav2_msgs", "nav2_msgs.action",
    "ackermann_msgs", "ackermann_msgs.msg",
    "builtin_interfaces",
]:
    mod = types.ModuleType(pkg)
    sys.modules[pkg] = mod

# ── Project imports ────────────────────────────────────────────────────
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from config import (
    SOUNDS_DIR, ROBOT, ROSBRIDGE_IP, ROSBRIDGE_PORT, MODE, INTERFACE,
    CAMERA_TOPIC_NAME, SHOW_LIDAR, SHOW_DESCRIPTION
)

os.makedirs(SOUNDS_DIR, exist_ok=True)

# ── Mock launcher that provides the same API as DataStream ─────────────
import numpy as np

class MockSubscriber:
    def __init__(self, name="mock"):
        self.name = name
    def get_motor_data(self):
        return "Motor temps: N/A (demo mode)"
    def get_battery_data(self):
        return "Battery: --% (demo)"
    def get_orientation_data(self):
        return "Roll: 0  Pitch: 0  Yaw: 0"
    def get_system_state_data(self):
        return "System: demo mode"

class MockImageSub:
    def __init__(self):
        self.name = "image_sub"

class MockRSImageSub:
    def __init__(self):
        self.name = "rs_image_sub"

class MockMapSub:
    def __init__(self):
        self.name = "map_sub"
    def draw_gradio(self):
        img = np.zeros((275, 400, 3), dtype=np.uint8)
        img[130:145, 195:210] = [0, 200, 255]
        return img
    def get_live_data(self):
        return "x: 0.00, y: 0.00", "z: 0.00", "v: 0.00 m/s"

class MockNav2:
    def __init__(self):
        self.name = "nav2"
    def go_to(self, wp): pass
    def run_route(self, wps): pass

class MockBM:
    def __init__(self):
        self._sub = MockSubscriber()
    def get_motor_data(self):
        return self._sub.get_motor_data()
    def get_battery_data(self):
        return self._sub.get_battery_data()
    def get_orientation_data(self):
        return self._sub.get_orientation_data()
    def get_system_state_data(self):
        return self._sub.get_system_state_data()

class MockAudio:
    def __init__(self):
        self.name = "audio"
    def play_sound(self, name): pass
    def stop_sound(self): pass
    def change_volume(self, vol): pass
    def mic_stream(self, *a, **kw): pass

class MockAction:
    available_actions = ["stand", "sit", "hello", "dance", "wave"]

class MockLED:
    def led_off(self): pass
    def change_color(self, c): pass
    def change_brightness(self, b): pass

class MockWireless:
    def pub_wirelesscontroller(self, *a): pass
    def move_forward(self): pass
    def move_backward(self): pass
    def move_left(self): pass
    def move_right(self): pass
    def rotate_left(self): pass
    def rotate_right(self): pass
    def stop_robot(self): pass

class MockLauncher:
    def __init__(self):
        self.image_subscriber = MockImageSub()
        self.rs_image_subscriber = MockRSImageSub()
        self.map_subscriber = MockMapSub()
        self.nav2_controller = MockNav2()
        self.bm_subscriber = MockBM()
        self.audio_subscriber = MockAudio()
        self.action_subscriber = MockAction()
        self.led_subscriber = MockLED()
        self.wirelesscontroller = MockWireless()

    def run_action(self, action):
        return f"Action '{action}' sent (demo mode)"

    def live_cam_feed(self, mode):
        """Yields placeholder frames."""
        import time, cv2
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, f"Camera: {mode} (demo)", (50, 240),
                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        while True:
            yield placeholder, f"[{mode}] Demo feed — no ROS2 connection"
            time.sleep(1)

    def load_data(self):
        import json
        wp_file = "data/waypoints/waypoints.json"
        if os.path.exists(wp_file):
            try:
                with open(wp_file) as f:
                    return json.load(f)
            except (json.JSONDecodeError, ValueError):
                return {}
        return {}

    def save_data(self, data):
        import json
        wp_file = "data/waypoints/waypoints.json"
        os.makedirs(os.path.dirname(wp_file), exist_ok=True)
        with open(wp_file, "w") as f:
            json.dump(data, f, indent=2)

    def wp_choices(self):
        return list(self.load_data().keys())

    def get_robot_pose(self):
        return 0.0, 0.0, 0.0

# ── Stub the ROS2 topic/service modules used by dev.py ────────────────
ros_topics_mod = types.ModuleType("web_backend.ros2_topics")
ros_topics_mod.view_list_topics = lambda: ["/demo/topic1", "/demo/topic2"]
ros_topics_mod.get_topic_operations = lambda: ["info", "echo"]
ros_topics_mod.execute_topic_operation = lambda *a: "No ROS2 connection (demo)"
ros_topics_mod.build_ros_graph_snapshot = lambda *a: None
sys.modules["web_backend.ros2_topics"] = ros_topics_mod

ros_services_mod = types.ModuleType("web_backend.ros2_services")
ros_services_mod.view_list_services = lambda: ["/demo/service1"]
ros_services_mod.get_service_operations = lambda: ["info", "call"]
ros_services_mod.execute_service_operation = lambda *a: "No ROS2 connection (demo)"
ros_services_mod.build_service_graph_snapshot = lambda *a: None
sys.modules["web_backend.ros2_services"] = ros_services_mod

mcp_agent_mod = types.ModuleType("web_backend.mcp_agent")
async def _mock_stream(msg):
    yield {"type": "thinking", "text": "Analyzing..."}
    yield {"type": "content", "text": "This is a demo response. Connect to ROS2 for live data."}
mcp_agent_mod.stream_agent = _mock_stream
sys.modules["web_backend.mcp_agent"] = mcp_agent_mod

# ── Import Gradio frontend ─────────────────────────────────────────────
import gradio as gr
from web_frontend.index import get_index_page
from web_frontend.action import get_action_page
from web_frontend.dev import get_dev_page
from web_frontend.chatbot import get_chatbot_page


def load_css_file(path):
    with open(path, "r") as f:
        return f.read()


def logo_data_uri():
    with open("assets/logo.png", "rb") as f:
        b64 = base64.b64encode(f.read()).decode()
    return f"data:image/png;base64,{b64}"


def main():
    launcher = MockLauncher()

    with gr.Blocks(title="MCP Web — Demo Mode", css=load_css_file("web_frontend/style.css")) as demo:
        gr.HTML(
            f"""
            <div style="display:flex; align-items:center; gap:12px; padding:8px 4px;">
                <img src="{logo_data_uri()}" alt="logo" style="height:40px; width:auto;" />
                <div style="font-size:15px;">
                    ROBOT <code>{ROBOT}</code> &nbsp;|&nbsp;
                    ROSBRIDGE <code>{ROSBRIDGE_IP}:{ROSBRIDGE_PORT}</code> &nbsp;|&nbsp;
                    MODE <code>{MODE}</code> &nbsp;|&nbsp; INTERFACE <code>{INTERFACE}</code>
                    <br/><span style="font-size:13px; color:#e67e22;">⚠️ Running in demo mode (no ROS2 connection)</span>
                </div>
            </div>
            """
        )

        with gr.Tabs():
            with gr.Tab("Main"):
                get_index_page(demo, launcher)
            with gr.Tab("Actions"):
                get_action_page(demo, launcher)
            with gr.Tab("Development"):
                get_dev_page()
            with gr.Tab("🤖 AI Assistant"):
                get_chatbot_page()

        demo.launch(
            server_name="127.0.0.1",
            server_port=7860,
            share=False,
        )


if __name__ == "__main__":
    main()
