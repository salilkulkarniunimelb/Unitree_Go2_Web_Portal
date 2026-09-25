import time
import cv2
import numpy as np
import subprocess
import threading
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from config import YOLO_MODEL, YOLO_MODE


class RosImageSubscriber(Node):
    def __init__(self, interface:str, topic_name: str):
        super().__init__('image_subscriber')

        # Match the REAL Robot State this topic is published on (BEST_EFFORT, VOLATILE)
        # so frames are not silently dropped due to a QoS mismatch.
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            self.qos_profile
        )

        self.bridge = CvBridge()
        self.color_frame = None
        self.model = YOLO(YOLO_MODEL)  # or yolov8n.pt
        self.detection_frame = None

        self.depth_frame = None
        self.segmentation_frame = None
        self.interface = interface
        self.topic_name = topic_name
        self.frame_count = 0

        self._stop = False
        self._restart_lock = threading.Lock()

        # Start GStreamer publisher in background thread
        self._start_gstreamer_publisher(topic_name)

    def _start_gstreamer_publisher(self, topic_name):
        """Start GStreamer subprocess to publish camera frames"""
        self.width = 1280
        self.height = 720

        gst_cmd = [
            'gst-launch-1.0', '-q',
            'udpsrc', 'address=230.1.1.1', 'port=1720', f'multicast-iface={self.interface}',
            '!', 'application/x-rtp,media=video,encoding-name=H264',
            '!', 'rtph264depay',
            '!', 'h264parse',
            '!', 'avdec_h264',
            '!', 'videoconvert',
            '!', f'video/x-raw,format=BGR,width={self.width},height={self.height}',
            '!', 'fdsink'
        ]

        try:
            self.gst_process = subprocess.Popen(
                gst_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=self.width * self.height * 3 * 10
            )
            self.get_logger().info("Started RosImageSubscriber camera stream")

            # Only create the publisher once (avoid rebuild errors on restart).
            if not hasattr(self, 'publisher_'):
                self.publisher_ = self.create_publisher(Image, topic_name, self.qos_profile)

            self.gst_thread = threading.Thread(target=self._read_and_publish_frames, daemon=True)
            self.gst_thread.start()
            self.get_logger().info(f"GStreamer PID={self.gst_process.pid} started for topic {topic_name}")

        except Exception as e:
            self.get_logger().error(f"Failed to start RosImageSubscriber camera stream: {e}")

    def _read_and_publish_frames(self):
        """Read frames from GStreamer and publish to ROS2 topic, auto-restarting on crash or stall."""
        import select

        frame_size = self.width * self.height * 3
        consecutive_reads = 0
        stall_timeout = 5.0  # seconds without any bytes before we treat it as stalled
        last_ok = time.time()

        while not self._stop:
            try:
                # Wait for usable bytes on the pipe with a timeout so a live-but-silent
                # pipeline (wrong interface / no multicast stream) is detected & restarted.
                rlist, _, _ = select.select([self.gst_process.stdout], [], [], stall_timeout)
                if not rlist:
                    raise TimeoutError(
                        f"No camera bytes for {stall_timeout}s (silent pipeline). "
                        f"Check INTERFACE={self.interface} and the robot's multicast stream."
                    )
                if self.gst_process.poll() is not None:
                    raise EOFError("GStreamer process exited")

                raw_frame = self.gst_process.stdout.read(frame_size)

                if len(raw_frame) == frame_size:
                    frame = np.frombuffer(raw_frame, dtype=np.uint8)
                    frame = frame.reshape((self.height, self.width, 3))

                    resized = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
                    img_msg = self.bridge.cv2_to_imgmsg(resized, encoding="bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "camera_link"
                    self.publisher_.publish(img_msg)
                    consecutive_reads = 0
                    last_ok = time.time()

                elif len(raw_frame) == 0:
                    # EOF -> stream closed/crashed
                    raise EOFError("GStreamer stream closed (EOF)")

            except Exception as e:
                self.get_logger().error(f"Error in Go2 Camera GStreamer thread: {e}")
                consecutive_reads += 1

                # Give the process a moment to be reaped, then restart.
                if not self._stop:
                    self._restart_gstreamer()

                # If it fails continuously, back off so we don't spin on CPU.
                time.sleep(min(1.0, 0.5 * consecutive_reads))

    def _restart_gstreamer(self):
        """Terminate a crashed GStreamer process and start a fresh one."""
        with self._restart_lock:
            if self._stop:
                return
            try:
                if hasattr(self, 'gst_process') and self.gst_process:
                    self.gst_process.terminate()
                    try:
                        self.gst_process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self.gst_process.kill()
                        self.gst_process.wait()
            except Exception as e:
                self.get_logger().warning(f"Error cleaning up GStreamer: {e}")

            self.get_logger().warning("Restarting GStreamer camera pipeline...")
            self._start_gstreamer_publisher(self.topic_name)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.color_frame = frame
            self.frame_count += 1

            if YOLO_MODE == "main":
                # Wrap YOLO separately so a slow/large model can never stall the
                # core frame pipeline (which is what makes the camera appear stale).
                try:
                    results = self.model(self.color_frame, verbose=False)
                    self.detection_frame = results[0].plot()
                except Exception as e:
                    self.get_logger().error(f"YOLO detection error: {e}")

        except Exception as e:
            self.get_logger().error(f"Failed to produce Go2 Camera detection image: {e}")

    def destroy_node(self):
        """Cleanup GStreamer process on shutdown"""
        self._stop = True
        if hasattr(self, 'gst_process') and self.gst_process:
            try:
                self.gst_process.terminate()
                self.gst_process.wait(timeout=2)
            except Exception:
                try:
                    self.gst_process.kill()
                    self.gst_process.wait()
                except Exception:
                    pass
        super().destroy_node()




