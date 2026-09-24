from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math
from rclpy.node import Node
from config import POSE_HEADER_FRAME_ID


class Nav2Controller(Node):
    def __init__(self):
        super().__init__('web_nav_client')
        self.single_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.multi_client  = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.get_logger().info("Nav2Controller initialized")

    def _pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = POSE_HEADER_FRAME_ID
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2)
        p.pose.orientation.w = math.cos(yaw / 2)
        return p

    def _server_ready(self, client, action_name):
        """Wait up to a few seconds for the action server; returns bool."""
        if client.server_is_ready():
            return True
        if not client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                f"{action_name} action server not available. Is Nav2 running? "
                f"Check `ros2 action list` for '{action_name}'."
            )
            return False
        return True

    def go_to(self, wp):
        if not self._server_ready(self.single_client, 'navigate_to_pose'):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(wp["x"], wp["y"], wp["yaw"])
        self.single_client.send_goal_async(goal)
        self.get_logger().info(f"Sent navigate_to_pose goal to ({wp['x']:.2f}, {wp['y']:.2f}, yaw={wp['yaw']:.2f})")
        return True

    def run_route(self, waypoints):
        if not self._server_ready(self.multi_client, 'navigate_through_poses'):
            return False
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._pose(w["x"], w["y"], w["yaw"]) for w in waypoints]
        self.multi_client.send_goal_async(goal)
        self.get_logger().info(f"Sent navigate_through_poses goal with {len(waypoints)} poses")
        return True


