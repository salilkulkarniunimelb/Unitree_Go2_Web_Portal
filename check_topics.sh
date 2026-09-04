#!/bin/bash
# check_topics.sh - Check which map/camera/pose topics are publishing live
# data on the QOD lab Go2 (runs inside the robot_hivemind container).
#
# Usage:
#   bash check_topics.sh            # all topics, ~2s each
#   TOPIC=/luna/frontvideostream bash check_topics.sh hz   # rate for one topic
# ---------------------------------------------------------------------------
SERVER="${SERVER:-salil.kulkarni@10.4.48.11}"
CONTAINER="${CONTAINER:-robot_hivemind}"
W="docker exec $CONTAINER bash -lc"
CMD="$W 'source /opt/ros/humble/setup.bash; [ -f /workspace/hardware_code/ros2_ws/install/setup.bash ] && source /workspace/hardware_code/ros2_ws/install/setup.bash; \$1' _"

ssh -o BatchMode=yes "$SERVER" "
  $W '
    source /opt/ros/humble/setup.bash
    [ -f /workspace/hardware_code/ros2_ws/install/setup.bash ] && source /workspace/hardware_code/ros2_ws/install/setup.bash
    echo \"HOST: \$(hostname)  ROS_DOMAIN_ID=\$ROS_DOMAIN_ID\"
    echo
    echo \"=== 1. TOPIC LIST (map/camera/pose/odom) ===\"
    ros2 topic list | grep -iE \"lidar_slam|frontvideo|current_pose|robot_odom|lowstate|completed_map\"
    echo
    for T in /luna/lidar_slam_2d_map /astro/lidar_slam_2d_map \
             /luna/frontvideostream /astro/frontvideostream \
             /luna/go2/restamped/robot_odom /astro/go2/restamped/robot_odom \
             /go2/stabilized/current_pose /lf/lowstate; do
        echo \"--- hz \$T ---\"
        timeout 4 ros2 topic hz \$T 2>&1 | head -1
    done
  '
"
