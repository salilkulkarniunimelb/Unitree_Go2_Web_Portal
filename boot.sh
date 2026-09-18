#!/bin/bash
# boot.sh - main command for the robot_hivemind container.
# Auto-starts the dashboard portal whenever the container starts (server boot /
# docker restart), then keeps the container alive. Do not edit.
bash /workspace/lab_start.sh start || true
exec sleep infinity