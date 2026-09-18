#!/bin/bash
# ---------------------------------------------------------------------------
# start_dashboard.sh - start/stop the Go2 dashboard from the server, by ANY
# user (anyone in the docker group, which covers all the lab users).
#
# The app + deps live inside the always-running "robot_hivemind" container,
# so this only controls the portal process. Run it like this (from anywhere):
#
#   docker exec robot_hivemind /workspace/start_dashboard.sh
#   docker exec robot_hivemind /workspace/start_dashboard.sh start
#   docker exec robot_hivemind /workspace/start_dashboard.sh restart
#   docker exec robot_hivemind /workspace/start_dashboard.sh stop
#   docker exec robot_hivemind /workspace/start_dashboard.sh status
#
# Then open http://10.4.48.11:7860 in any browser on the network.
# ---------------------------------------------------------------------------
cd /workspace || exit 1

if ! python3 -c "import gradio" >/dev/null 2>&1; then
    echo "ERROR: dashboard deps not installed in the container."
    echo "Have the dashboard owner run the one-time deploy."
    exit 1
fi

bash lab_start.sh "${1:-start}"

case "${1:-start}" in
    start|restart)
        echo ""
        echo "Dashboard is up. Open it in any browser:  http://10.4.48.11:7860"
        ;;
esac