#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/crawler/crawler_ws/install/setup.bash
source /home/crawler/crawler_ws/.venv/bin/activate
export PYTHONPATH=/home/crawler/crawler_ws/src/crawler_control:$PYTHONPATH
python3 -m crawler_control.nodes.imu_node
