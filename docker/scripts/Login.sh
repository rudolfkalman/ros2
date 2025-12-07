#!/usr/bin/env bash
set -euo pipefail
sudo docker exec --user ubuntu -it ros2_jazzy_gui bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ubuntu/projects/slam; [ -f install/setup.bash ] && source install/setup.bash; exec bash'