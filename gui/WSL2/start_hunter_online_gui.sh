#!/usr/bin/env bash
set -e
source /opt/ros/melodic/setup.bash
if [ -f "$HOME/hunter_ws/devel/setup.bash" ]; then
  source "$HOME/hunter_ws/devel/setup.bash"
fi
exec python3 "$(cd "$(dirname "$0")" && pwd)/hunter_online_gui.py"
