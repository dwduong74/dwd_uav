#!/usr/bin/env bash
set -eo pipefail
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
exec ros2 launch delivery_ros delivery.launch.py config:="${1:?Supply calibrated site configuration}"
