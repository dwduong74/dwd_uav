#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /deps/install/setup.bash
colcon --log-base log/container build --build-base build/container --install-base install/container --executor sequential
source install/container/setup.bash
python3 -m unittest discover -s tests -v
