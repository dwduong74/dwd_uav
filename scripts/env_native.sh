#!/usr/bin/env bash
# Source this file to use only Humble and this project's native build.
delivery_repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:/usr/local/sbin
export PYTHONNOUSERSITE=1
export ROS_LOG_DIR="$delivery_repo/log/ros"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
export PATH="$delivery_repo/install/agent_242/bin:$PATH"
export LD_LIBRARY_PATH="$delivery_repo/install/agent_242/lib:${LD_LIBRARY_PATH:-}"
if [ -f "$delivery_repo/install/px4_117/local_setup.bash" ]; then
    source "$delivery_repo/install/px4_117/local_setup.bash"
fi
if [ -f "$delivery_repo/install/native/local_setup.bash" ]; then
    source "$delivery_repo/install/native/local_setup.bash"
elif [ -f "$delivery_repo/install/local_setup.bash" ]; then
    source "$delivery_repo/install/local_setup.bash"
fi
# Prefer only Ubuntu's NumPy/OpenCV; putting all dist-packages first loads ROS 1 messages.
mkdir -p "$delivery_repo/build/python_system"
ln -sfn /usr/lib/python3/dist-packages/numpy "$delivery_repo/build/python_system/numpy"
for delivery_cv2 in /usr/lib/python3/dist-packages/cv2*.so; do
    ln -sfn "$delivery_cv2" "$delivery_repo/build/python_system/$(basename "$delivery_cv2")"
done
export PYTHONPATH="$delivery_repo/build/python_system:${PYTHONPATH:-}"
