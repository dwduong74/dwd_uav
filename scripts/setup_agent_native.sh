#!/usr/bin/env bash
set -eo pipefail
delivery_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [ ! -d "$delivery_root/build/Micro-XRCE-DDS-Agent/.git" ]; then
    git clone --depth 1 --branch v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
        "$delivery_root/build/Micro-XRCE-DDS-Agent"
fi
source /opt/ros/humble/setup.bash
cmake -S "$delivery_root/build/Micro-XRCE-DDS-Agent" -B /tmp/dwd_uav_agent_242_build \
    -DUAGENT_SUPERBUILD=OFF -DUAGENT_P2P_PROFILE=OFF \
    -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_LOGGER=ON \
    -Dfastcdr_DIR=/opt/ros/humble/lib/cmake/fastcdr \
    -Dfastrtps_DIR=/opt/ros/humble/share/fastrtps/cmake \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$delivery_root/install/agent_242"
cmake --build /tmp/dwd_uav_agent_242_build --parallel 2
cmake --install /tmp/dwd_uav_agent_242_build
