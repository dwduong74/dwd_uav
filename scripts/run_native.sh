#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env_native.sh"
cd "$delivery_repo"
case "${1:-demo}" in
    deps)
        if [ ! -d src/px4_msgs/.git ]; then
            git init src/px4_msgs
            git -C src/px4_msgs remote add origin https://github.com/PX4/px4_msgs.git
            git -C src/px4_msgs fetch --depth 1 origin 86d8239e962f6939e05c3737784f60c02fa884db
            git -C src/px4_msgs checkout --detach FETCH_HEAD
        fi
        if [ "$(git -C src/px4_msgs rev-parse HEAD)" != 86d8239e962f6939e05c3737784f60c02fa884db ]; then
            echo 'px4_msgs must match dependencies.repos; import the pinned commit first.' >&2
            exit 1
        fi
        env -i PATH=/usr/bin:/bin LANG=C.UTF-8 /bin/bash --noprofile --norc "$delivery_repo/scripts/setup_agent_native.sh"
        exec env -i PATH=/usr/bin:/bin LANG=C.UTF-8 PYTHONNOUSERSITE=1 \
            /bin/bash --noprofile --norc -c '
                source /opt/ros/humble/setup.bash
                export PYTHONPATH="$1/build/python_system:${PYTHONPATH:-}"
                exec colcon --log-base "$1/log/px4_117" build --base-paths "$1/src/px4_msgs" \
                    --build-base /tmp/dwd_uav_px4_117_build --install-base "$1/install/px4_117" \
                    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
            ' bash "$delivery_repo"
        ;;
    build)
        if [ ! -f install/px4_117/local_setup.bash ]; then
            echo 'Run bash scripts/run_native.sh deps first to build PX4 1.17 messages.' >&2
            exit 1
        fi
        exec env -i PATH=/usr/bin:/bin LANG=C.UTF-8 PYTHONNOUSERSITE=1 \
            /bin/bash --noprofile --norc -c '
                source /opt/ros/humble/setup.bash
                source "$1/install/px4_117/local_setup.bash"
                export PYTHONPATH="$1/build/python_system:${PYTHONPATH:-}"
                cd "$1"
                exec colcon --log-base log/native build --base-paths src/delivery_ros src/delivery_interfaces \
                    --build-base build/native --install-base install/native --executor sequential
            ' bash "$delivery_repo"
        ;;
    test)
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
        export ROS_LOCALHOST_ONLY=1
        exec /usr/bin/python3 -m unittest discover -s tests -v
        ;;
    demo)
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
        export ROS_LOCALHOST_ONLY=0
        exec ros2 launch delivery_ros delivery.launch.py config:="$delivery_repo/config/native_demo.yaml"
        ;;
    sim)
        exec /usr/bin/python3 "$delivery_repo/scripts/run_gazebo_native.py"
        ;;
    table-c)
        exec /usr/bin/python3 "$delivery_repo/scripts/run_gazebo_native.py" --table-c \
            --course "${2:-$delivery_repo/config/table_c.yaml}"
        ;;
    hardware)
        exec ros2 launch delivery_ros delivery.launch.py config:="${2:?Supply calibrated hardware configuration}"
        ;;
    *) echo 'Usage: bash scripts/run_native.sh {deps|build|test|demo|sim|table-c|hardware /path/config.yaml}' >&2; exit 2 ;;
esac
