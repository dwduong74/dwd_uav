#!/usr/bin/env bash
set -eo pipefail
if [ "$#" -ne 2 ]; then echo 'Usage: bash scripts/run_sitl.sh /path/PX4 /path/sitl_assets'; exit 2; fi
px4_dir=$(realpath "$1")
asset_dir=$(realpath "$2")
source "$(dirname "${BASH_SOURCE[0]}")/env_native.sh"
exec /usr/bin/python3 "$delivery_repo/scripts/run_gazebo_native.py" \
    --scene-only --px4-dir "$px4_dir" --assets "$asset_dir"
