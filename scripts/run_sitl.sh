#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -ne 2 ]; then echo 'Usage: bash scripts/run_sitl.sh /path/PX4 /path/sitl_assets'; exit 2; fi
px4_dir=$(realpath "$1")
asset_dir=$(realpath "$2")
export GZ_SIM_RESOURCE_PATH="$asset_dir/models:$asset_dir/worlds:$px4_dir/Tools/simulation/gz/models:$px4_dir/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export PX4_SYS_AUTOSTART=4001
export PX4_SIM_MODEL=gz_x500_delivery
export PX4_GZ_WORLD=delivery
cd "$px4_dir"
exec build/px4_sitl_default/bin/px4
