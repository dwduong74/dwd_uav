#!/usr/bin/env bash
set -eo pipefail
exec ros2 bag record -o "${1:-delivery-$(date +%Y%m%d-%H%M%S)}" \
  /delivery/mission_status /delivery/flight_state /delivery/flight_intent \
  /delivery/payload_state /delivery/landing_target /delivery/markers \
  /delivery/vision_latency /diagnostics /tf /tf_static \
  /fmu/out/vehicle_command_ack /fmu/out/vehicle_odometry /fmu/out/vehicle_status \
  /fmu/out/vehicle_land_detected /fmu/out/vehicle_local_position \
  /camera/image_raw /camera/camera_info
