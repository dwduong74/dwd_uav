# Validation record — 2026-09-12

## Executed

- Ubuntu 22.04 / ROS 2 Humble inside Docker on WSL: `colcon build` succeeded
  for `delivery_interfaces` and `delivery_ros`.
- `px4_msgs` compiled at release/1.15 commit
  `a1045ec4feb6d709bdecaf3895f1d5b43a5dabb8`; dependency manifest pins that commit.
- Final full run: 33 tests passed together in 29.008 s, including real ROS action transport,
  cancellation/concurrent-goal rejection, transient-local late subscription,
  matching topic/feedback/result snapshots, round trip through the payload
  action, OpenCV image-to-TF pose and adapter ACK/state/watchdog behavior.
- Asset test passed in the same Humble run: generated SDF parses,
  two real marker textures decode to IDs 0 and 1, sim payload profile is selected.
- Pure engine test completes 20 deterministic round trips and checks cancellation,
  pending arm, payload cancellation, stale/duplicate markers, estimator reset,
  telemetry loss, low battery, range loss and no release before landed/disarmed.
- Python sources compile on the Windows development host. ROS tests deliberately
  skip only when rclpy is unavailable; missing interfaces/dependencies in ROS fail.

## What these results do not establish

The flight double is kinematic: it acknowledges commands and follows setpoints.
It does **not** model PX4 dynamics, estimator behavior, motor response, GPS noise
or landing physics. Image tests use synthetic pixels and a known transform.

Not yet run:

1. Generated world/model in PX4 v1.15.4 + Gazebo Harmonic; 20 actual SITL trials
   and measured <=0.3 m touchdown error against simulator ground truth.
2. Arduino Nano firmware compilation/upload and electrical/physical switch,
   servo, watchdog, power-loss and jam tests on the selected mechanism.
3. Raspberry Pi 4 30-minute CPU/thermal/latency/heartbeat benchmark.
4. Camera/rangefinder calibration and staged flight tests on the actual airframe.

These are pending acceptance gates, not passed tests. The test image contains
ROS/OpenCV/px4_msgs; it does not install PX4 firmware or Gazebo. No flight
controller, servo or hardware output was connected or operated in this session.

## Reproduce

```bash
docker build -t delivery-test -f Dockerfile.test .
docker run --rm -v "$PWD:/workspace" delivery-test bash scripts/test_container.sh
```

The full current suite includes the asset test (33 tests total).
See `docs/sitl.md` for the remaining Gazebo checks and `scripts/benchmark_pi.py`
for the hardware performance record. CI runs the same ROS build/test script.
