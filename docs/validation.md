# Validation record — updated 2026-09-17

## Table C implementation — 2026-09-17

- Native build passes with competition, safety, VIO and bidirectional payload APIs.
- **44 tests pass**, including three-package ordering, route validation, marker
  confirmation and idempotent GRAB/RELEASE behavior.
- The Table C Gazebo world starts with PX4 v1.17 and all ROS nodes. A run has
  reached package scan, simulated grab and first-delivery TRANSIT.
- A complete repeated three-package SITL run and hardware VIO/gripper operation
  remain unvalidated and must not be presented as acceptance evidence yet.

## PX4 v1.17.0 migration — 2026-09-16

- Native ROS 2 Humble build passed with pinned `px4_msgs` release/1.17 commit
  `86d8239e962f6939e05c3737784f60c02fa884db`. All 12 message definitions used by
  the adapter match the installed PX4 source.
- Final full suite: **38/38 passed in 29.405 s**, including versioned topic names,
  initial EKF yaw alignment gating, DDS acquisition-time mapping/frozen-frame behavior,
  delayed-TF frame retention, specific preflight failure details and ACK
  correlation with a PX4 clock 250 ms behind wall time (stale ACKs still rejected).
- Gazebo GUI opened with `x500_delivery_0`; real PX4 DDS telemetry reported
  healthy flight state, valid range, healthy simulated payload and camera/TF readiness.
- Agent v2.4.2 is installed locally with Humble Fast DDS 2.6. The system Agent
  v3.0.1 and the outer workspace's v1.15 messages are not used by the native launcher.
- After mapping acquisition stamps through DDS timesync and reducing the camera
  to 320x240, a 15-second post-startup sample received 150 frames and
  150/150 vision-ready messages. Countdown failures now report each failed gate.
- A disarmed Offboard-only probe against real PX4 returned command_state=2,
  offboard=true, armed=false and "ACK and state confirmed" after the ACK clock fix.
- No delivery goal/arming command was sent to PX4 during this integration check.
  This does not establish a successful physical or simulated round-trip flight.

## Native flight check — 2026-09-16

- With `DELIVERY_HEADLESS=1`, real PX4 SITL accepted Offboard and arm, climbed
  to 1.53 m, completed TAKEOFF and entered TRANSIT and then SEARCH.
- PX4 v1.17's single expected post-arm heading alignment reset is accepted only
  during first TAKEOFF. Reference, XY, Z, later heading and multi-counter resets
  still terminate the mission.
- Full round-trip delivery and GUI-mode flight remain pending acceptance checks.

## Precision landing at B — 2026-09-17

- The downward camera was moved below the landing gear, pad textures use unique
  resource names, and the PX4 LAND handoff occurs at 0.8 m AGL.
- Real PX4 SITL completed TRANSIT, SEARCH, APPROACH, DESCEND and LAND at marker 0.
  Gazebo ground truth at touchdown was `(2.9922, -0.0193)` m for target `(3, 0)`:
  **0.021 m horizontal error**. The simulated payload was released.
- This trial later ended with `TELEMETRY_LOST` before the return leg, so it proves
  the requested A-to-B precision landing but not a complete B-to-A round trip.

## Original container validation — 2026-09-12


- Ubuntu 22.04 / ROS 2 Humble inside Docker on WSL: `colcon build` succeeded
  for `delivery_interfaces` and `delivery_ros`.
- Original validation used `px4_msgs` release/1.15 commit
  `a1045ec4feb6d709bdecaf3895f1d5b43a5dabb8`; the current manifest targets 1.17.
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

1. PX4 v1.17.0 + Gazebo Harmonic: 20 actual SITL round-trip trials
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

The current suite includes Table C planner/supervisor, asset and sensor-clock tests (44 tests total).
See `docs/sitl.md` for the remaining Gazebo checks and `scripts/benchmark_pi.py`
for the hardware performance record. CI runs the same ROS build/test script.
