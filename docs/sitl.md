# PX4 1.15.4 + Gazebo Harmonic

Prerequisites: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15.4 with submodules and its
Gazebo build dependencies, Micro XRCE-DDS Agent v2.4.2, a working GPU/software
renderer. Install the Humble bridge built for **Harmonic**, not the default
Fortress bridge; follow the ROS/Gazebo compatibility instructions for that pairing.
The unit-test Docker image does not contain PX4 or Gazebo.

From this repo:

```bash
python3 scripts/enable_land_topic.py /path/PX4-Autopilot
python3 scripts/prepare_sitl.py /path/PX4-Autopilot
# In PX4 checkout: make px4_sitl
```

This creates `build/sitl_assets/models`, `worlds/delivery.sdf` and `sitl.yaml`.
The world preserves the checked-out PX4 default world's plugins and spherical
coordinates. The delivery pad is 3 m east of launch, ID 0; launch pad ID 1.
Markers are 0.4 m black squares inside 0.6 m white pads.

Three terminals, with ROS/workspace sourced where needed:

```bash
bash scripts/run_sitl.sh /path/PX4-Autopilot "$PWD/build/sitl_assets"
MicroXRCEAgent udp4 -p 8888
ros2 launch delivery_ros sitl.launch.py config:="$PWD/build/sitl_assets/sitl.yaml"
```

Keep QGroundControl connected. Set and verify the PX4 Offboard-loss behavior,
RC policy and auto-disarm-on-land before starting. Do not remove arming checks
to hide an incomplete simulation configuration. The companion does not force disarm.

The camera bridge uses wall-time image stamps to match ROS/PX4 time sync;
all nodes use wall time. Gazebo must run close to real time. Pause/slowdown will
exercise watchdogs rather than pause mission timeouts. Check `gz_frame_id` is
`camera_optical_frame` on both Image and CameraInfo with the installed bridge.

The generated sim profile uses a one-beam ROS LaserScan as its range source.
Hardware defaults to PX4's fused range-backed `dist_bottom`. Do not use the sim
profile on hardware. Static camera TF is valid only for the generated model.

Before starting:

```bash
ros2 topic echo /delivery/flight_state --once
ros2 topic echo /delivery/vision_ready --once
ros2 topic echo /delivery/payload_state --once
ros2 service call /delivery/start std_srvs/srv/Trigger '{}'
```

Require healthy flight/payload, camera ready and range valid. Record rosbag and
ULog. Measure actual touch-down position against pad center from Gazebo ground
truth; do not use the commanded target as measured landing error.

For 20 trials, restart the sim payload node between trials to reload its simulated
package, and return vehicle to the launch pad. Run one goal at a time. Include
missing/wrong markers, image freeze, Agent loss, command rejection, cancel in
flight and payload failure. A generated world alone is not a SITL pass; record
the actual results in `docs/validation.md`.
