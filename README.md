# Holybro X650 PX4 + ROS 2 Gazebo simulation

This repository contains a self-contained Gazebo model of the Holybro X650,
a PX4 SITL launcher, and a small ROS 2 offboard-control example.

The model uses the X650's published 650 mm wheelbase. Its 3.2 kg simulated
mass represents the 2.0 kg frame/electronics kit plus a 6S 10 Ah battery. The
aerodynamic constants are an initial engineering approximation for 15-inch
propellers; tune them against logs from your exact motor/propeller/battery
combination before treating the simulation as a digital twin.

## Clone the repository

The ROS 2 message definitions and Micro XRCE-DDS Agent are pinned as Git
submodules. Clone them together with the main repository:

```bash
git clone --recurse-submodules \
  git@github.com:MYahsap/Bird-Safe-Window-Marker-Instalation-Drone.git
cd Bird-Safe-Window-Marker-Instalation-Drone
```

If the repository was cloned without `--recurse-submodules`, initialize the
dependencies afterward:

```bash
git submodule update --init --recursive
```

## Supported stack

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (`gz sim`)
- PX4 v1.17.0
- Micro XRCE-DDS Agent on UDP port 8888

Other recent ROS 2/Gazebo releases may work, but the versions above are the
tested target. PX4 and `px4_msgs` must use matching release branches.

## Recommended setup on CachyOS, Arch, or other non-Ubuntu hosts

ROS 2 Jazzy binary packages target Ubuntu 24.04, and PX4 v1.17's supported
native toolchain uses the same platform. Use the official PX4 Jazzy/Gazebo
container instead of mixing rolling-distribution Python packages with the
PX4 build.

Install either Podman or Docker. On CachyOS/Arch, for example:

```fish
sudo pacman -S podman fuse-overlayfs
```

`fuse-overlayfs` is recommended for fast rootless image storage on Btrfs. If
it is absent, the launcher automatically uses a slower project-local VFS
store instead of failing with an OverlayFS error. Container data is stored in
`.container-storage/` and can be removed when you no longer need the image.
The image reference is fully qualified as `docker.io/px4io/...`, so Podman
does not require a system-wide unqualified-search registry configuration.

Then build and run the project from Fish or any other shell:

```fish
./scripts/container.sh build
./scripts/container.sh check
./scripts/container.sh run
```

The `build` command must finish successfully before `run`; otherwise ROS 2
will only see its `/opt/ros/jazzy` underlay and report `x650_sim` as missing.
Container artifacts use `build-container/`, `install-container/`, and
`log-container/`, keeping CMake's absolute container paths separate from any
native build attempted on the host.

Use `./scripts/container.sh headless` when no GUI is needed, or
`./scripts/container.sh shell` for an interactive Bash shell in the prepared
environment. The project and PX4 source/build directories are bind-mounted,
so builds persist when the container exits.

In GUI mode, Gazebo, PX4, and ROS 2 run together in the supported Ubuntu
container, eliminating cross-container Gazebo discovery problems. Because
OGRE2 currently fails through native Wayland on this host, the launcher uses
GNOME's XWayland compatibility display. It mounts the X11 socket and the
session authorization cookie into the container, and exposes the Intel
`/dev/dri/renderD128` node. X11 clients can potentially observe desktop input
or other windows while running, so use this only with the trusted PX4 image and
stop it with `Ctrl+C` when finished.

## Native Ubuntu 24.04 setup

### 1. Install prerequisites

Install ROS 2 Jazzy and the PX4 Ubuntu toolchain first. Then clone and build
the dependencies (the script does not use `sudo` or modify system packages):

```bash
./scripts/bootstrap.sh
```

By default dependencies are placed next to this repository:

```text
../PX4-Autopilot
ros2_ws/src/px4_msgs
ros2_ws/src/Micro-XRCE-DDS-Agent
```

Override the PX4 path with `PX4_AUTOPILOT_PATH`.

The bootstrap script now checks the operating system and PX4 Python modules
before starting a build. If PX4 dependencies are missing, run the toolchain
installer path printed by the script.

### 2. Build the ROS 2 workspace

```bash
./scripts/build_ros2.sh
source ros2_ws/install/setup.bash
```

When using Fish, source the Fish environment file—not the Bash file:

```fish
source ros2_ws/install/setup.fish
```

### 3. Run

Launch Gazebo, PX4 SITL, and the DDS agent:

```bash
export PX4_AUTOPILOT_PATH="$PWD/../PX4-Autopilot"
ros2 launch x650_sim sim.launch.py
```

### QGroundControl

On the CachyOS host, install the pinned QGroundControl v5.1.4 AppImage into
the project and launch it with:

```fish
./scripts/qgroundcontrol.sh install
./scripts/qgroundcontrol.sh run
```

The launcher automatically uses AppImage extraction mode if the host has
FUSE 3 but not `libfuse.so.2`. Because the simulation container uses host
networking, QGroundControl should discover PX4 on UDP port 14550 without a
manual communication link. Start the simulation first, then QGroundControl.

In a second terminal, check the bridge:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /fmu/out/vehicle_status_v1
```

Start the demonstration flight only after Gazebo is running and the estimator
is ready:

```bash
ros2 run x650_sim offboard_hover --ros-args -p altitude:=3.0
```

The node streams a 3 m NED position setpoint, switches to Offboard after one
second, and arms. Stop it with Ctrl-C; PX4's offboard-loss failsafe then takes
over. You can also start it with the launcher by setting
`start_offboard:=true`, but the separate command is safer for debugging.

## One-DOF STS3215 manipulator

The model includes an STS3215-sized servo housing 15 cm forward and 10 cm below
`base_link`, plus a 50 cm, 100 g moving link with a simulated 0.5 kg payload at
its midpoint. The revolute joint turns about vehicle +Y, so it tilts
forward/backward in the X-Z plane. Its software range is -180 to +90 degrees,
and its initial target is -90 degrees.

The actuator is modeled as the 12 V ST-3215-C018 variant: 30 kg.cm
(2.942 N.m) peak/stall torque, 10 kg.cm (0.981 N.m) rated torque, and
0.222 s/60 degrees no-load speed. The Gazebo controller uses the peak torque
as its hard limit and includes integral action to remove steady-state sag.
Thermal limits are not simulated; the 0.5 kg midpoint load requires about
1.47 N.m while horizontal, which exceeds the physical servo's continuous
rated torque even though it is below stall torque.

The world also contains a 40 m wide, 20 m tall wall centered 20 m in front of
the takeoff point. It is 0.25 m thick, spans Y=-20..20 m, and rests on Z=0.

## Fixed RGB camera

The drone has a fixed forward-facing RGB camera attached to `base_link` at
`x=0.14 m`, `z=0.02 m`. It renders 640x480 RGB images at 30 FPS with a
90-degree horizontal field of view. The camera bridge starts automatically and
publishes standard ROS 2 messages on:

```text
/x650/camera/image_raw       sensor_msgs/msg/Image
/x650/camera/camera_info     sensor_msgs/msg/CameraInfo
```

Check the stream from a second container shell:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install-container/setup.bash
ros2 topic hz /x650/camera/image_raw
ros2 topic echo --once /x650/camera/camera_info
```

Record camera data with:

```bash
ros2 bag record /x650/camera/image_raw /x650/camera/camera_info
```

The simulation controller starts automatically and implements Cartesian
impedance in the arm's X-Z plane. It computes
`F = K (p_target - p) - D v`, maps that force to the revolute joint with
`tau = J(q)^T F`, adds gravity compensation, and limits output to the
ST-3215-C018 peak torque of 2.942 N.m. Default gains are 40 N/m stiffness and
5 N.s/m damping on both X and Z. Coordinates are relative to `base_link`.

Forward stabilization is enabled by default. The controller subscribes to
PX4 attitude on `/fmu/out/vehicle_attitude`, extracts vehicle pitch, and uses:

```text
joint_target = -pi/2 + vehicle_pitch
```

This rotates the arm opposite the vehicle's pitch so it remains horizontal
and forward relative to the world. The arm has only one DOF, so it cannot
compensate vehicle roll or independently control world yaw. Also, forward is
the -90 degree forward reference is centered inside the compensation range:
vehicle pitch from -90 to +90 degrees can be cancelled with joint targets from
-180 to 0 degrees. Commands outside the configured range are still clamped and
produce a warning.

The extended negative range can sweep the physical arm toward the airframe and
propeller plane. Gazebo has intra-model self-collision disabled; verify real
mechanical clearance before enabling this range on hardware.

Inspect the compensated command with:

```bash
ros2 topic echo /x650/manipulator/stabilized_angle
```

Disable stabilization when manually commanding Cartesian or angle targets:

```bash
ros2 launch x650_sim sim.launch.py manipulator_stabilize_forward:=false
```

With stabilization disabled, command a Cartesian tip target from a second
container shell:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install-container/setup.bash

ros2 topic pub --once /x650/manipulator/cartesian_target \
  geometry_msgs/msg/Point '{x: 0.50, y: 0.0, z: -0.35}'
```

Because the arm has one DOF, it can only generate force tangent to its circular
workspace; the Jacobian projects an arbitrary X-Z target onto that available
direction. The legacy angle topic remains available and is converted to a
Cartesian point on the 0.5 m workspace circle:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install-container/setup.bash

ros2 topic pub --once /x650/manipulator/angle std_msgs/msg/Float64 '{data: 0.785398}'
ros2 topic pub --once /x650/manipulator/angle std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /x650/manipulator/angle std_msgs/msg/Float64 '{data: -0.785398}'
```

Those commands request +45, 0, and -45 degrees. Change the joint's `<axis>` in
`models/x650/model.sdf` if the intended physical tilt axis is different.

Controller feedback is available for plotting and rosbag logging:

```text
/x650/manipulator/joint_state        sensor_msgs/msg/JointState
/x650/manipulator/tip_position       geometry_msgs/msg/PointStamped
/x650/manipulator/commanded_torque   std_msgs/msg/Float64
/x650/manipulator/stabilized_angle   std_msgs/msg/Float64
```

Tune compliance through launch arguments, for example:

```bash
ros2 launch x650_sim sim.launch.py \
  manipulator_stiffness_x:=25.0 manipulator_stiffness_z:=25.0 \
  manipulator_damping_x:=4.0 manipulator_damping_z:=4.0
```

Keep stiffness low enough that normal Cartesian errors do not continuously
saturate the 2.942 N.m torque limit. State loss commands zero torque after
2 seconds. The timeout tolerates Gazebo running well below real time while
still removing actuator effort when feedback is lost.

For the physical servo, first identify the adapter using a stable path under
`/dev/serial/by-id/`. Pass it into a container:

```bash
X650_SERIAL_DEVICE=/dev/serial/by-id/YOUR_ADAPTER ./scripts/container.sh shell
```

Inside that shell, start the hardware driver (factory defaults shown):

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install-container/setup.bash
ros2 run x650_sim manipulator_controller --ros-args \
  -p mode:=hardware \
  -p port:=/dev/ttyUSB0 \
  -p baud:=1000000 \
  -p servo_id:=1 \
  -p zero_position:=2048 \
  -p direction:=1
```

Use the resolved device printed by `readlink -f /dev/serial/by-id/YOUR_ADAPTER`
as the `port` value if it is not `/dev/ttyUSB0`. Publish angles from another
ROS container as above. Set `direction:=-1` if positive motion is reversed.
Hardware mode remains position-only: true impedance requires validated
bidirectional position/velocity feedback and torque or current control, which
the current minimal STS3215 serial protocol intentionally does not enable.
Forward pitch stabilization does operate in hardware mode by sending the
compensated position target at up to 20 Hz, provided the PX4 attitude topic is
available through the ROS 2 bridge.

The adapter must support the STS3215 single-wire half-duplex TTL bus. For the
ST-3215-C018, use a suitable 12 V supply capable of handling its 2.7 A stall
current, and connect the adapter and supply grounds together. Do not power the
servo motor from the USB data adapter.

Useful launch options:

```bash
ros2 launch x650_sim sim.launch.py headless:=true
ros2 launch x650_sim sim.launch.py start_agent:=false
ros2 launch x650_sim sim.launch.py spawn_pose:="0,0,0.25,0,0,0"
```

## Layout

```text
models/x650/                  Gazebo SDF model
worlds/x650_world.sdf         Flat test world
ros2_ws/src/x650_sim/         ROS 2 package and launcher
scripts/bootstrap.sh          Dependency/build helper
scripts/build_ros2.sh         Guarded ROS 2 workspace builder
scripts/container.sh          CachyOS/Arch container entry point
scripts/validate.sh           Offline structural checks
```

## Notes

- The ROS/PX4 position setpoint is NED: negative Z means upward.
- PX4's SITL client connects to the XRCE agent at `127.0.0.1:8888`.
- The launcher starts the custom world by absolute path and runs PX4 with
  `PX4_GZ_STANDALONE=1`; PX4 v1.17 otherwise searches only its in-tree world
  directory when it starts Gazebo itself.
- DDS is unauthenticated. Keep port 8888 and ROS discovery local to the
  development machine.
- To tune flight behavior, start with `MC_MOT_THST_EXPO`, `MPC_THR_HOVER`, and
  the multicopter rate gains, then adjust the SDF motor constants only when
  matching measured propulsion data.

## PX4 controller baseline

The launcher explicitly restores PX4 v1.17's multicopter attitude, rate, and
position gains plus the proven gz_x500 control-allocation geometry. Explicit
startup values are necessary because experimental values may already have
been saved in `parameters.bson`.

Do not copy Gazebo SDF rotor Y positions directly into `CA_ROTOR*_PY`: Gazebo
model poses use FLU body coordinates while PX4's allocator uses FRD, so Y must
change sign. An earlier experimental preset missed that conversion and could
turn roll stabilization into positive feedback during takeoff. This baseline
prioritizes a stable takeoff; further load compensation must be based on ULog
setpoint-versus-response data.

To inspect the values after PX4 starts:

```bash
px4-param show MC_PITCHRATE_I
px4-param show MC_PITCHRATE_D
px4-param show MPC_XY_P
px4-param show MPC_Z_P
px4-param show CA_ROTOR0_PX
```

Run one axis at a time and compare setpoint versus response in the PX4 ULog
before changing gains. Do not copy SITL gains directly to the physical X650.

# Bird-Safe-Window-Marker-Instalation-Drone
