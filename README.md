# Holybro X650 PX4 + ROS 2 Gazebo simulation

Simulation and control environment for a Holybro X650 quadrotor equipped with
a one-degree-of-freedom manipulator and a forward-facing RGB camera.

The project combines:

- PX4 v1.17 software-in-the-loop (SITL)
- ROS 2 Jazzy and `px4_msgs`
- Gazebo Harmonic
- Micro XRCE-DDS Agent
- A 650 mm X650 model with an asymmetric payload
- A simulated ST-3215-C018 servo and Cartesian impedance controller
- A ROS 2 RGB camera bridge
- An optional STS3215 USB-to-TTL hardware interface

> [!IMPORTANT]
> This model is an engineering approximation, not a validated digital twin.
> Validate propulsion, inertia, controller gains, servo loading, and mechanical
> clearances before transferring anything to real hardware.

## Quick start

The container workflow is recommended on CachyOS, Arch, and other non-Ubuntu
hosts. It keeps ROS, Gazebo, and PX4 on the supported Ubuntu 24.04 toolchain.

### 1. Clone the project

```bash
git clone --recurse-submodules \
  git@github.com:MYahsap/Bird-Safe-Window-Marker-Instalation-Drone.git
cd Bird-Safe-Window-Marker-Instalation-Drone
```

For an existing clone that is missing the dependencies:

```bash
git submodule update --init --recursive
```

### 2. Install a container runtime

Install Podman or Docker. On CachyOS or Arch:

```bash
sudo pacman -S podman fuse-overlayfs
```

`fuse-overlayfs` is strongly recommended on Btrfs. Without it, the launcher
uses a slower project-local VFS store under `.container-storage/`.

### 3. Build and validate

```bash
./scripts/container.sh build
./scripts/container.sh check
```

The initial build downloads the PX4 development image, builds PX4 SITL, and
builds the ROS 2 workspace. Generated files are kept in ignored directories,
so subsequent builds are incremental.

### 4. Start the simulator

```bash
./scripts/container.sh run
```

This starts Gazebo with its GUI, PX4 SITL, the XRCE-DDS agent, the manipulator
controller, and the camera bridge. Stop the simulation with `Ctrl+C`.

For a simulation without the Gazebo GUI:

```bash
./scripts/container.sh headless
```

## Interacting with ROS 2

Keep the simulator running and open another terminal in the repository:

```bash
./scripts/container.sh shell
```

Inside the container shell:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install-container/setup.bash
ros2 topic list
```

Confirm that PX4 is connected:

```bash
ros2 topic echo --once /fmu/out/vehicle_status_v1
```

### Offboard hover example

Wait until Gazebo is running and PX4 has completed estimator initialization,
then command a 3 m hover:

```bash
ros2 run x650_sim offboard_hover --ros-args -p altitude:=3.0
```

The node continuously publishes a NED position setpoint, requests Offboard
mode, and requests arming until PX4 confirms both transitions. In NED
coordinates, a negative Z setpoint means upward motion. Stop the node with
`Ctrl+C`; PX4's offboard-loss failsafe then takes over.

The example is intentionally small. Use it as a starting point for a new ROS 2
node instead of treating it as a complete flight controller.

## Simulated vehicle

### X650 airframe and world

- 650 mm diagonal motor-to-motor wheelbase
- 3.191 kg base airframe mass before the moving manipulator assembly
- Four 15-inch-class simulated propellers
- PX4 `gz_x500` control-allocation baseline adapted to the X650 geometry
- Spawn pose: `x=0`, `y=0`, `z=0.25 m`
- A 40 m wide, 20 m tall wall centered 20 m in front of the vehicle
- A directional light directly above the world origin

The aerodynamic coefficients are starting estimates. Tune them only against
measured motor, propeller, battery, and flight-log data.

### One-DOF manipulator

The manipulator pivot is 15 cm forward and 10 cm below `base_link`. Its 50 cm
moving link rotates about vehicle +Y, producing motion in the vehicle X-Z
plane. The moving assembly is modeled as 0.60 kg, including the 0.50 kg
mid-link payload.

| Property | Value |
| --- | --- |
| Joint range | -180° to +90° |
| Initial/forward angle | -90° |
| Link length | 0.50 m |
| Servo model | 12 V ST-3215-C018 |
| Peak/stall torque | 2.942 N·m (30 kg·cm) |
| Rated torque | 0.981 N·m (10 kg·cm) |
| No-load speed | 0.222 s/60° |

The horizontal arm requires approximately 1.47 N·m for the midpoint load.
That is below stall torque but above the physical servo's continuous rated
torque. Gazebo does not simulate winding temperature or servo damage.

The extended negative joint range can also sweep the arm toward the airframe
or propellers. Intra-model collision is disabled in the simulation, so confirm
real mechanical clearance independently.

### Cartesian impedance control

The controller applies a spring-damper force in the arm's X-Z plane:

```text
F = K (p_target - p) - D v
tau = J(q)^T F + gravity compensation
```

Default stiffness is 40 N/m per axis, default damping is 5 N·s/m per axis,
and output is limited to 2.942 N·m. Because the arm has one DOF, the Jacobian
can apply only the component of Cartesian force tangent to its circular
workspace.

Forward stabilization is enabled by default. The controller uses PX4 pitch to
command:

```text
joint_target = -pi/2 + vehicle_pitch
```

This compensates pitch so the arm remains forward with respect to the world.
One joint cannot compensate vehicle roll or independently control world yaw.

Monitor the manipulator:

```bash
ros2 topic echo /x650/manipulator/joint_state
ros2 topic echo /x650/manipulator/tip_position
ros2 topic echo /x650/manipulator/commanded_torque
ros2 topic echo /x650/manipulator/stabilized_angle
```

`joint_state` is the measured joint angle relative to the vehicle. The
`stabilized_angle` topic is the pitch-compensated command. Comparing these with
`/fmu/out/vehicle_attitude` shows how the joint behaves relative to the world.

To command the arm manually, restart the launch with stabilization disabled:

```bash
ros2 launch x650_sim sim.launch.py manipulator_stabilize_forward:=false
```

Command a Cartesian tip target relative to `base_link`:

```bash
ros2 topic pub --once /x650/manipulator/cartesian_target \
  geometry_msgs/msg/Point '{x: 0.50, y: 0.0, z: -0.35}'
```

Or command a joint angle in radians:

```bash
ros2 topic pub --once /x650/manipulator/angle \
  std_msgs/msg/Float64 '{data: -1.570796}'
```

Tune compliance at launch time:

```bash
ros2 launch x650_sim sim.launch.py \
  manipulator_stiffness_x:=25.0 \
  manipulator_stiffness_z:=25.0 \
  manipulator_damping_x:=4.0 \
  manipulator_damping_z:=4.0
```

Keep stiffness low enough to avoid continuously saturating the torque limit.
The controller removes actuator effort if Gazebo joint feedback is absent for
more than two seconds.

### RGB camera

The fixed camera is 14 cm forward and 2 cm above `base_link`. It produces
640×480 RGB images at 30 FPS with a 90° horizontal field of view.

| ROS 2 topic | Message |
| --- | --- |
| `/x650/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/x650/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

Check the stream:

```bash
ros2 topic hz /x650/camera/image_raw
ros2 topic echo --once /x650/camera/camera_info
```

## QGroundControl

Install the pinned QGroundControl AppImage once, then run it on the host:

```bash
./scripts/qgroundcontrol.sh install
./scripts/qgroundcontrol.sh run
```

Start the simulation first. Because the container uses host networking,
QGroundControl should discover PX4 automatically on UDP port 14550.

## Data logging

Record the main ROS 2 data streams from a container shell:

```bash
ros2 bag record \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_local_position \
  /x650/manipulator/joint_state \
  /x650/manipulator/tip_position \
  /x650/manipulator/commanded_torque \
  /x650/camera/image_raw \
  /x650/camera/camera_info
```

ROS bag directories and `.db3`/`.mcap` files are ignored by Git. Camera data
is large; omit the image topic when it is not needed.

PX4 also produces ULog flight logs under its SITL root filesystem. Use those
logs to compare setpoints and responses before changing PX4 gains.

## Physical STS3215 interface

The hardware path sends position commands through a single-wire half-duplex
TTL adapter. It does **not** provide true hardware impedance control because
the current implementation does not enable validated current/torque feedback.

Find the adapter's stable device path and pass it into the container:

```bash
ls -l /dev/serial/by-id/
X650_SERIAL_DEVICE=/dev/serial/by-id/YOUR_ADAPTER \
  ./scripts/container.sh shell
```

Inside the container:

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

Use the device returned by `readlink -f /dev/serial/by-id/YOUR_ADAPTER` if it
is not `/dev/ttyUSB0`. Set `direction:=-1` if positive motion is reversed.

> [!WARNING]
> Use a suitable 12 V supply capable of the servo's 2.7 A stall current. Join
> the adapter and supply grounds, and do not power the servo motor from the USB
> adapter.

## Launch options

When launching directly inside a prepared ROS 2 environment:

```bash
ros2 launch x650_sim sim.launch.py headless:=true
ros2 launch x650_sim sim.launch.py start_agent:=false
ros2 launch x650_sim sim.launch.py start_offboard:=true
ros2 launch x650_sim sim.launch.py start_manipulator:=false
ros2 launch x650_sim sim.launch.py start_camera_bridge:=false
ros2 launch x650_sim sim.launch.py spawn_pose:="0,0,0.25,0,0,0"
```

Run `ros2 launch x650_sim sim.launch.py --show-args` for the complete list.

## Native Ubuntu 24.04 setup

The container path is preferred. For a native installation, first install ROS
2 Jazzy and the PX4 Ubuntu development toolchain, then run:

```bash
./scripts/bootstrap.sh
./scripts/build_ros2.sh
source ros2_ws/install/setup.bash
export PX4_AUTOPILOT_PATH="$PWD/../PX4-Autopilot"
ros2 launch x650_sim sim.launch.py
```

Fish users must source `ros2_ws/install/setup.fish`, not the Bash setup file.
The `PX4_AUTOPILOT_PATH` variable can point to a PX4 checkout elsewhere.

## Troubleshooting

### `Package 'x650_sim' not found`

Build first and source the matching install tree:

```bash
./scripts/container.sh build
source ros2_ws/install-container/setup.bash
```

Use `install-container` inside the project container and `install` only for a
native build.

### PX4 waits for or times out on the Gazebo world

Start the complete stack with `./scripts/container.sh run` or `headless`.
Starting PX4 separately before the custom Gazebo world is available causes the
`Waiting for Gazebo world` timeout.

### Gazebo GUI does not appear

Run the command from a terminal opened in the active graphical desktop
session. GUI mode requires `DISPLAY`, `XAUTHORITY`, the X11 socket, and access
to `/dev/dri/renderD128`. Use `./scripts/container.sh headless` if no graphical
session is available.

### Podman fails on Btrfs OverlayFS

Install `fuse-overlayfs`. The launcher otherwise falls back to project-local
VFS storage, which works but consumes substantially more disk space.

### Fish reports errors while sourcing `setup.bash`

Do not source Bash setup scripts from Fish. Use the generated `setup.fish`, or
enter `./scripts/container.sh shell`, which starts Bash.

## PX4 controller baseline

The launch file restores PX4 v1.17 multicopter attitude, rate, and position
defaults and applies the proven `gz_x500` allocation layout. These startup
overrides prevent experimental values saved in `parameters.bson` from making
takeoff unstable.

PX4 allocation uses forward-right-down coordinates, while Gazebo model poses
use forward-left-up coordinates. The rotor Y signs must therefore be inverted;
copying them directly from SDF can turn roll stabilization into positive
feedback.

Do not copy SITL gains directly to the physical X650. Tune one axis at a time
using PX4 ULog setpoint-versus-response data.

## Project layout

```text
models/x650/                       Gazebo X650 model
worlds/x650_world.sdf              Test world and wall
ros2_ws/src/x650_sim/              ROS 2 package, nodes, and launcher
ros2_ws/src/px4_msgs/              PX4 message submodule
ros2_ws/src/Micro-XRCE-DDS-Agent/  DDS agent submodule
scripts/bootstrap.sh               PX4/native dependency bootstrap
scripts/build_ros2.sh              ROS 2 workspace build helper
scripts/container.sh               Podman/Docker entry point
scripts/qgroundcontrol.sh          QGroundControl installer/launcher
scripts/validate.sh                Offline structural validation
```

Run the offline checks after editing the model, launch file, or ROS nodes:

```bash
./scripts/validate.sh
```

## Security notes

- XRCE-DDS and local ROS discovery are unauthenticated; keep them on the
  development machine or a trusted network.
- GUI containers receive access to the desktop X11 socket. Use only trusted
  images and stop the container when finished.
- Never commit SSH keys, serial-device credentials, flight logs, or ROS bags.
