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

The bootstrap script recognizes both standalone Git clones and the `.git`
pointer files used by submodules. If an older checkout reports that
`ros2_ws/src/px4_msgs` already exists, pull the latest changes, initialize the
submodules as above, and rerun the build.

### 2. Install a container runtime

Install Podman or Docker. On CachyOS or Arch:

```bash
sudo pacman -S podman fuse-overlayfs
```

`fuse-overlayfs` is strongly recommended on Btrfs. Without it, the launcher
uses a slower project-local VFS store under `.container-storage/`.

The repository also publishes its pinned PX4/ROS/Gazebo environment to GitHub
Container Registry. After the `Publish simulation container` workflow has
completed, use it instead of the upstream image with:

```bash
podman pull ghcr.io/myahsap/bird-safe-window-marker-instalation-drone:jazzy

X650_CONTAINER_IMAGE=ghcr.io/myahsap/bird-safe-window-marker-instalation-drone:jazzy \
  ./scripts/container.sh build
```

The image contains the development environment; project source, PX4 source,
and generated builds remain bind-mounted so edits persist on the host. GitHub
packages are private by default. If anonymous pulls are required, change the
package visibility to public in the repository's package settings.

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

## Platform installation guides

The project always runs a Linux PX4/ROS/Gazebo environment. Linux hosts run
that container directly; Windows and macOS run it inside Docker Desktop's Linux
VM. ROS 2 Jazzy's binary packages target Ubuntu 24.04, so Debian, Windows, and
macOS should use the container workflow instead of attempting a native ROS/PX4
installation.

| Host | Recommended runtime | Gazebo GUI | Headless simulation |
| --- | --- | --- | --- |
| Debian Linux | Podman | Supported on a compatible X11/XWayland session | Supported |
| Windows 10/11 | Docker Desktop with WSL 2 | Not supported by the current launcher | Supported |
| macOS | Docker Desktop | Not supported by the current launcher | Supported |

Docker Desktop must have **host networking** enabled because PX4, Gazebo, the
DDS agent, ROS 2 shells, and QGroundControl communicate over host UDP. Docker
documents this as an opt-in feature in Desktop 4.34 and newer under
**Settings → Resources → Network → Enable host networking**.

### Windows 10/11 with WSL 2

1. Open an administrator PowerShell terminal, install or update WSL, and
   restart when requested:

   ```powershell
   wsl --install
   wsl --update
   ```

2. Install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
   with the WSL 2 backend. In Docker Desktop, enable integration for the Ubuntu
   WSL distribution and enable host networking.

3. Open the Ubuntu/WSL terminal. Keep the repository in the Linux filesystem
   under your home directory rather than under `/mnt/c`; this gives container
   builds Linux permissions and substantially better filesystem performance.

   ```bash
   sudo apt update
   sudo apt install -y git

   cd ~
   git clone --recurse-submodules \
     https://github.com/MYahsap/Bird-Safe-Window-Marker-Instalation-Drone.git
   cd Bird-Safe-Window-Marker-Instalation-Drone
   docker version
   ```

4. Build, validate, and start headless mode:

   ```bash
   ./scripts/container.sh build
   ./scripts/container.sh check
   ./scripts/container.sh headless
   ```

Run ROS 2 commands from a second WSL terminal with
`./scripts/container.sh shell`. WSLg can run ordinary Linux GUI applications,
but the project's Gazebo launcher additionally expects Linux X11 authorization
and `/dev/dri` device passthrough; use headless mode on Windows for now.

On a Windows-on-ARM computer, the published simulation image still uses the
`amd64` architecture. Enable x86-64 emulation in Docker Desktop and set:

```bash
export X650_CONTAINER_PLATFORM=linux/amd64
```

Install QGroundControl as a native Windows application rather than running the
project's Linux AppImage helper.

### macOS

1. Install [Docker Desktop for Mac](https://docs.docker.com/desktop/setup/install/mac-install/)
   for the correct processor: Apple silicon or Intel. Enable host networking
   in Docker Desktop settings.

2. Install Apple's command-line developer tools if `git` is unavailable:

   ```bash
   xcode-select --install
   ```

3. Clone and build from Terminal:

   ```bash
   git clone --recurse-submodules \
     https://github.com/MYahsap/Bird-Safe-Window-Marker-Instalation-Drone.git
   cd Bird-Safe-Window-Marker-Instalation-Drone

   ./scripts/container.sh build
   ./scripts/container.sh check
   ./scripts/container.sh headless
   ```

The published PX4 simulation image is built for `linux/amd64`. On Apple
silicon, install Rosetta when prompted and set the platform before building:

```bash
softwareupdate --install-rosetta
export X650_CONTAINER_PLATFORM=linux/amd64
./scripts/container.sh build
```

Emulation is slower than running on an Intel/AMD Linux host. The current GUI
launcher mounts Linux X11 sockets and Linux GPU render devices, neither of
which Docker Desktop exposes in the same form on macOS. Use `headless`, and
install QGroundControl as a native macOS application.

### Debian Linux

Debian 11 and later provide Podman in the distribution repositories. Install
Podman, Git, and `fuse-overlayfs`:

```bash
sudo apt update
sudo apt install -y git podman fuse-overlayfs
podman info
```

Clone, build, and validate:

```bash
git clone --recurse-submodules \
  https://github.com/MYahsap/Bird-Safe-Window-Marker-Instalation-Drone.git
cd Bird-Safe-Window-Marker-Instalation-Drone

./scripts/container.sh build
./scripts/container.sh check
```

Start the GUI from a terminal inside the active X11/XWayland desktop session:

```bash
./scripts/container.sh run
```

The GUI requires `DISPLAY`, a valid `XAUTHORITY` file, `/tmp/.X11-unix`, and
access to a `/dev/dri/renderD*` device. If any of those are unavailable, use:

```bash
./scripts/container.sh headless
```

Do not install Ubuntu ROS 2 binary packages directly on Debian. The container
already provides the matching Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and
PX4 toolchain.

Official platform references:

- [Install WSL and Ubuntu](https://learn.microsoft.com/windows/wsl/install/)
- [Run Linux GUI applications with WSLg](https://learn.microsoft.com/windows/wsl/tutorials/gui-apps)
- [Docker Desktop host networking](https://docs.docker.com/engine/network/drivers/host/)
- [Install Podman on Debian](https://podman.io/docs/installation)
- [Install ROS 2 Jazzy on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

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

### Coordinated end-effector pose control

The `end_effector_controller` lets you specify where the end of the arm should
be and which direction it should face. It solves the arm geometry and commands
the drone position, drone yaw, and joint angle together so that the tool pose
remains constrained in the Gazebo `world` frame.

Rebuild after adding or changing ROS 2 nodes:

```bash
./scripts/container.sh build
```

Start the coordinated controller in simulation:

```bash
./scripts/container.sh run \
  start_end_effector_control:=true \
  end_effector_auto_arm:=true
```

`end_effector_auto_arm` defaults to `false`. Set it to `true` only when the
simulation is clear for immediate autonomous movement. When coordinated
control is enabled, the launcher automatically disables the manipulator's
forward-only stabilizer and suppresses the conflicting hover example.

From a second container shell, publish a world-frame target. This example
places the tool tip at `(4, 0, 2)` metres and faces it horizontally along world
`+X`:

```bash
ros2 topic pub --once /x650/end_effector/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 4.0, y: 0.0, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

The orientation's local `+X` axis is the tool-facing direction. For example,
to face world `+Y` while holding the same position, use a +90° yaw quaternion:

```bash
ros2 topic pub --once /x650/end_effector/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 4.0, y: 0.0, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.707107, w: 0.707107}}}"
```

The controller supports tool yaw and elevation. Rotation about the tool's own
axis is ignored because the arm has only one actuated joint. Targets above
±80° elevation, outside the joint limits, or requiring the drone base below
0.25 m are rejected.

Monitor the solved drone pose and joint command:

```bash
ros2 topic echo /x650/end_effector/base_setpoint
ros2 topic echo /x650/end_effector/joint_setpoint
ros2 topic echo /x650/manipulator/joint_state
```

The target uses Gazebo world coordinates: X is North/forward, Y is West/left,
and Z is up. The controller converts these setpoints to PX4's NED convention.

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

Gazebo Transport is pinned to `127.0.0.1` by default. This prevents an active
VPN interface (such as Tailscale) from being advertised when the laptop is
offline, which otherwise leaves Gazebo's GUI blank and prevents PX4 from
finishing startup. Override it only when Gazebo clients must run on another
machine:

```bash
X650_GZ_IP=192.168.1.10 ./scripts/container.sh run
```

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
takeoff unstable. Airborne tilt is limited to ±12°, reducing horizontal
acceleration during coordinated ROS 2 pose movements.

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
