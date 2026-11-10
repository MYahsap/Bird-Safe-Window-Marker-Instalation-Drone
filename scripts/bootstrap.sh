#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_path="${PX4_AUTOPILOT_PATH:-$(dirname "$repo_root")/PX4-Autopilot}"
agent_path="$repo_root/ros2_ws/src/Micro-XRCE-DDS-Agent"
px4_msgs_path="$repo_root/ros2_ws/src/px4_msgs"

command -v git >/dev/null || { echo "git is required" >&2; exit 1; }

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" && "${X650_ALLOW_UNSUPPORTED_HOST:-0}" != "1" ]]; then
  cat >&2 <<EOF
Unsupported native host: ${PRETTY_NAME:-unknown Linux}

PX4 v1.17 + ROS 2 Jazzy is supported on Ubuntu 24.04. On Arch/CachyOS use
the project container instead:

  ./scripts/container.sh build
  ./scripts/container.sh run

Install Docker or Podman first if neither command exists. To deliberately try
a native build, set X650_ALLOW_UNSUPPORTED_HOST=1; this is not a supported path.
EOF
  exit 2
fi

if [[ ! -d "$px4_path/.git" ]]; then
  git clone --recursive --branch v1.17.0 --depth 1 \
    https://github.com/PX4/PX4-Autopilot.git "$px4_path"
fi

if [[ ! -d "$px4_msgs_path/.git" ]]; then
  git clone --branch release/1.17 --depth 1 \
    https://github.com/PX4/px4_msgs.git "$px4_msgs_path"
fi

if ! command -v MicroXRCEAgent >/dev/null && [[ ! -d "$agent_path/.git" ]]; then
  git clone --recursive --branch v2.4.3 --depth 1 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$agent_path"
fi

if ! python3 -c 'import menuconfig' >/dev/null 2>&1; then
  cat >&2 <<EOF
PX4 Python build dependencies are missing (module: menuconfig/kconfiglib).
Install the complete PX4 toolchain before building:

  bash "$px4_path/Tools/setup/ubuntu.sh" --no-nuttx

Then open a new terminal and run this script again. Avoid installing only
kconfiglib: the PX4 build requires several Python and system packages.
EOF
  exit 3
fi

echo "Building PX4 SITL..."
make -C "$px4_path" px4_sitl

echo "PX4 dependencies are ready. Build ROS 2 with:"
echo "  $repo_root/scripts/build_ros2.sh"


