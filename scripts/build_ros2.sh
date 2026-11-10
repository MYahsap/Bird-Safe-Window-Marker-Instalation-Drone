#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_underlay="${ROS_UNDERLAY:-/opt/ros/jazzy/setup.bash}"
build_base="${COLCON_BUILD_BASE:-build}"
install_base="${COLCON_INSTALL_BASE:-install}"
log_base="${COLCON_LOG_BASE:-log}"

if [[ ! -f "$ros_underlay" ]]; then
  cat >&2 <<EOF
ROS 2 Jazzy was not found at $ros_underlay.

On Ubuntu 24.04, install ROS 2 Jazzy first. On Arch/CachyOS, use:

  ./scripts/container.sh build
  ./scripts/container.sh run

This check prevents colcon from producing the misleading ament_cmake error.
EOF
  exit 4
fi

# ROS-generated setup files probe optional environment variables and are not
# compatible with Bash nounset mode. Disable it only while sourcing the
# underlay, then restore strict checking for this script.
set +u
source "$ros_underlay"
set -u

command -v ros2 >/dev/null || { echo "ros2 is unavailable after sourcing $ros_underlay" >&2; exit 5; }
command -v colcon >/dev/null || { echo "colcon is required" >&2; exit 5; }

cd "$repo_root/ros2_ws"
colcon --log-base "$log_base" build \
  --build-base "$build_base" \
  --install-base "$install_base" \
  --symlink-install

cat <<EOF
ROS 2 workspace built successfully.

Bash/Zsh:
  source "$repo_root/ros2_ws/$install_base/setup.bash"

Fish:
  source "$repo_root/ros2_ws/$install_base/setup.fish"
EOF

