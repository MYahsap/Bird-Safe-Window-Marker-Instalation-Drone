#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_path="${PX4_AUTOPILOT_PATH:-$(dirname "$repo_root")/PX4-Autopilot}"
image="${X650_CONTAINER_IMAGE:-docker.io/px4io/px4-dev-ros2-gazebo:main-jazzy}"
gz_partition="${X650_GZ_PARTITION:-x650_sim}"
host_display="${DISPLAY:-}"
host_xauthority="${XAUTHORITY:-}"
action="${1:-shell}"

if command -v podman >/dev/null 2>&1; then
  runtime=podman
  # Rootless kernel OverlayFS is not universally available over Btrfs. Keep a
  # project-local store so its driver never conflicts with Podman's global
  # store. Prefer fuse-overlayfs when installed and otherwise use portable VFS.
  export XDG_RUNTIME_DIR="/tmp/x650-xdg-${UID}"
  run_root="/tmp/x650-podman-${UID}"
  podman_tmp="/tmp/x650-libpod-${UID}"
  if command -v fuse-overlayfs >/dev/null 2>&1; then
    storage_root="$repo_root/.container-storage/podman-overlay"
    runtime_args=(
      --root "$storage_root"
      --runroot "$run_root"
      --tmpdir "$podman_tmp"
      --storage-driver overlay
      --storage-opt "overlay.mount_program=$(command -v fuse-overlayfs)"
    )
  else
    storage_root="$repo_root/.container-storage/podman-vfs"
    runtime_args=(
      --root "$storage_root"
      --runroot "$run_root"
      --tmpdir "$podman_tmp"
      --storage-driver vfs
    )
    echo "Using Podman VFS storage (install fuse-overlayfs for faster image storage)." >&2
  fi
  mkdir -p "$storage_root" "$XDG_RUNTIME_DIR" "$run_root" "$podman_tmp"
  chmod 700 "$storage_root" "$XDG_RUNTIME_DIR" "$run_root" "$podman_tmp"
elif command -v docker >/dev/null 2>&1; then
  runtime=docker
  runtime_args=()
else
  cat >&2 <<'EOF'
Docker or Podman is required on non-Ubuntu hosts.

On CachyOS/Arch, install one of them, for example:
  sudo pacman -S podman

Then rerun this command. Installing a system package requires your approval,
so this project does not do it automatically.
EOF
  exit 6
fi

mkdir -p "$px4_path"

run_args=(
  run --rm -it
  --network host
  --ipc host
  -e "PX4_AUTOPILOT_PATH=/workspace/PX4-Autopilot"
  -e "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
  -e "GZ_PARTITION=$gz_partition"
  -v "$repo_root:/workspace/sims:rw"
  -v "$px4_path:/workspace/PX4-Autopilot:rw"
  -w /workspace/sims
)

# Optional physical STS3215 adapter passthrough. Leave unset for simulation.
serial_device="${X650_SERIAL_DEVICE:-}"
if [[ -n "$serial_device" ]]; then
  serial_device="$(readlink -f "$serial_device")"
  if [[ ! -c "$serial_device" ]]; then
    echo "Serial device is not a character device: $serial_device" >&2
    exit 10
  fi
  run_args+=(--device "$serial_device:$serial_device")
fi
if [[ "$runtime" == podman ]]; then
  run_args+=(--group-add keep-groups)
fi

case "$action" in
  shell)
    command=(bash)
    ;;
  build)
    command=(bash -lc './scripts/bootstrap.sh && COLCON_BUILD_BASE=build-container COLCON_INSTALL_BASE=install-container COLCON_LOG_BASE=log-container ./scripts/build_ros2.sh')
    ;;
  run)
    if [[ -z "$host_display" || ! -d /tmp/.X11-unix ]]; then
      echo "XWayland display is unavailable; start GUI mode from the active desktop session." >&2
      exit 8
    fi
    if [[ -z "$host_xauthority" || ! -f "$host_xauthority" ]]; then
      echo "XWayland authorization file is unavailable (XAUTHORITY=$host_xauthority)." >&2
      echo "Start GUI mode from a terminal opened inside the GNOME session." >&2
      exit 9
    fi
    run_args+=(
      -e "QT_QPA_PLATFORM=xcb"
      -e "DISPLAY=$host_display"
      -e "XAUTHORITY=/tmp/x650.Xauthority"
      -v "/tmp/.X11-unix:/tmp/.X11-unix:ro"
      -v "$host_xauthority:/tmp/x650.Xauthority:ro"
    )
    # Give Gazebo direct-render access when available. Rootless Podman needs
    # the caller's supplementary groups retained for render-node permissions.
    # On hybrid Intel/NVIDIA laptops, exposing both nodes makes Mesa select the
    # NVIDIA device even though this image has no proprietary NVIDIA userspace
    # driver. Prefer the standard integrated-GPU render node; it is overrideable.
    gpu_device="${X650_DRI_DEVICE:-/dev/dri/renderD128}"
    if [[ -c "$gpu_device" ]]; then
      run_args+=(--device "$gpu_device:$gpu_device")
    else
      echo "No /dev/dri render devices found; Gazebo will use software rendering." >&2
      run_args+=(-e "LIBGL_ALWAYS_SOFTWARE=1")
    fi
    command=(bash -lc 'source /opt/ros/jazzy/setup.bash && source ros2_ws/install-container/setup.bash && ros2 launch x650_sim sim.launch.py')
    ;;
  headless)
    command=(bash -lc 'source /opt/ros/jazzy/setup.bash && source ros2_ws/install-container/setup.bash && ros2 launch x650_sim sim.launch.py headless:=true')
    ;;
  check)
    command=(bash -lc 'source /opt/ros/jazzy/setup.bash && source ros2_ws/install-container/setup.bash && ros2 pkg prefix x650_sim && ros2 launch x650_sim sim.launch.py --show-args')
    ;;
  smoke)
    command=(bash -lc 'source /opt/ros/jazzy/setup.bash && source ros2_ws/install-container/setup.bash && ros2 launch x650_sim sim.launch.py headless:=true start_agent:=false')
    ;;
  *)
    echo "Usage: $0 {shell|build|check|smoke|run|headless}" >&2
    exit 64
    ;;
esac

"$runtime" "${runtime_args[@]}" "${run_args[@]}" "$image" "${command[@]}"

