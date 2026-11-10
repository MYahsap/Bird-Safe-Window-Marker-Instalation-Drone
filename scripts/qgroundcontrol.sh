#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
qgc_dir="$repo_root/.tools/qgroundcontrol"
qgc_appimage="$qgc_dir/QGroundControl-x86_64.AppImage"
qgc_version="v5.1.4"
qgc_url="https://github.com/mavlink/QGroundControl/releases/download/$qgc_version/QGroundControl-x86_64.AppImage"
action="${1:-run}"

install_qgc() {
  command -v curl >/dev/null || { echo "curl is required" >&2; exit 1; }
  mkdir -p "$qgc_dir"
  echo "Downloading QGroundControl $qgc_version..."
  curl --fail --location --proto '=https' --tlsv1.2 \
    --output "$qgc_appimage.part" "$qgc_url"
  mv "$qgc_appimage.part" "$qgc_appimage"
  chmod +x "$qgc_appimage"
  echo "Installed: $qgc_appimage"
}

case "$action" in
  install)
    install_qgc
    ;;
  run)
    [[ -x "$qgc_appimage" ]] || install_qgc
    # CachyOS commonly has FUSE 3 but not libfuse.so.2. AppImage's extraction
    # mode avoids requiring the legacy FUSE package.
    if [[ -e /usr/lib/libfuse.so.2 || -e /usr/lib64/libfuse.so.2 ]]; then
      exec "$qgc_appimage"
    else
      exec "$qgc_appimage" --appimage-extract-and-run
    fi
    ;;
  path)
    printf '%s\n' "$qgc_appimage"
    ;;
  *)
    echo "Usage: $0 {install|run|path}" >&2
    exit 64
    ;;
esac

