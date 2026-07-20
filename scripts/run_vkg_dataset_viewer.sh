#!/usr/bin/env bash
# Launcher for the .h5 image viewer. Resolves its own directory so it can be run
# from any working directory. The viewer needs no shaders.
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$DIR/vulkan_glasses_h5_viewer" "$@"
