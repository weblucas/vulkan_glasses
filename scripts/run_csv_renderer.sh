#!/usr/bin/env bash
# Launcher for the CSV renderer. Resolves its own directory so the compiled
# shaders (installed next to this script under ./shaders) are found regardless
# of the current working directory.
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# --shader_folder is placed last so it wins over any value set earlier (e.g. a
# --shader_folder inside a --flagfile), pointing at the shaders bundled here.
exec "$DIR/vkg_csv_renderer" "$@" --shader_folder="$DIR/shaders"
