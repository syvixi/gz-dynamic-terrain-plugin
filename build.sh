#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rm -rf "$ROOT/build"
cmake -S "$ROOT" -B "$ROOT/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/build" -j"$(nproc)"
echo
echo "Use:"
echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\"$ROOT/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}\""

echo "Expected library: $ROOT/build/libgz-dynamic-terrain-system.so"
