#!/usr/bin/env bash
set -euo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=${1:-"$workspace/build/p0-offline"}

cmake -S "$workspace/src/encos_core" -B "$build_dir" \
  -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build "$build_dir" --parallel
ctest --test-dir "$build_dir" --output-on-failure
