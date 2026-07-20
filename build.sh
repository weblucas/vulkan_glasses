#!/usr/bin/env bash
# Single entrypoint for the repo's two build systems.
#   ./build.sh core   -> CMake: vkg libraries + standalone apps + tests
#   ./build.sh ros2   -> colcon: the ROS2 overlay (needs a sourced ROS2 env)
#   ./build.sh all    -> core then ros2
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

cmd="${1:-all}"

build_core() {
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_CSV_RENDERER=ON -DBUILD_DATASET_VIEWER=ON -DBUILD_TESTS=ON
    cmake --build build -j"$(nproc)"
}

build_ros2() {
    if [[ -z "${ROS_DISTRO:-}" ]]; then
        echo "ROS2 not sourced (ROS_DISTRO unset). Source /opt/ros/<distro>/setup.bash first." >&2
        exit 1
    fi
    colcon build --paths ros2/vulkan_glasses_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
}

case "$cmd" in
    core) build_core ;;
    ros2) build_ros2 ;;
    all)  build_core; build_ros2 ;;
    *) echo "usage: $0 {core|ros2|all}" >&2; exit 1 ;;
esac
