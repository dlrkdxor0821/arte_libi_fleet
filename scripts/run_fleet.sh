#!/usr/bin/env bash
# libi_fleet FMS 노드 실행
set -e
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/../install/setup.bash"
ros2 run libi_fleet fleet_node --ros-args \
  --params-file "$(dirname "$0")/../libi_fleet/config/algo_params.yaml"
