#!/usr/bin/env bash
# headless slotcar sim bring-up (1대).
# pinky 패키지·world 는 open-rmf-practice 빌드본을 overlay 로 재사용,
# 맵 데이터(building.yaml)는 본 repo libi_fleet/maps/library 사용.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source ~/open-rmf-test/rmf_ws/install/setup.bash                 # rmf_fleet_msgs, building_map_server
source ~/personal_repo/open-rmf-practice/install/setup.bash      # pinky_description, pinky_gz_sim
source "$REPO/install/setup.bash" 2>/dev/null || true            # libi_*

exec ros2 launch "$REPO/scripts/sim/sim_slotcar.launch.xml" \
  building_yaml:="$REPO/libi_fleet/maps/library/new_map.building.yaml" \
  "$@"
