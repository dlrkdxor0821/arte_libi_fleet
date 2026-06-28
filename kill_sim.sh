#!/usr/bin/env bash
# kill_sim.sh — libi 시뮬/관제 관련 프로세스를 "완전히" 정리한다.
#   ./kill_sim.sh
#
# 스크립트 내부에서 pkill/pgrep 하므로 런처 셸 자기-매칭(자기 자신을 죽이는) 위험이 없다.
# (직접 터미널에서 `pkill -f 'uvicorn aba_service'` 하면 그 명령 자신이 매칭돼 셸이 죽는 버그를 회피)
set +e

echo "[kill_sim] tmux 세션(libi) 종료"
tmux kill-session -t libi 2>/dev/null

PATTERNS=(
  "ros2 launch .*sim_slotcar"
  "ros2 launch .*view.launch"
  "sim_view.launch"
  "gz sim"
  "ruby .*gz sim"
  "gz-sim-server"
  "gz-sim-gui"
  "building_map_server"
  "parameter_bridge"
  "ros_gz_sim"
  "ros_gz_image"
  "robot_state_publisher"
  "joint_state_publisher"
  "fleet_node"
  "uvicorn aba_service"
  "console:app"
  "rviz2"
  "show_navgraph.py"
  "robot_markers.py"
  "drive_slotcar.py"
  "map_server"
  "lifecycle_manager"
  "static_transform_publisher"
)

killed=0
for p in "${PATTERNS[@]}"; do
  pids=$(pgrep -f "$p" 2>/dev/null)
  if [ -n "$pids" ]; then
    echo "  kill: $p  ($(echo $pids | tr '\n' ' '))"
    kill -9 $pids 2>/dev/null
    killed=$((killed + $(echo "$pids" | wc -w)))
  fi
done

# 포트 점유 해제 (콘솔 8001 · aba_service 8000)
for port in 8000 8001; do
  if fuser -k "${port}/tcp" 2>/dev/null; then echo "  port $port 해제"; fi
done

left=$(pgrep -f 'gz sim|fleet_node|uvicorn aba|rviz2|building_map_server' 2>/dev/null | wc -l)
echo "[kill_sim] 완료 — kill ${killed}개, 잔여 관련 프로세스 ${left}개"
