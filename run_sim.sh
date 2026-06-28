#!/usr/bin/env bash
# run_sim.sh — Gazebo + RViz + fleet(FMS) + 관제 콘솔을 tmux 한 세션으로 한 번에 띄운다.
#
#   ./run_sim.sh          # up    : 세션 시작 후 attach (이미 떠있으면 attach)
#   ./run_sim.sh down     # down  : tmux 세션 + 잔여 gz/bridge/노드 정리
#   ./run_sim.sh status   # status: 세션/윈도우 상태
#
# 윈도우(탭): gazebo(sim+GUI, slotcar 3대) · fleet(FMS) · console(관제 :8001) · rviz(navgraph)
# tmux 단축키: Ctrl-b n(다음) / p(이전) / 0~3(번호) / d(detach) · 각 창 Ctrl-c(종료)
set -uo pipefail

SESSION="libi"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVGRAPH="$REPO/libi_fleet/maps/library/new_map.navgraph.yaml"
BUILDING="$REPO/libi_fleet/maps/library/new_map.building.yaml"
PRACTICE="$HOME/personal_repo/open-rmf-practice"          # pinky 패키지 빌드본(overlay)
RMF_WS="$HOME/open-rmf-test/rmf_ws"                       # rmf_fleet_msgs / building_map_server
SHOW_NAVGRAPH="$PRACTICE/scripts/rmf/show_navgraph.py"    # rviz navgraph 마커

# 모든 창 공통 환경 소싱
SRC="source /opt/ros/jazzy/setup.bash; \
source $RMF_WS/install/setup.bash; \
source $PRACTICE/install/setup.bash; \
source $REPO/install/setup.bash"

CLEANUP=("gz sim" "ruby .*gz sim" "building_map_server" "parameter_bridge" \
  "robot_state_publisher" "ros_gz_sim" "fleet_node" "uvicorn aba_service" \
  "show_navgraph.py" "rviz2" "sim_slotcar3")

command -v tmux >/dev/null || { echo "[run_sim] tmux 미설치 → sudo apt install tmux"; exit 1; }

case "${1:-up}" in
  down)
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "[run_sim] 세션 종료"
    for p in "${CLEANUP[@]}"; do pkill -9 -f "$p" 2>/dev/null; done
    echo "[run_sim] 정리 완료"; exit 0 ;;
  status)
    tmux has-session -t "$SESSION" 2>/dev/null && tmux list-windows -t "$SESSION" \
      || echo "[run_sim] 세션 없음"; exit 0 ;;
esac

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "[run_sim] 이미 실행 중 — attach"; exec tmux attach -t "$SESSION"
fi

# ① gazebo (sim + GUI + slotcar 3대 + building_map_server)
tmux new-session -d -s "$SESSION" -n gazebo
tmux set-option -t "$SESSION" remain-on-exit on
tmux send-keys -t "$SESSION:gazebo" \
  "$SRC; ros2 launch $REPO/scripts/sim/sim_slotcar3.launch.xml building_yaml:=$BUILDING gui:=true" C-m

# ② fleet (FMS) — sim 뜬 뒤
tmux new-window -t "$SESSION" -n fleet
tmux send-keys -t "$SESSION:fleet" \
  "$SRC; sleep 6; ros2 run libi_fleet fleet_node --ros-args -p navgraph_file:=$NAVGRAPH" C-m

# ③ 관제 콘솔 (FastAPI :8001) + 브라우저 자동 열기
tmux new-window -t "$SESSION" -n console
tmux send-keys -t "$SESSION:console" \
  "$SRC; sleep 4; cd $REPO/service/aba_service; ( sleep 5 && xdg-open http://localhost:8001 >/dev/null 2>&1 ) & LIBI_NAVGRAPH=$NAVGRAPH python3 -m uvicorn aba_service.console:app --host 0.0.0.0 --port 8001" C-m

# ④ rviz (map + navgraph + 로봇 3대 마커, fixed=map)
tmux new-window -t "$SESSION" -n rviz
tmux send-keys -t "$SESSION:rviz" \
  "$SRC; sleep 8; ros2 launch $REPO/scripts/sim/view.launch.py map:=$REPO/libi_fleet/maps/library/new_map.yaml navgraph:=$NAVGRAPH" C-m

cat <<EOF

  ── libi 관제 스택 기동 (tmux: $SESSION) ──
   gazebo  : Gazebo GUI + slotcar 3대
   fleet   : FMS (배차/교통 pluginlib)
   console : 관제 화면 → http://localhost:8001
   rviz    : map + navgraph 마커
  전환 Ctrl-b n/p · detach Ctrl-b d · 종료 ./run_sim.sh down

EOF
exec tmux attach -t "$SESSION"
