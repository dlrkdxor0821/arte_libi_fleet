#!/usr/bin/env bash
# 데모 ①: 배차 수락 + 주행
#   로봇을 대기(IDLE)로 두고 특정 노드로 배차 → Dijkstra 경로 계획 → 주행 → 도착 → 완료.
#   사용: ./demo_dispatch.sh [robot=pinky1] [goal_vertex=5]
source "$(dirname "$0")/_lib.sh"
require_console
ROBOT="${1:-pinky1}"; GOAL="${2:-}"
echo "▶ 1) $ROBOT 를 대기(IDLE)로 (배차 받을 수 있게)"
mode "$ROBOT" IDLE; sleep 2; robot_line "$ROBOT"
if [ -z "$GOAL" ]; then   # 목표 미지정 → 로봇에서 가장 먼 정점(이동이 잘 보이게)
  GOAL=$(state | python3 -c "
import sys,json
d=json.load(sys.stdin); r=d['robots']['$ROBOT']; V=d['vertices']
print(max(range(len(V)),key=lambda i:(V[i][0]-r['x'])**2+(V[i][1]-r['y'])**2))")
fi
echo "=== 데모①: 배차 수락 + 주행  ($ROBOT → v$GOAL) ==="
echo "▶ 2) $ROBOT → v$GOAL 배차 요청"
R=$(dispatch "$ROBOT" "$GOAL"); echo "   응답: $R"
echo "$R" | grep -q '"accepted":true' || { echo "   ❌ 거절됨 — 로봇이 이미 목표에 있거나 busy일 수 있음"; exit 1; }
echo "   ✅ 수락 — Dijkstra 경로 계획됨 (콘솔에서 로봇색 경로가 목표까지 그려짐)"
echo "▶ 3) 주행 관찰 (도착 시 goals 에서 사라짐 = 완료. 교통 양보 시 WAITING)"
for i in $(seq 1 40); do
  D=$(state | python3 -c "
import sys,json
d=json.load(sys.stdin); r=d['robots']['$ROBOT']; g=d.get('goals',{}).get('$ROBOT')
print(f\"{r['mode']:7} ({r['x']:.1f},{r['y']:.1f})  {'→ v'+str(g) if g is not None else 'DONE'}\")")
  echo "   t$(printf %02d $i): $D"
  echo "$D" | grep -q DONE && { echo "   ✅ 도착 + 작업 완료"; exit 0; }
  sleep 1.5
done
echo "   ⏱ 관찰 시간 초과 (교통 대기 길어질 수 있음)"
