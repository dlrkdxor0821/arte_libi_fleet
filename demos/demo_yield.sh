#!/usr/bin/env bash
# 데모 ④: 양보(yield) — 3대가 외곽 루프를 순회하다 좁은 교차 노드에서 경합하면,
#   한 대가 노드를 먼저 예약(점유)하고 다른 대는 양보(대기)한다.
#   노드 예약(ReservationDeadlock)만으로 정면충돌/후미추돌을 막는다.
#   → 콘솔에서 "점유 링"이 뜨고 그 옆 로봇이 잠깐 멈췄다 가는 걸 보세요.
source "$(dirname "$0")/_lib.sh"
require_console
echo "=== 데모④: 양보 (순회 중 노드 경합 → 자연 발생) ==="
echo "▶ 3대 순회 시작"
for r in pinky1 pinky2 pinky3; do mode "$r" PATROL; done
echo "▶ 26초 관찰 — 노드가 점유되면(occ) 그 노드로 가려던 로봇은 좌표가 멈춤(=양보)"
for i in $(seq 1 13); do
  state | python3 -c "
import sys,json
d=json.load(sys.stdin); occ=d.get('occupancy',{}); R=d['robots']
pos=' '.join(f\"{n[-1]}({R[n]['x']:.1f},{R[n]['y']:.1f})\" for n in sorted(R))
print(f\"  t$(printf %02d $i):  점유={occ if occ else '{}'}   {pos}\")"
  sleep 2
done
echo "✅ 점유 링이 뜨는 동안 옆 로봇 좌표가 고정되면 = 양보 성공 (콘솔에서 시각 확인)"
