#!/usr/bin/env bash
# 데모 ②: 경매(Auction) 배차 — 유휴 로봇들이 목표까지 경로비용으로 입찰, 최근접(최저) 낙찰.
#   사용: ./demo_auction.sh [goal_vertex=3]
source "$(dirname "$0")/_lib.sh"
require_console
GOAL="${1:-3}"
echo "=== 데모②: 경매 배차  (goal v$GOAL) ==="
echo "▶ 1) 3대 모두 대기(IDLE) — 입찰 가능하게"
for r in pinky1 pinky2 pinky3; do mode "$r" IDLE; done; sleep 2.5
echo "▶ 2) 입찰가(각 로봇 최근접→목표 경로비용) 조회"
post bids "{\"goal\":$GOAL}" | python3 -c "
import sys,json;d=json.load(sys.stdin)
for b in sorted(d['bids'],key=lambda x:(x['bid'] is None,x['bid'] or 0)):
    print(f\"     {b['robot']}: 입찰가 {b['bid']}\")
print(f\"   → 예상 낙찰(최저 입찰): {d['winner']}\")"
echo "▶ 3) 경매 실행 (dispatcher가 최저 입찰 로봇에 배차)"
R=$(auction "$GOAL"); echo "   응답: $R"
echo "$R" | grep -q '"accepted":true' && echo "   ✅ 낙찰·배차됨 (위 예상 낙찰과 일치)" || echo "   결과: $R"
