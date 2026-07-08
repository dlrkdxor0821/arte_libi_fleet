#!/usr/bin/env bash
# 데모 ③: 배터리 완주 관문 — 배터리 부족 로봇은 배차 거절(방전 좌초 방지), 충분하면 수락.
#   소비 = 주행_m×1% + 팔동작×0.5%,  완주가능 = 배터리 ≥ 소비 + reserve(15%).
#   사용: ./demo_battery.sh [robot=pinky3] [goal=0]
source "$(dirname "$0")/_lib.sh"
require_console
ROBOT="${1:-pinky3}"; GOAL="${2:-0}"
echo "=== 데모③: 배터리 완주 관문  ($ROBOT → v$GOAL) ==="
mode "$ROBOT" IDLE; sleep 1.5
echo "▶ 1) 배터리 5% 로 낮추고 배차 → 거절 기대"
battery "$ROBOT" 5; sleep 0.5
R=$(dispatch "$ROBOT" "$GOAL"); echo "   응답: $R"
echo "$R" | grep -q 'insufficient_battery' && echo "   ✅ insufficient_battery 거절 (완주 불가)" || echo "   ⚠ 예상과 다름: $R"
echo "▶ 2) 배터리 100% 로 올리고 재시도 → 수락 기대"
battery "$ROBOT" 100; sleep 0.5
R=$(dispatch "$ROBOT" "$GOAL"); echo "   응답: $R"
echo "$R" | grep -q '"accepted":true' && echo "   ✅ 수락 (완주 가능)" || echo "   ⚠ 예상과 다름: $R"
