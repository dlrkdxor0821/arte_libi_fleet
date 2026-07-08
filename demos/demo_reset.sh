#!/usr/bin/env bash
# 데모 리셋: 3대 전부 순회(PATROL) + 배터리 100% — 데모 시작 전 깨끗한 상태로.
source "$(dirname "$0")/_lib.sh"
require_console
echo "=== 리셋: 3대 순회 + 배터리 100% ==="
for r in pinky1 pinky2 pinky3; do mode "$r" PATROL; battery "$r" 100; done
sleep 3
all_lines
echo "✅ 3대 자동 순회 중"
