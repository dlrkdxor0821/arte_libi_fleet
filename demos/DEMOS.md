# LIBI FMS 데모 시나리오

배차(Auction)·교통협상(ReservationDeadlock) 알고리즘을 **사람들에게 시연**하기 위한 재사용 스크립트.
전부 콘솔 HTTP API(`:8001`)만 사용하므로 ROS 소싱 불필요 — **스택이 떠 있으면** 어디서든 실행.

## 준비
```bash
./run_sim.sh              # 스택 기동 (당신 터미널에서. sim+fleet+console)
# 브라우저로 http://localhost:8001 열기 (데모하는 동안 화면 공유)
```

## 시나리오
| 스크립트 | 시연 내용 |
|---|---|
| `demos/demo_reset.sh` | 3대 순회 + 배터리 100% 로 초기화 (데모 시작 전) |
| `demos/demo_dispatch.sh [robot] [goal]` | **배차 수락 + 주행** — 대기 로봇에 목표 배차 → Dijkstra 경로 → 주행 → 완료 |
| `demos/demo_auction.sh [goal]` | **경매 배차** — 유휴 로봇들 경로비용 입찰 → 최근접(최저) 낙찰 |
| `demos/demo_battery.sh [robot] [goal]` | **배터리 완주 관문** — 배터리 부족 거절 / 충분 수락 |
| `demos/demo_yield.sh` | **양보** — 순회 중 노드 경합 시 한 대 예약·다른 대 대기(정면충돌 방지) |

### 예시
```bash
./demos/demo_reset.sh
./demos/demo_dispatch.sh pinky1 6      # pinky1 을 v6 으로 (먼 노드 권장 = 이동이 잘 보임)
./demos/demo_auction.sh 3              # v3 경매 → 최근접 로봇 낙찰
./demos/demo_battery.sh pinky3 0
./demos/demo_yield.sh                  # 순회하며 양보 발생 관찰
```

## 데모 팁
- **배차 목표는 로봇에서 먼 노드**를 고르세요(가까운 노드=이동이 거의 없음).
- **양보/교착**은 순회 중 자연 발생 — 콘솔에서 점유 링 + 로봇 정지로 보입니다.
- 콘솔 화면 요소: 흰 간선 · **로봇색 방향 삼각형(▷ = 향하는 방향)** · **다음 목적지/최종 목적지** 패널 · 점유 노드 링.

## ⚠️ 알려진 제약 (sim/슬롯카)
- 드물게 슬롯카가 **급회전(노드 90°)에서 차선을 이탈해 벽에 끼여 멈추는** 경우가 있습니다.
  이는 **fleet 알고리즘이 아니라 슬롯카 주행/물리 문제**입니다(fleet 는 정상적으로 GRANT·경로 전송).
- 회복: 해당 로봇을 다른 노드로 재배차해도 안 빠지면 **`./run_sim.sh down && ./run_sim.sh` 로 sim 재시작**(재스폰).
- 데모 촬영 전 `demo_reset.sh` 로 초기화하고, 로봇이 끼면 재시작 후 다시.

## 검증 완료된 동작 (라이브)
- ✅ 자동 순회 (기동 후 3대 자동, kick 불필요)
- ✅ 배차 수락 + Dijkstra 경로 주행 + 완료
- ✅ 경매 최근접 낙찰 (최저 입찰가)
- ✅ 배터리 완주 관문 (부족 거절 / 충분 수락)
- ✅ 양보 (노드 예약 → 대기)
- ✅ 교착 감지 + 우회 (livelock 없음)
