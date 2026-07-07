# arte_libi_fleet — Libi FMS (도서관 사서 협동로봇 관제)

**ABA / 리비(Libi)** — 주행로봇 + 로봇팔(모바일 매니퓰레이터)로 도서관 사서를 돕는 협동로봇의
**Fleet Management System(관제)** 구현. 여러 UI 명령을 중앙에서 받아 **배차(누가 할지) + 교통협상(주행 중 충돌 회피)** 으로
멀티 로봇을 지휘한다.

> 핵심 목표: **nav2 대신 sim(slotcar)에서 배차·교통 알고리즘을 pluginlib로 갈아끼우며 테스트**하는 것.
> 설계 문서: `docs/superpowers/specs/2026-06-28-libi-fleet-design.md` (gitignore — 로컬).

---

## 전체 흐름

```
[UI(브라우저/콘솔)]
   │ HTTP
[service/aba_service]  FastAPI 중앙관제 입구 (rclpy 브리지)
   │ ROS2 (SubmitTask 서비스)
┌── libi_fleet (FMS 두뇌, C++) ────────────────────────────────┐
│  TaskManager(무큐·즉시거절) → Dispatcher★ → fleet_adapter      │
│                                  └ 매 이동 → Traffic★(공유 1개) │
└──────────────────────────────────┬──────────────────────────┘
            domain_bridge(실배포) / 단일도메인(sim) │
        ┌──────────────┴──────────────┐
   [로봇 = 1 도메인]            ...
     Drive Controller (주행, nav2/slotcar)  → Handy Controller (팔)
   ★ = pluginlib 전략 (config / 콘솔에서 교체)
```

- **fleet ↔ Drive only**: fleet은 로봇당 Drive와만 통신(`navigate`·`perform_action`). 팔(Handy)은 Drive가 지휘.
- **배차·교통은 RMF 흐름 참고 + 직접 구현**(우선순위 정책 반영). RMF의 dispatcher/rmf_traffic은 미사용.
- **실물 기준 설계** — sim은 fleet_adapter의 로봇 백엔드(slotcar `PathRequest` ↔ 실물 nav2 `NavigateToPose`)만 교체.

---

## 주요 기능

### 1. FMS 코어 — `libi_fleet/` (C++ / ament_cmake / pluginlib)
- `fleet_node` : TaskManager(무큐·거절) + 배차/교통 플러그인 로딩 + 주행 상태머신
- `navgraph` : navgraph(yaml) 로드 + 최근접 정점 + **Dijkstra** 경로
- 런타임 서비스: `/fms/submit_task`(배차) · `/fms/set_plugins`(알고리즘 스왑) · `/fms/reload_navgraph`(정점 편집 반영)
- 인터페이스(`libi_fleet_msgs`): `SubmitTask`·`SetPlugins`(srv), `Navigate`·`PerformAction`(action), `TaskState`·`RobotState`(msg)

### 2. 배차·교통 알고리즘 (pluginlib · `/fms/set_plugins` 로 런타임 교체)
`plugins/` 에 배치, `plugins.xml` 등록. 현재 구현·빌드된 전략은 아래 2개.

**배차 — `Auction`** (`plugins/auction.cpp`)
- 유휴 로봇이 goal 까지 **Dijkstra 경로비용으로 입찰 → 최저가 낙찰** (단일 task 는 최근접비용에 귀결, task 큐를 얹으면 SSI).
- **배터리 = 완주 가능성 관문**(입찰가 가중치 아님, 실무/RMF 방식):
  `소비 = 주행_m×drain_per_m(1%) + 팔동작수×drain_per_act(0.5%)`, `완주가능 = 배터리 ≥ 소비 + reserve(15%)`.
  못 넘으면 입찰 제외(강제 배정도 `insufficient_battery` 거절). 입찰가는 거리만, 배터리는 순위에 안 들어감.

**교통 — `ReservationDeadlock`** (`plugins/reservation_deadlock.cpp`)
- **노드 예약**: 목표 노드 잠금(진입 차단). "타깃 확보 후 출발 노드 해제" 규율이라 **노드예약만으로 정면충돌까지 방지**(엣지예약 불필요).
- **데드락 해소**: wait-for 그래프 **DFS 사이클 감지 → 최저 우선순위가 우회(reroute)**. 단순 점유 충돌은 그냥 대기.
- **우선순위 사다리**(높을수록 안 비킴): `완전막힘(동적)` > `충전복귀(CHARGE)` > `작업`(동률=시작 오래된순→배터리 낮은순) > `순회`. STOP=장애물(사다리 밖, 다른 로봇이 우회).
  - **완전 막힘**(우회 실패) → 우선순위를 최상위로 escalate(주변이 비켜줌), 풀리면 원복.

> 파라미터(fleet_node ROS param, sim 가정값): `battery_drain_per_m`(1.0) · `battery_drain_per_act`(0.5) · `battery_reserve_pct`(15.0).
> 배터리는 sim(slotcar) 값을 무시하고 **내부 기본 100%**, 콘솔/`/fms/set_battery` 로 각 로봇 설정.

### 3. 중앙관제 — `service/aba_service/` (Python / FastAPI)
- `main.py` : `POST /tasks` → SubmitTask, 가용 로봇 없으면 503(거절)
- 3개 UI 동시 명령의 **상태 충돌**을 중앙에서 직렬 처리(단일 진실원천 + 선착순 배정 + 거절)

### 4. 테스트 콘솔 (웹 대시보드) — `service/aba_service/aba_service/console.py` (:8001)
blueprint 관제 화면. 한 곳에서:
- **라이브 맵(도면) + navgraph + 로봇** 캔버스 표시
- 로봇 **텔레메트리**(mode/배터리/위치) + **태스크 피드**(ASSIGNED→EXECUTING→COMPLETED)
- **명령**: 정점 직접 이동 / 특정 로봇 배차 / dispatcher 자동 배차 — 각 배차에 **로봇팔 동작 횟수** 입력(배터리 완주 판단에 반영)
- **로봇 상태**: 순회 · 대기 · 정지 · **충전복귀(CHARGE, 교착 시 최우선)** · **배터리 %(각 로봇 설정 → 완주 관문·우선순위에 실제 반영)**
- **알고리즘 런타임 스왑** (배차·교통 `/fms/set_plugins` → 적용)
- **정점 편집**: 추가(빈 곳 클릭)·이동(드래그)·삭제(우클릭)·차선(정점→정점) → 저장 시 fleet 리로드

### 5. 시뮬 & RViz — `scripts/sim/`
- `sim_slotcar{,2,3}.launch.xml` : slotcar 1/2/3대 headless bringup (+`gui:=true`로 Gazebo GUI)
- `view.launch.py` + `libi.rviz` : RViz에 맵 + navgraph + **로봇 마커(robot_markers.py)**
- `drive_slotcar.py` : PathRequest 수동 주행 도구

### 6. 스크립트 (최상위 + `scripts/`)
| 스크립트 | 역할 |
|---|---|
| **`./run_sim.sh`** | Gazebo + RViz + fleet + 콘솔을 **tmux 한 세션**으로 (`down`/`status` 지원) |
| **`./kill_sim.sh`** | 관련 프로세스(gz·fleet·console·rviz·브리지·포트) **완전 정리** |
| `scripts/run_fleet.sh` · `run_console.sh` · `run_aba_service.sh` | 개별 실행 |

---

## 폴더 구조

```
arte_libi_fleet/
├── run_sim.sh · kill_sim.sh            # 통합 실행 / 완전 정리
├── libi_fleet/src/
│   ├── libi_fleet_msgs/                # 인터페이스 (srv/msg/action)
│   └── libi_fleet/                     # FMS 코어 (C++)
│       ├── src/{fleet_node,navgraph}.cpp
│       ├── include/libi_fleet/*.hpp    # navgraph / dispatcher_base / traffic_base / types
│       ├── plugins/                    # ★ 배차·교통 알고리즘
│       │   ├── greedy_cost · farthest_cost · dispatch_more   (배차)
│       │   └── edge_node_lock · no_lock · traffic_more        (교통)
│       └── plugins.xml
├── service/aba_service/aba_service/
│   ├── main.py · ros_bridge.py         # 중앙관제 입구
│   └── console.py                      # 테스트 콘솔 (웹 대시보드)
├── controller/
│   ├── libi_drive_controller/src/{libi_drive, pinky_pro(gitignore)}
│   └── libi_handy_controller/
├── scripts/sim/                        # sim launch · rviz · 마커 · 주행도구
└── libi_fleet/maps/library/            # navgraph · building.yaml · pgm (맵 데이터)
```

---

## 실행 방법

### 0) 전제 — 오버레이 2개 + Gazebo
- **ROS 2 Jazzy** + **Gazebo Harmonic** (GUI 렌더는 GPU 필요 · **콘솔 :8001만 쓰면 GPU 무관**)
- `~/open-rmf-test/rmf_ws` — `rmf_fleet_msgs` · `building_map_server` (소스빌드)
- `~/personal_repo/open-rmf-practice/install` — pinky slotcar 패키지 (overlay)

### 1) 빌드 — ⚠️ **반드시 repo 루트에서**
`install/` 이 **`repo_root/install`** 에 생겨야 run 스크립트가 소싱한다. `libi_fleet/` 안에서 빌드하면
`libi_fleet/install` 에 생겨 run 스크립트가 못 찾는다.
```bash
cd ~/personal_repo/arte_libi_fleet          # ← repo 루트
source /opt/ros/jazzy/setup.bash
source ~/open-rmf-test/rmf_ws/install/setup.bash            # rmf_fleet_msgs
colcon build --packages-select libi_fleet_msgs libi_fleet   # → repo_root/install
```
> `SubmitTask.srv`(팔 동작)·`SetBattery.srv`(신규) 등 인터페이스를 바꾸면 `libi_fleet_msgs` 부터 다시 빌드.

### 2) 통합 실행 (Gazebo + RViz + fleet + 콘솔 · tmux 한 세션)
```bash
./run_sim.sh          # 4개 창(gazebo·fleet·console·rviz) 기동 후 attach → 콘솔 http://localhost:8001
./run_sim.sh status   # 세션/창 상태
./run_sim.sh down     # 세션 종료 + 잔여 gz/노드 정리
./kill_sim.sh         # 포트·gz·fleet·console·rviz·브리지 완전 정리
```
- run_sim.sh 가 **rmf_ws · open-rmf-practice · repo install 을 모두 소싱**하므로, 빌드만 돼 있으면 한 줄로 끝.
- tmux: `Ctrl-b` 뒤 `n`/`p`(창 전환) · `0~3`(번호) · `d`(detach) · 각 창 `Ctrl-c`(그 프로세스 종료)

### 3) 개별 / 헤드리스 실행 (디버깅 · GPU 없이)
```bash
scripts/run_fleet.sh      # FMS 노드만 (rmf_fleet_msgs 필요 → 안 뜨면 먼저 `source ~/open-rmf-test/rmf_ws/install/setup.bash`)
scripts/run_console.sh    # 관제 콘솔만 :8001 — sim 없이 UI·서비스(배차/배터리/모드) 점검 가능
```

---

## 검증된 동작 (이 repo에서 실측 확인)
- slotcar 멀티로봇 **navgraph 주행** (PathRequest, 데이터 확인)
- 콘솔 `task 제출 → 배차 → 주행 → 완료` end-to-end
- **배차 완주 관문**: 배터리 낮은 로봇 입찰 제외(`insufficient_battery`) · 팔 동작 수만큼 소비↑ (헤드리스 실측 4/4)
- **교통**: 정면 교차 시 **노드예약 차단 → DFS 데드락 감지 → 우회**로 두 로봇 완주 · **우선순위 tier**(CHARGE가 task-나이 tiebreak 덮어써 양보자 바뀜) 대조 (헤드리스 실측)
- **정점 편집**: 추가→저장→fleet 리로드→새 정점으로 배차 주행
- RViz 맵+navgraph+로봇 마커, 콘솔 맵+navgraph+로봇 표시

## 한계 / TODO
- **GUI 렌더**(Gazebo/RViz)는 GPU 필요. 콘솔(:8001)은 무관.
- **batch 배차**(Hungarian 등) 진짜 차이 → **task 큐** 추가 필요.
- **CBS+A*** → traffic에 **경로예약 인터페이스** 추가 시 진짜 구현 가능(그래프 기반, slotcar 적합).
- **ORCA/VO** → **속도(cmd_vel) 제어** 필요 → 실물 nav2/diffdrive(M5)에서 자연스러움.
- 로봇팔(`perform_action`) 실제화(MoveIt2/티치포즈) = M4 단계.

## 참고 자산 (사용자 머신 경로)
- `~/open-rmf-test/rmf_ws` — rmf_fleet_msgs · building_map_server · rmf_demos
- `~/personal_repo/open-rmf-practice` — pinky slotcar 패키지 빌드본 · navgraph 도구 · RViz 셋업
- `~/Documents/obsidian/ASD/ROS2/FMS` — 배차·교통 알고리즘 기술조사 노트
