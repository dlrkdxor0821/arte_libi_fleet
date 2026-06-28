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

### 2. 배차·교통 알고리즘 (config / 콘솔 드롭다운으로 교체)
`plugins/` 에 배치, `config`(파라미터) 또는 콘솔에서 런타임 교체. **정직한 상태 표시**:

| 분류 | 알고리즘 | 상태 |
|---|---|---|
| **배차** | `GreedyCost`(최근접) · `FarthestCost`(최원거리) | ✅ 실동작 |
| 배차 | `Hungarian` · `Auction/SSI` · `CBBA` · `MILP/VRP` · `GA/ACO` | ⚠️ 등록·선택 가능, **batch(task 큐) 추가 시 차이 발현** (현재 cost최소로 귀결) |
| **교통** | `EdgeNodeLock`(FIFO 양보) · `NoLock`(양보X) · `Priority`(우선순위 양보) | ✅ 실동작·서로 다름 |
| 교통 | `DijkstraReservation`(노드예약) | ✅ 동작 (DFS 데드락 감지는 추후) |
| 교통 | `CbsAstar` · `Orca` | ⚠️ 등록·선택 가능, **베이스라인** (CBS=경로예약 인터페이스 / ORCA=속도제어 필요) |

### 3. 중앙관제 — `service/aba_service/` (Python / FastAPI)
- `main.py` : `POST /tasks` → SubmitTask, 가용 로봇 없으면 503(거절)
- 3개 UI 동시 명령의 **상태 충돌**을 중앙에서 직렬 처리(단일 진실원천 + 선착순 배정 + 거절)

### 4. 테스트 콘솔 (웹 대시보드) — `service/aba_service/aba_service/console.py` (:8001)
blueprint 관제 화면. 한 곳에서:
- **라이브 맵(도면) + navgraph + 로봇** 캔버스 표시
- 로봇 **텔레메트리**(mode/배터리/위치) + **태스크 피드**(ASSIGNED→EXECUTING→COMPLETED)
- **명령**: 정점 직접 이동 / 특정 로봇 배차 / dispatcher 자동 배차
- **알고리즘 런타임 스왑** (배차·교통 드롭다운 → 적용)
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

```bash
# 0) 전제: ROS2 Jazzy + Gazebo Harmonic.
#    rmf_fleet_msgs/building_map_server → ~/open-rmf-test/rmf_ws (소스빌드),
#    pinky slotcar 패키지 → ~/personal_repo/open-rmf-practice/install (overlay).

# 1) 빌드 (rmf_ws 소스 후)
source /opt/ros/jazzy/setup.bash
source ~/open-rmf-test/rmf_ws/install/setup.bash
colcon build --packages-select libi_fleet_msgs libi_fleet

# 2) 한 방에 (Gazebo+RViz+fleet+콘솔)
./run_sim.sh                 # → 콘솔: http://localhost:8001
./run_sim.sh down            # 종료
./kill_sim.sh                # 잔여 프로세스 완전 정리

# (개별 실행도 가능: scripts/run_fleet.sh, scripts/run_console.sh)
```

---

## 검증된 동작 (이 repo에서 실측 확인)
- slotcar 멀티로봇 **navgraph 주행** (PathRequest, 데이터 확인)
- 콘솔 `task 제출 → 배차 → 주행 → 완료` end-to-end
- **교통 양보 A/B**: `EdgeNodeLock`(양보 발생) ↔ `NoLock`(양보 없음) 런타임 스왑 대조
- **배차 스왑**: `GreedyCost`(가까운 로봇) ↔ `FarthestCost`(먼 로봇) 다른 결과
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
