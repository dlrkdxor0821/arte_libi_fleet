# 실행 방법 (RUN)

`arte_libi_fleet`는 **FMS 두뇌(배차/교통)만** 만든 repo다. 로봇·시뮬레이터·통신 메시지는
RMF 생태계에서 **빌려 쓴다(overlay)**. 그래서 아래 토대들을 먼저 빌드/소싱해야 한다.

```
[arte_libi_fleet]           ← 우리가 만든 것: FMS 배차/교통 (fleet_node + 플러그인)
      │ overlay(source)로 얹힘
      ▼
[~/open-rmf-test/rmf_ws]        rmf_fleet_msgs(통신 언어) · building_map_server(맵 레벨)
[open-rmf-practice/pinky_*]     로봇 실체: slotcar Gazebo 월드·모델·description
[Gazebo (gz-sim)]               물리 시뮬레이터
```

우리가 실제로 빌드하는 건 `libi_fleet_msgs` · `libi_fleet` 둘뿐. 나머지는 밑에 깔리는 토대다.

---

## 0. 전제 (한 번만 준비)

| 토대 | 무엇 | 왜 필요 |
|---|---|---|
| **ROS 2 Jazzy** | `/opt/ros/jazzy` | 기반 |
| **Gazebo Harmonic** | `ros-jazzy-gz-sim-vendor` 등 | slotcar 물리 시뮬 (GUI는 GPU 필요) |
| **`~/open-rmf-test/rmf_ws`** | `rmf_fleet_msgs`·`building_map_server`·`rmf_building_map_tools` | 메시지 타입 + 맵 레벨(L1) 발행 → slotcar가 `/robot_state` 냄 |
| **`~/personal_repo/open-rmf-practice`** | `pinky_gz_sim`·`pinky_description` (slotcar) | 시뮬 로봇 스폰·주행 백엔드 |

> `libi_rmf_*`(open-rmf-practice 내)는 **순정 RMF 통합용**이라 우리 sim에는 **불필요**하다.
> (우리가 RMF dispatcher/traffic을 우리 것으로 갈아끼우는 프로젝트라, 그 경로는 안 씀)

---

## 1. 빌드 — ⚠️ 반드시 **repo 루트**에서

`install/`이 **`repo_root/install`**에 생겨야 `run_sim.sh`가 소싱한다.
`libi_fleet/` 안에서 빌드하면 `libi_fleet/install`에 생겨 스크립트가 못 찾는다.

```bash
cd ~/personal_repo/arte_libi_fleet          # ← repo 루트
source /opt/ros/jazzy/setup.bash
source ~/open-rmf-test/rmf_ws/install/setup.bash          # rmf_fleet_msgs
colcon build --packages-select libi_fleet_msgs libi_fleet # → repo_root/install
```

- 인터페이스(`SubmitTask.srv`·`SetBattery.srv` 등)를 바꾸면 `libi_fleet_msgs`부터 다시 빌드.

---

## 2. 통합 실행 — Gazebo + RViz + fleet + 콘솔 (tmux 한 세션)

```bash
cd ~/personal_repo/arte_libi_fleet
./run_sim.sh          # 4개 창(gazebo·fleet·console·rviz) 기동 후 attach
                      # → 관제 콘솔: http://localhost:8001
./run_sim.sh status   # 세션/창 상태
./run_sim.sh down     # 세션 종료 + 잔여 gz/노드 정리
./kill_sim.sh         # 포트·gz·fleet·console·rviz·브리지 완전 정리
```

- `run_sim.sh`가 **rmf_ws · open-rmf-practice · repo install을 모두 소싱**한다 → 빌드만 돼 있으면 한 줄로 끝.
- tmux: `Ctrl-b` 뒤 → `n`/`p`(창 전환) · `0~3`(번호) · `d`(detach) · 각 창 `Ctrl-c`(그 프로세스 종료)

---

## 3. 개별 / 헤드리스 실행 (디버깅 · GPU 없이)

```bash
scripts/run_fleet.sh      # FMS 노드만
scripts/run_console.sh    # 관제 콘솔만 :8001 — sim 없이 UI·서비스 점검 가능
```

- **콘솔만** 띄우면 Gazebo(GPU) 없이도 배차/배터리/모드 서비스와 UI를 확인할 수 있다(로봇은 sim 연결 시 표시).

---

## 4. 콘솔(:8001)에서 할 수 있는 것

- **왼쪽 상태 패널**: 로봇별 카드(모드 칩 · 배터리 바 · 현재 task · 위치)
- **① 특정 로봇 배차** / **② 자동 배차(경매)** — 각 배차에 **로봇팔 동작 횟수** 입력(배터리 완주 판단 반영)
- **③ 로봇 상태**: 순회 · 대기 · 정지 · **충전복귀(CHARGE)** · **배터리 %(각 로봇 설정 → 완주 관문·우선순위에 실제 반영)**
- 라이브 맵 + navgraph + 로봇, 태스크 피드, 정점 편집(추가/이동/삭제/차선 → 저장 시 fleet 리로드)

---

## 5. 트러블슈팅

### (A) `package 'pinky_gz_sim' not found` — 오버레이 setup 이 stale
`open-rmf-practice/install/setup.bash` 가 일부 패키지만 등록한 경우. 클린 재빌드로 복구:
```bash
cd ~/personal_repo/open-rmf-practice
rm -rf build install log
colcon build --symlink-install        # 이 워크스페이스는 symlink-install 로 만들어짐
```
⚠️ 이 워크스페이스에 plain `colcon build` 를 섞어 돌리지 말 것 — 설치 모드가 혼용되면
`could not create install/...` 에러가 나고, 아래 (B) 증상으로 이어진다.

### (B) `package 'libi_rmf_tasks' not found` — Gazebo 런치가 통째로 죽음
sim launch 파일은 `libi_rmf_*` 를 참조하지 않는데도 이 에러가 난다. 원인은:
`ros_gz_sim/launch/gz_sim.launch.py` 의 `get_paths()` 가 **설치된 모든 패키지를 순회**하며
`get_package_share_directory()` 를 호출하는데, **깨진(부분설치) 패키지가 ament 인덱스에 이름만
등록돼 있으면 거기서 크래시**한다. 즉 sim 에 불필요한 `libi_rmf_*` 의 깨진 등록이 gz 런치 전체를 오염.
- **해결**: 깨진 패키지의 설치물을 인덱스에서 제거(소스는 유지, 필요시 재빌드).
  ```bash
  cd ~/personal_repo/open-rmf-practice
  rm -rf install/libi_rmf_* build/libi_rmf_*      # 우리 sim 은 pinky 만 필요 → 안전
  ros2 pkg list | grep libi_rmf                   # 아무것도 안 나와야 함
  ```
- 이러면 Gazebo 가 뜨고, 맵 서빙 + slotcar 3대 스폰 + 물리(/clock) 진행까지 확인됨.

### (C) Gazebo 창이 안 뜸 / `/robot_state` 안 나옴 — GPU 렌더링
> ⚠️ **`/robot_state` 가 안 나온다면 GPU보다 drive 모드를 먼저 의심하라** → [`docs/트러블슈팅.md`](docs/트러블슈팅.md).
> `libEGL warning: ... driver (null)` 는 대개 Mesa→NVIDIA 폴백 노이즈일 뿐(카메라는 실제 초기화됨).
> GPU가 진짜 원인인 건 3D 창/센서 렌더가 통째로 죽는 경우다.

로그에 `libEGL warning: ... driver (null)` · `egl: failed to create dri2 screen` 가 보이면
**GPU 드라이버가 렌더링에 안 잡히는 것**(예: 원격/헤드리스 세션, NVIDIA 드라이버 미설정).
- 물리는 돌지만 slotcar 센서/렌더가 초기화 못 해 `/robot_state` 가 안 나올 수 있다.
- 확인: `glxinfo -B` 로 renderer 가 실제 GPU/llvmpipe 인지. NVIDIA면 드라이버·`nvidia-smi` 상태 점검.
- 급하면 GUI 없이(`gui:=false`) 물리만 돌리고, **로봇은 콘솔(:8001)에서** 보면 GPU 무관.

### 콘솔만 띄웠는데 맵/navgraph가 안 나옴
`run_console.sh`가 `LIBI_NAVGRAPH`를 안 잡아 기본 경로(오타)로 맵을 못 읽는다. 직접 지정:
```bash
LIBI_NAVGRAPH=~/personal_repo/arte_libi_fleet/libi_fleet/maps/library/new_map.navgraph.yaml \
  scripts/run_console.sh
```

### 프로세스가 남아 재실행이 꼬임
```bash
./kill_sim.sh     # gz·fleet·console·rviz·브리지·포트 완전 정리
```
