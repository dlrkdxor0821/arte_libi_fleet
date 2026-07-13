# CLAUDE.md — arte_libi_fleet

## ⚠️ 필수 의존성 — `open-rmf-practice` 저장소가 항상 있어야 함
- 경로: **`~/personal_repo/open-rmf-practice`**
- 제공: pinky slotcar sim + rmf 패키지(`pinky_description`, `pinky_gz_sim` 등). `run_sim.sh`가 이를 소싱하고, 슬롯카 world/urdf 가 여기 있음. **이 저장소 없으면 sim 이 안 뜬다.**
- 관련 파일(별도 저장소라 이쪽 커밋엔 안 들어감):
  - 슬롯카 주행 파라미터: `.../pinky_description/urdf/pinky_gz_slotcar.urdf.xacro`
  - 물리 world: `.../pinky_gz_sim/worlds/new_map.sdf`

## 🔎 미검증 변경 — 직접 확인 필요 (2026-07-08 세션)
아래 변경들이 있었습니다. **정상 동작하는지 직접 확인/검증 부탁드립니다** (테스트: `demos/` 스크립트, 실행은 `./run_sim.sh`):

- `libi_fleet/.../fleet_node.cpp` — 배차 `no_path`(start==goal) 수정 · 교착 livelock 수정 · `/fms/goals` 발행 · stuck 로봇 감지/회복 · **순회 중에도 특정 배차·경매 되도록**(순회 인터럽트)
- `service/aba_service/.../console.py` — 경로 시각화(흰 간선·다음노드 경로·간선 위 투영) · 방향 삼각형 · 다음/최종 목적지 패널 · 상태칩(순회/배차/대기) · entity 보간 · 10Hz 갱신
- `run_sim.sh` — headless 기본(`SIM_GUI=true`로 GUI) · console `--reload`/자동 브라우저 열기 제거 · fleet 시작 `sleep 12`
- `demos/` — 데모 시나리오 스크립트 (`DEMOS.md` 참고)
- (별도 저장소 `open-rmf-practice`) 슬롯카 주행 튜닝(회전↑·감속↑ → 차선 이탈/벽 끼임 방지) · `new_map.sdf` `max_step_size` 0.001→0.003

> 아직 커밋 전 상태일 수 있음 — 위 동작을 확인한 뒤 커밋 여부 판단하세요.
