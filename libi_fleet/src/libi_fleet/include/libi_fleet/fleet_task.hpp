#pragma once
#include <cstddef>
#include <string>
#include <vector>

// FMS 오케스트레이터(fleet_node)의 순수 데이터 모델 — 제어 상수 + 활성 task 상태.
// ROS·플러그인 비의존이라 폴더 이동·단위테스트에 독립적으로 따라다닌다.
namespace libi_fleet
{

constexpr double kArrive = 0.35;   // 도착 판정 거리(m)

// 교통 우선순위 인코딩(단일 int): tier(가장 큼) > task 나이(오래된=큼) > 배터리(낮은=큼).
//   tier: 순회=0 · 작업=1 · 충전복귀(CHARGE)=2 · 완전막힘(STUCK, 동적)=3
constexpr int kTierStep = 50000000;      // tier 간 간격 (age 최대치 kSeqMax*kAgeStep 보다 큼)
constexpr int kAgeStep  = 128;           // task 나이 1스텝 (배터리 최대 100 보다 큼)
constexpr int kSeqMax   = 100000;        // 나이 정규화 상한(세션 태스크 수 가정)
constexpr int kStopPrio = 4 * kTierStep; // STOP 장애물(사다리 밖, 항상 최상위)
constexpr int kMaxReroutes = 3;          // 교착 우회 최대 연속 횟수 — 초과 시 우회 포기·escalate(livelock 방지)
constexpr int kStuckTicks  = 100;        // 이동 지시됐는데 무진행이 이 틱(≈15s@150ms) 넘으면 slotcar stuck으로 보고 task 취소
constexpr int kRerouteWaitTicks = 33;    // 일반 WAIT 가 이 틱(≈5s@150ms) 넘으면 우회 재탐색(작업·순회)

struct ActiveTask
{
  std::string id;
  std::string robot;
  std::vector<int> path;   // 정점 인덱스 경로(시작 포함)
  size_t idx{1};           // 현재 향하는 path 인덱스
  bool moving{false};
  bool wait_logged{false};
  bool patrol{false};      // 순회 task: 끝에 도달해도 완료 안 하고 루프 반복
  bool stuck{false};       // 완전 막힘(우회 실패) → 우선순위 top 으로 escalate, 풀리면 원복
  int priority{0};         // (참고용) UI 지정 우선도. 교통 우선순위는 compute_priority 가 계산.
  int start_seq{0};        // 생성 순서(작을수록 오래됨) — 우선순위 나이 tiebreak
  int arm_actions{0};      // 팔 동작 횟수(배터리 소비 추정용)
  int reroutes{0};         // 연속 우회 횟수(노드 도달 시 리셋). 초과 시 우회 포기·escalate → livelock 방지.
  double last_x{0}, last_y{0};   // 직전 틱 위치 — 무진행(stuck) 감지용
  int no_move{0};          // 이동 지시 상태에서 무진행 틱 수
  int wait_ticks{0};       // 일반 WAIT 지속 틱 수(타임드 우회용). 진전 시 0 리셋.
};

}  // namespace libi_fleet
