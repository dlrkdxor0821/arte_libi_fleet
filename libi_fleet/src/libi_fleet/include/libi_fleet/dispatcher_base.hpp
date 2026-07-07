#pragma once
#include <string>
#include <vector>

#include "libi_fleet/fms_types.hpp"
#include "libi_fleet/navgraph.hpp"

namespace libi_fleet
{

// 배터리 소비 모델 파라미터(sim 가정값). 완주 가능성 관문에서 사용.
struct EnergyParams
{
  double drain_per_m{1.0};     // 주행 1m당 배터리 소비 %
  double drain_per_act{0.5};   // 팔 동작 1회당 배터리 소비 %
  double reserve{15.0};        // 완주 후 남겨둘 최소 배터리 % (플로어)
};

// 배차 전략 인터페이스(pluginlib base). goal 정점으로 보낼 로봇 1대를 고른다.
class DispatcherBase
{
public:
  virtual ~DispatcherBase() = default;
  // 가용(idle) 로봇 중 선택. 적합 로봇 없으면 "" (거절).
  //  arm_actions: 이 작업의 팔 동작 횟수(배터리 소비 추정용).
  //  energy: 소비 모델. 완주 가능성(배터리 ≥ 소비 + reserve) 관문에 사용.
  virtual std::string assign(
    int goal_vertex,
    int arm_actions,
    const std::vector<RobotInfo> & robots,
    const Navgraph & graph,
    const EnergyParams & energy) = 0;
};

}  // namespace libi_fleet
