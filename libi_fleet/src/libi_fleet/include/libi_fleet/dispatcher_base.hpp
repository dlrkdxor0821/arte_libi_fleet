#pragma once
#include <string>
#include <vector>

#include "libi_fleet/fms_types.hpp"
#include "libi_fleet/navgraph.hpp"

namespace libi_fleet
{

// 배차 전략 인터페이스(pluginlib base). goal 정점으로 보낼 로봇 1대를 고른다.
class DispatcherBase
{
public:
  virtual ~DispatcherBase() = default;
  // 가용(idle) 로봇 중 선택. 적합 로봇 없으면 "" (거절).
  virtual std::string assign(
    int goal_vertex,
    const std::vector<RobotInfo> & robots,
    const Navgraph & graph) = 0;
};

}  // namespace libi_fleet
