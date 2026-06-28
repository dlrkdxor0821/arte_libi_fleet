#pragma once
#include <string>

namespace libi_fleet
{

enum class MoveDecision { GRANT, WAIT };

// 교통협상 전략 인터페이스(pluginlib base). navgraph 노드 진입 허가/대기.
// 전체 로봇을 한 인스턴스가 본다(공유 1개).
class TrafficBase
{
public:
  virtual ~TrafficBase() = default;
  // robot 이 target_node 로 진입 요청. 비었으면 GRANT(+점유), 점유 중이면 WAIT.
  virtual MoveDecision request_move(const std::string & robot, int target_node) = 0;
  // robot 이 node 점유 해제.
  virtual void release(const std::string & robot, int node) = 0;
};

}  // namespace libi_fleet
