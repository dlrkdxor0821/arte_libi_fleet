#pragma once
#include <string>
#include <utility>
#include <vector>

namespace libi_fleet
{

enum class MoveDecision { GRANT, WAIT, DEADLOCK };

// 교통협상 전략 인터페이스(pluginlib base). navgraph 노드 진입 허가/대기.
// 전체 로봇을 한 인스턴스가 본다(공유 1개).
class TrafficBase
{
public:
  virtual ~TrafficBase() = default;
  // robot 이 from_node→to_node 이동 요청. from==to 는 현재 노드 점유(claim).
  // priority: 로봇의 현재 우선순위(높을수록 우선). 교착 시 최저 우선순위가 양보.
  //  GRANT   : 목표 노드 예약 성공.
  //  WAIT    : 점유 중 → 양보 대기(우선순위 높아 직진 대기 포함).
  //  DEADLOCK: 대기 사이클(교착)에서 이 로봇이 최저 우선순위 → 호출측이 우회(재경로).
  // ※ 노드예약만으로 정면충돌·후미추돌이 다 막히므로 엣지예약은 두지 않는다.
  virtual MoveDecision request_move(const std::string & robot, int from_node, int to_node,
                                    int priority) = 0;
  // robot 이 node 점유 해제. 취소/정지용.
  virtual void release(const std::string & robot, int node) = 0;
  // 출발/도착 순간: 떠나는 노드 해제.
  virtual void release_node(const std::string & robot, int node) = 0;
  // 현재 점유 중인 (노드, 로봇) 목록 — 시각화용.
  virtual std::vector<std::pair<int, std::string>> occupancy() const = 0;
};

}  // namespace libi_fleet
