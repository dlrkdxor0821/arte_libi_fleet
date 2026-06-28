// 교통협상 알고리즘 추가분 (옵시디언 'ROS2/FMS/교통 협상 알고리즘.md').
// Priority/DijkstraReservation 은 노드락 인터페이스(request_move)에 맞아 실동작.
// CbsAstar/Orca 는 시공간 경로 사전계획 / 연속 속도공간이라 노드락 인터페이스로는 부분만 — 베이스라인 등록.
#include <map>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "libi_fleet/traffic_base.hpp"

namespace libi_fleet
{

// 우선순위 협상 (§5) — 노드 경합 시 우선순위 높은 로봇이 차지. (데모 우선순위: pinky1>pinky2>pinky3)
//   EdgeNodeLock(FIFO) 과 달리 높은 우선순위는 대기하지 않고 노드를 가져간다 → 양보 비대칭.
class Priority : public TrafficBase
{
public:
  MoveDecision request_move(const std::string & robot, int node) override
  {
    auto it = owner_.find(node);
    if (it == owner_.end() || it->second == robot) { owner_[node] = robot; return MoveDecision::GRANT; }
    if (prio(robot) > prio(it->second)) { owner_[node] = robot; return MoveDecision::GRANT; }
    return MoveDecision::WAIT;
  }
  void release(const std::string & robot, int node) override
  {
    auto it = owner_.find(node);
    if (it != owner_.end() && it->second == robot) { owner_.erase(it); }
  }

private:
  std::map<int, std::string> owner_;
  static int prio(const std::string & r)
  { return r == "pinky1" ? 3 : r == "pinky2" ? 2 : r == "pinky3" ? 1 : 0; }
};

// Dijkstra + 노드예약 + DFS (§2) — 노드 예약. [DFS 데드락 감지는 대기-그래프 정보 필요 → 추후]
class DijkstraReservation : public TrafficBase
{
public:
  MoveDecision request_move(const std::string & robot, int node) override
  {
    auto it = owner_.find(node);
    if (it == owner_.end() || it->second == robot) { owner_[node] = robot; return MoveDecision::GRANT; }
    return MoveDecision::WAIT;
  }
  void release(const std::string & robot, int node) override
  {
    auto it = owner_.find(node);
    if (it != owner_.end() && it->second == robot) { owner_.erase(it); }
  }

private:
  std::map<int, std::string> owner_;
};

// CBS + space-time A* (§1) — 출발 전 시공간 경로 사전계획. [경로단위 예약 인터페이스 필요 → 노드락 베이스라인]
class CbsAstar : public TrafficBase
{
public:
  MoveDecision request_move(const std::string & robot, int node) override
  {
    auto it = owner_.find(node);
    if (it == owner_.end() || it->second == robot) { owner_[node] = robot; return MoveDecision::GRANT; }
    return MoveDecision::WAIT;
  }
  void release(const std::string & robot, int node) override
  {
    auto it = owner_.find(node);
    if (it != owner_.end() && it->second == robot) { owner_.erase(it); }
  }

private:
  std::map<int, std::string> owner_;
};

// ORCA / VO (§4) — 연속 속도공간 상호양보(분산). [속도 인터페이스 필요 → 노드락 아님, 항상 grant 베이스라인]
class Orca : public TrafficBase
{
public:
  MoveDecision request_move(const std::string &, int) override { return MoveDecision::GRANT; }
  void release(const std::string &, int) override {}
};

}  // namespace libi_fleet

PLUGINLIB_EXPORT_CLASS(libi_fleet::Priority, libi_fleet::TrafficBase)
PLUGINLIB_EXPORT_CLASS(libi_fleet::DijkstraReservation, libi_fleet::TrafficBase)
PLUGINLIB_EXPORT_CLASS(libi_fleet::CbsAstar, libi_fleet::TrafficBase)
PLUGINLIB_EXPORT_CLASS(libi_fleet::Orca, libi_fleet::TrafficBase)
