// 교통관제 = 노드 예약 + wait-for 그래프 DFS 교착 감지.
//   · 로봇은 자기가 서 있는 노드를 점유(claim)하고, 인접 노드로 이동 요청.
//   · GRANT 시 목표 노드를 예약(출발 노드는 출발 순간 release).
//   · 목표 노드가 점유중이면 대기.
//   · 대기가 사이클을 이루면(A→B, B→A …) DFS 로 감지 → DEADLOCK 반환(호출측이 우회).
//   ※ "타깃 노드를 먼저 확보한 뒤 출발 노드를 놓는" 규율이라, 노드예약만으로
//      정면충돌(head-on)·후미추돌이 전부 막힌다 → 엣지예약은 두지 않는다.
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "libi_fleet/traffic_base.hpp"

namespace libi_fleet
{

class ReservationDeadlock : public TrafficBase
{
public:
  MoveDecision request_move(const std::string & robot, int from, int to, int priority) override
  {
    prio_[robot] = priority;   // 로봇의 현재 우선순위 갱신

    // 현재 노드 점유(claim): from==to. 시작 노드 확보에 사용.
    if (from == to) {
      auto it = node_owner_.find(to);
      if (it == node_owner_.end() || it->second == robot) {
        node_owner_[to] = robot;
        waitfor_.erase(robot);
        return MoveDecision::GRANT;
      }
      return MoveDecision::WAIT;
    }

    // 경합 상대 찾기: 목표 노드 점유자.
    std::string blocker = owner_of_node(to, robot);

    if (blocker.empty()) {
      node_owner_[to] = robot;              // 목표 노드 예약
      waitfor_.erase(robot);
      return MoveDecision::GRANT;
    }

    // 대기 → wait-for 갱신 후 사이클 검사.
    waitfor_[robot] = blocker;
    std::vector<std::string> cyc;
    if (find_cycle(robot, cyc)) {
      // 사이클 내 최저 우선순위(동률이면 이름 큰 쪽)가 양보 = 우회.
      const std::string & victim = lowest_priority(cyc);
      if (victim == robot) {
        waitfor_.erase(robot);   // 이 로봇이 양보자 → 물러나(우회)므로 대기변 제거
        return MoveDecision::DEADLOCK;
      }
      // 우선순위 높음 → 낮은 쪽이 빠질 때까지 대기 후 직진.
    }
    return MoveDecision::WAIT;
  }

  void release(const std::string & robot, int node) override
  {
    auto it = node_owner_.find(node);
    if (it != node_owner_.end() && it->second == robot) { node_owner_.erase(it); }
    waitfor_.erase(robot);
  }

  void release_node(const std::string & robot, int node) override
  {
    auto it = node_owner_.find(node);
    if (it != node_owner_.end() && it->second == robot) { node_owner_.erase(it); }
    waitfor_.erase(robot);
  }

  std::vector<std::pair<int, std::string>> occupancy() const override
  {
    std::vector<std::pair<int, std::string>> out;
    for (const auto & kv : node_owner_) { out.emplace_back(kv.first, kv.second); }
    return out;
  }

private:
  std::string owner_of_node(int node, const std::string & self) const
  {
    auto it = node_owner_.find(node);
    return (it != node_owner_.end() && it->second != self) ? it->second : "";
  }

  // waitfor_ 는 각 로봇이 정확히 1명을 기다리는 함수형 그래프.
  // start 에서 따라가 start 로 돌아오면 사이클(교착). out 에 사이클 멤버 채움.
  bool find_cycle(const std::string & start, std::vector<std::string> & out) const
  {
    out.clear();
    std::string cur = start;
    std::set<std::string> seen;
    for (;;) {
      out.push_back(cur);
      auto it = waitfor_.find(cur);
      if (it == waitfor_.end()) { return false; }        // 막다른 길 → 교착 아님
      cur = it->second;
      if (cur == start) { return true; }                 // start 복귀 → 사이클
      if (!seen.insert(cur).second) { return false; }    // start 무관한 다른 사이클
    }
  }

  int prio_of(const std::string & r) const
  {
    auto it = prio_.find(r);
    return it == prio_.end() ? 0 : it->second;
  }
  // 사이클 내 최저 우선순위 로봇(동률이면 이름이 큰 쪽) = 양보자.
  const std::string & lowest_priority(const std::vector<std::string> & cyc) const
  {
    const std::string * v = &cyc[0];
    for (const auto & r : cyc) {
      if (prio_of(r) < prio_of(*v) || (prio_of(r) == prio_of(*v) && r > *v)) { v = &r; }
    }
    return *v;
  }

  std::map<int, std::string> node_owner_;      // 노드 → 점유 로봇
  std::map<std::string, std::string> waitfor_; // 로봇 → 기다리는 상대
  std::map<std::string, int> prio_;            // 로봇 → 현재 우선순위
};

}  // namespace libi_fleet

PLUGINLIB_EXPORT_CLASS(libi_fleet::ReservationDeadlock, libi_fleet::TrafficBase)
