#include <map>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "libi_fleet/traffic_base.hpp"

namespace libi_fleet
{

// Edge/Node Lock (§3): navgraph 노드를 mutex 로 잠가 동시 진입 차단.
// 노드가 비었거나 본인 소유면 GRANT(+점유), 타 로봇 소유면 WAIT.
class EdgeNodeLock : public TrafficBase
{
public:
  MoveDecision request_move(const std::string & robot, int target_node) override
  {
    auto it = owner_.find(target_node);
    if (it == owner_.end() || it->second == robot) {
      owner_[target_node] = robot;
      return MoveDecision::GRANT;
    }
    return MoveDecision::WAIT;
  }

  void release(const std::string & robot, int node) override
  {
    auto it = owner_.find(node);
    if (it != owner_.end() && it->second == robot) {
      owner_.erase(it);
    }
  }

private:
  std::map<int, std::string> owner_;   // node -> 점유 robot
};

}  // namespace libi_fleet

PLUGINLIB_EXPORT_CLASS(libi_fleet::EdgeNodeLock, libi_fleet::TrafficBase)
