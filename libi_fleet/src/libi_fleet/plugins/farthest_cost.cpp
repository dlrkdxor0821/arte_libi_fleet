#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "libi_fleet/dispatcher_base.hpp"

namespace libi_fleet
{

// 교체실험용 대안 배차: GreedyCost 의 반대 — goal 에서 가장 "먼" 가용 로봇 선택.
// (알고리즘 교체가 config 한 줄로 동작함을 보이기 위한 대조군)
class FarthestCost : public DispatcherBase
{
public:
  std::string assign(
    int goal_vertex,
    const std::vector<RobotInfo> & robots,
    const Navgraph & graph) override
  {
    const Vertex & g = graph.vertex(goal_vertex);
    std::string best;
    double worst_cost = -1.0;
    for (const auto & r : robots) {
      if (r.busy) { continue; }
      double cost = std::hypot(r.x - g.x, r.y - g.y);
      if (cost > worst_cost) { worst_cost = cost; best = r.name; }
    }
    return best;
  }
};

}  // namespace libi_fleet

PLUGINLIB_EXPORT_CLASS(libi_fleet::FarthestCost, libi_fleet::DispatcherBase)
