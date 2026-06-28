#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "libi_fleet/dispatcher_base.hpp"

namespace libi_fleet
{

// Greedy + Cost Function (IA). 가용 로봇 중 goal 까지 cost 최소인 로봇 선택.
// cost = goal 까지 직선거리 (battery/task/patrol/congestion 항은 추후 확장).
class GreedyCost : public DispatcherBase
{
public:
  std::string assign(
    int goal_vertex,
    const std::vector<RobotInfo> & robots,
    const Navgraph & graph) override
  {
    const Vertex & g = graph.vertex(goal_vertex);
    std::string best;
    double best_cost = std::numeric_limits<double>::max();
    for (const auto & r : robots) {
      if (r.busy) { continue; }
      double cost = std::hypot(r.x - g.x, r.y - g.y);
      if (cost < best_cost) { best_cost = cost; best = r.name; }
    }
    return best;  // 가용 로봇 없으면 ""
  }
};

}  // namespace libi_fleet

PLUGINLIB_EXPORT_CLASS(libi_fleet::GreedyCost, libi_fleet::DispatcherBase)
