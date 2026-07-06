// Auction / SSI 배차 — 유휴 로봇들이 goal 까지 Dijkstra 경로비용으로 입찰, 최저가 낙찰.
// ⚠️ 단일 task 모델에선 "최저 경로비용 1대 선택"으로 귀결(수학적으로 argmin).
//    대기 task 큐(batch) 를 얹으면 순차 재입찰(SSI)로 확장된다 — 그때 Greedy 와 갈라진다.
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "libi_fleet/dispatcher_base.hpp"

namespace libi_fleet
{

class Auction : public DispatcherBase
{
public:
  std::string assign(int goal, const std::vector<RobotInfo> & robots, const Navgraph & g) override
  {
    std::string winner;
    double best = std::numeric_limits<double>::max();
    for (const auto & r : robots) {
      if (r.busy) { continue; }                  // 유휴 로봇만 입찰
      int start = g.nearest(r.x, r.y);
      auto path = g.dijkstra(start, goal);
      if (start != goal && path.size() < 2) { continue; }   // 도달 불가 → 입찰 포기
      double bid = path_cost(path, g);           // 입찰가 = 실제 경로비용(유클리드 아님)
      if (bid < best) { best = bid; winner = r.name; }
    }
    return winner;   // 없으면 "" (거절)
  }

private:
  static double path_cost(const std::vector<int> & path, const Navgraph & g)
  {
    double c = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      const Vertex & a = g.vertex(path[i - 1]);
      const Vertex & b = g.vertex(path[i]);
      c += std::hypot(a.x - b.x, a.y - b.y);
    }
    return c;
  }
};

}  // namespace libi_fleet

PLUGINLIB_EXPORT_CLASS(libi_fleet::Auction, libi_fleet::DispatcherBase)
