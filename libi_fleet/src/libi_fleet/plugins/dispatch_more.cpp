// 배차 알고리즘 추가분 (옵시디언 'ROS2/FMS/배차 알고리즘 - task_dispatcher.md').
// ⚠️ 이들은 *여러 대기 task 를 동시에* 최적화하는 batch 계열이라, 현재 무큐·단일 task 모델에선
//    모두 cost 최소(= GreedyCost)로 귀결한다. 진짜 차이는 task 큐(batch) 추가 시 발현.
//    지금은 폴더 배치 + config 교체 테스트용으로 등록만 해 둔다(베이스라인 = cost 최소).
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "libi_fleet/dispatcher_base.hpp"

namespace libi_fleet
{

static std::string pick_min_cost(int goal, const std::vector<RobotInfo> & robots, const Navgraph & g)
{
  const Vertex & v = g.vertex(goal);
  std::string best;
  double bc = std::numeric_limits<double>::max();
  for (const auto & r : robots) {
    if (r.busy) { continue; }
    double c = std::hypot(r.x - v.x, r.y - v.y);
    if (c < bc) { bc = c; best = r.name; }
  }
  return best;
}

// Hungarian (Kuhn-Munkres) — batch 1:1 최적 매칭 (노트 §2). [batch 필요]
class Hungarian : public DispatcherBase
{
public:
  std::string assign(int g, const std::vector<RobotInfo> & r, const Navgraph & n) override
  { return pick_min_cost(g, r, n); }
};
// Auction / SSI — 경매 최저 입찰 (§3). [batch 필요]
class Auction : public DispatcherBase
{
public:
  std::string assign(int g, const std::vector<RobotInfo> & r, const Navgraph & n) override
  { return pick_min_cost(g, r, n); }
};
// CBBA — 분산 묶음 합의 (§4). [batch 필요]
class Cbba : public DispatcherBase
{
public:
  std::string assign(int g, const std::vector<RobotInfo> & r, const Navgraph & n) override
  { return pick_min_cost(g, r, n); }
};
// MILP / VRP — 수리최적화 (§5). [batch 필요]
class Milp : public DispatcherBase
{
public:
  std::string assign(int g, const std::vector<RobotInfo> & r, const Navgraph & n) override
  { return pick_min_cost(g, r, n); }
};
// GA / ACO — 메타휴리스틱 (§6). [batch 필요]
class GaAco : public DispatcherBase
{
public:
  std::string assign(int g, const std::vector<RobotInfo> & r, const Navgraph & n) override
  { return pick_min_cost(g, r, n); }
};

}  // namespace libi_fleet

PLUGINLIB_EXPORT_CLASS(libi_fleet::Hungarian, libi_fleet::DispatcherBase)
PLUGINLIB_EXPORT_CLASS(libi_fleet::Auction, libi_fleet::DispatcherBase)
PLUGINLIB_EXPORT_CLASS(libi_fleet::Cbba, libi_fleet::DispatcherBase)
PLUGINLIB_EXPORT_CLASS(libi_fleet::Milp, libi_fleet::DispatcherBase)
PLUGINLIB_EXPORT_CLASS(libi_fleet::GaAco, libi_fleet::DispatcherBase)
