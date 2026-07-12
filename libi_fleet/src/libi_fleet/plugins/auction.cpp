// Auction / SSI 배차 — 유휴 로봇들이 goal 까지 Dijkstra 경로비용으로 입찰, 최저가 낙찰.
// ⚠️ 단일 task 모델에선 "최저 경로비용 1대 선택"으로 귀결(수학적으로 argmin).
//    대기 task 큐(batch) 를 얹으면 순차 재입찰(SSI)로 확장된다 — 그때 Greedy 와 갈라진다.
//
// 배터리는 입찰가 가중치가 아니라 "완주 가능성 관문(feasibility gate)"으로 다룬다(실무/RMF 표준).
//   소비량% = 주행거리_m × drain_per_m + 팔동작수 × drain_per_act
//   완주가능 = battery% ≥ 소비량% + reserve   ← 못 넘으면 입찰 제외
//   입찰가  = 주행거리_m                       ← 통과한 로봇끼리 최단거리 낙찰(배터리는 순위에 X)
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
  std::string assign(int goal, int arm_actions, const std::vector<RobotInfo> & robots,
                     const Navgraph & g, const EnergyParams & e) override
  {
    std::string winner;
    double best = std::numeric_limits<double>::max();
    for (const auto & r : robots) {
      if (r.busy) { continue; }                  // 유휴 로봇만 입찰
      int start = g.nearest(r.x, r.y);
      auto path = g.dijkstra(start, goal);
      if (start != goal && path.size() < 2) { continue; }   // 도달 불가 → 입찰 포기
      double dist = g.path_cost(path);           // 실제 주행거리(m)

      // 완주 가능성 관문(배터리로만): 작업 끝내고도 reserve 위에 남는가?
      double need = dist * e.drain_per_m + arm_actions * e.drain_per_act + e.reserve;
      if (r.battery < need) { continue; }        // 완주 불가 → 입찰 제외

      double bid = dist;                         // 입찰가 = 주행거리(배터리는 순위에 X)
      if (bid < best) { best = bid; winner = r.name; }
    }
    return winner;   // 없으면 "" (거절)
  }
};

}  // namespace libi_fleet

PLUGINLIB_EXPORT_CLASS(libi_fleet::Auction, libi_fleet::DispatcherBase)
