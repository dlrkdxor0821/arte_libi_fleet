#include "libi_fleet/patrol_cycle.hpp"

#include <cmath>
#include <limits>

namespace libi_fleet
{

namespace
{
// a 를 [0, 2π) 로 정규화.
double norm2pi(double a)
{
  const double tau = 2.0 * M_PI;
  while (a < 0) { a += tau; }
  while (a >= tau) { a -= tau; }
  return a;
}
}  // namespace

std::vector<int> right_hand_boundary_cycle(const Navgraph & g)
{
  const int n = g.size();
  if (n < 3) { return {}; }

  // 시작 노드: y 최대, 동률 시 x 최소.
  int start = 0;
  for (int i = 1; i < n; ++i) {
    const auto & v = g.vertex(i);
    const auto & b = g.vertex(start);
    if (v.y > b.y + 1e-9 || (std::abs(v.y - b.y) <= 1e-9 && v.x < b.x - 1e-9)) {
      start = i;
    }
  }

  std::vector<int> cycle;
  cycle.push_back(start);
  int cur = start;
  // 가상 이전 노드: 시작 진행 방향을 +x 로 두려면, 이전은 시작의 서쪽에 있는 것처럼 취급.
  // 즉 "들어온 방향(back)" = 서쪽(π). 첫 스텝에서 오른쪽 꺾기 우선이 +x 를 고른다.
  double back_angle = M_PI;   // cur 에서 이전 노드를 바라보는 각
  int prev = -1;

  const int max_steps = 4 * n + 4;   // 비정상 그래프 안전장치
  for (int step = 0; step < max_steps; ++step) {
    const auto & c = g.vertex(cur);
    const auto & nb = g.neighbors(cur);

    // back_angle 에서 시계방향으로 가장 먼저 만나는 이웃(=우수법 우회전). prev 는 제외하되
    // 다른 후보가 하나도 없으면(막다른 곳) prev 로 되돌아간다.
    int best = -1;
    double best_rel = std::numeric_limits<double>::infinity();
    int fallback_prev = -1;
    for (int nx : nb) {
      const auto & vn = g.vertex(nx);
      double a = std::atan2(vn.y - c.y, vn.x - c.x);
      double rel = norm2pi(back_angle - a);   // 시계방향 상대각(작을수록 더 오른쪽)
      if (rel < 1e-9) { rel += 2.0 * M_PI; }   // 되돌아감(rel≈0)은 최후순위로
      if (nx == prev) { fallback_prev = nx; continue; }
      if (rel < best_rel) { best_rel = rel; best = nx; }
    }
    if (best < 0) { best = fallback_prev; }   // 막다른 곳
    if (best < 0) { return {}; }              // 고립 노드

    if (best == start) { break; }             // 시작으로 복귀 → 사이클 완성
    cycle.push_back(best);

    // 다음 스텝의 back_angle = best 에서 cur 을 바라보는 각.
    const auto & vb = g.vertex(best);
    back_angle = std::atan2(c.y - vb.y, c.x - vb.x);
    prev = cur;
    cur = best;
  }

  if (cycle.size() < 3) { return {}; }
  return cycle;
}

}  // namespace libi_fleet
