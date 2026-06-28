#include "libi_fleet/navgraph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include <yaml-cpp/yaml.h>

namespace libi_fleet
{

bool Navgraph::load(const std::string & path, const std::string & level)
{
  YAML::Node root = YAML::LoadFile(path);
  YAML::Node lvl = root["levels"][level];
  if (!lvl) {
    return false;
  }
  vertices_.clear();
  for (const auto & v : lvl["vertices"]) {
    vertices_.push_back(Vertex{v[0].as<double>(), v[1].as<double>()});
  }
  adj_.assign(vertices_.size(), {});
  for (const auto & ln : lvl["lanes"]) {
    int a = ln[0].as<int>();
    int b = ln[1].as<int>();
    if (a >= 0 && a < size() && b >= 0 && b < size()) {
      adj_[a].push_back(b);   // navgraph 는 양방향을 둘 다 나열하므로 그대로 단방향 추가
    }
  }
  return !vertices_.empty();
}

int Navgraph::nearest(double x, double y) const
{
  int best = -1;
  double best_d = std::numeric_limits<double>::max();
  for (int i = 0; i < size(); ++i) {
    double d = std::hypot(vertices_[i].x - x, vertices_[i].y - y);
    if (d < best_d) { best_d = d; best = i; }
  }
  return best;
}

std::vector<int> Navgraph::dijkstra(int from, int to) const
{
  const int n = size();
  if (from < 0 || to < 0 || from >= n || to >= n) {
    return {};
  }
  std::vector<double> dist(n, std::numeric_limits<double>::max());
  std::vector<int> prev(n, -1);
  using QE = std::pair<double, int>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  dist[from] = 0.0;
  pq.push({0.0, from});
  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) { continue; }
    if (u == to) { break; }
    for (int v : adj_[u]) {
      double w = std::hypot(vertices_[u].x - vertices_[v].x, vertices_[u].y - vertices_[v].y);
      if (dist[u] + w < dist[v]) {
        dist[v] = dist[u] + w;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }
  if (dist[to] == std::numeric_limits<double>::max()) {
    return {};
  }
  std::vector<int> path;
  for (int cur = to; cur != -1; cur = prev[cur]) {
    path.push_back(cur);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace libi_fleet
