#pragma once
#include <cmath>
#include <string>
#include <vector>

namespace libi_fleet
{

struct Vertex { double x; double y; };

// navgraph(yaml) 로드 + 최근접 정점 + Dijkstra 최단경로.
class Navgraph
{
public:
  bool load(const std::string & path, const std::string & level = "L1");
  int size() const { return static_cast<int>(vertices_.size()); }
  const Vertex & vertex(int i) const { return vertices_.at(i); }
  int nearest(double x, double y) const;
  // from→to 정점 인덱스 경로(시작·끝 포함). 경로 없으면 빈 벡터.
  // avoid>=0 이면 그 정점을 통과하지 않는 경로(데드락 우회용).
  std::vector<int> dijkstra(int from, int to, int avoid = -1) const;
  // 정점 i 의 인접 정점 인덱스 목록(경계 순회 등 그래프 순회용).
  const std::vector<int> & neighbors(int i) const { return adj_.at(i); }
  // 정점 인덱스 경로의 실제 주행거리(m). 시작·끝 포함 경로.
  double path_cost(const std::vector<int> & path) const
  {
    double c = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      const Vertex & a = vertices_.at(path[i - 1]);
      const Vertex & b = vertices_.at(path[i]);
      c += std::hypot(a.x - b.x, a.y - b.y);
    }
    return c;
  }

private:
  std::vector<Vertex> vertices_;
  std::vector<std::vector<int>> adj_;   // 인접 리스트(정점 인덱스)
};

}  // namespace libi_fleet
