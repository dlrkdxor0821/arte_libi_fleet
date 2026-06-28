#pragma once
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
  std::vector<int> dijkstra(int from, int to) const;

private:
  std::vector<Vertex> vertices_;
  std::vector<std::vector<int>> adj_;   // 인접 리스트(정점 인덱스)
};

}  // namespace libi_fleet
