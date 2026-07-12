#pragma once
#include <vector>
#include "libi_fleet/navgraph.hpp"

namespace libi_fleet
{

// "오른쪽 우선 → 아래 우선"(우수법 경계 순회)으로 그래프 외곽을 도는 canonical 방향 루프를
// 계산한다. 시작 = 최상단(y 최대), 동률 시 최좌측(x 최소). 시작 진행 방향 = +x(오른쪽).
// 각 노드에서 들어온 방향 기준 가장 오른쪽으로 꺾는 이웃을 택한다(막다른 곳에서만 되돌아감).
// 반환: 방문 노드열(시작 노드로 닫지 않음). 사이클 형성 실패 시 빈 벡터.
std::vector<int> right_hand_boundary_cycle(const Navgraph & g);

}  // namespace libi_fleet
