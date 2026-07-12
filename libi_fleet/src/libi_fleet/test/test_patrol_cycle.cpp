#include <gtest/gtest.h>
#include <set>
#include <string>
#include "libi_fleet/navgraph.hpp"
#include "libi_fleet/patrol_cycle.hpp"

using libi_fleet::Navgraph;
using libi_fleet::right_hand_boundary_cycle;

// 실제 지도 yaml 경로는 CMake 에서 TEST_NAVGRAPH_PATH 로 주입.
static std::string kNavgraph() { return std::string(TEST_NAVGRAPH_PATH); }

TEST(PatrolCycle, MatchesRightDownBoundaryOnLibraryMap) {
  Navgraph g;
  ASSERT_TRUE(g.load(kNavgraph(), "L1"));
  auto cyc = right_hand_boundary_cycle(g);
  std::vector<int> expected = {0, 1, 2, 3, 7, 6, 5, 4};
  EXPECT_EQ(cyc, expected);
}

TEST(PatrolCycle, VisitsAllBoundaryNodesOnce) {
  Navgraph g;
  ASSERT_TRUE(g.load(kNavgraph(), "L1"));
  auto cyc = right_hand_boundary_cycle(g);
  std::set<int> uniq(cyc.begin(), cyc.end());
  EXPECT_EQ(uniq.size(), cyc.size());          // 중복 없음
  EXPECT_EQ(cyc.front(), 0);                   // 최상단·최좌측에서 시작
}
