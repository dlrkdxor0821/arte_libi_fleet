#pragma once
#include <string>

namespace libi_fleet
{

// fleet_adapter 가 보는 로봇 1대의 상태(위치는 /robot_state 로 갱신, busy 는 FMS 가 관리).
struct RobotInfo
{
  std::string name;
  double x{0.0};
  double y{0.0};
  double battery{100.0};   // 배터리 잔량(%). /robot_state 의 battery_percent 로 갱신(관측 전 100).
  bool busy{false};
  std::string task_id;
};

}  // namespace libi_fleet
