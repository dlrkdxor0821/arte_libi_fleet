#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "libi_fleet/traffic_base.hpp"

namespace libi_fleet
{

// 교체실험용 대안 교통: 교통제어 없음 — 항상 GRANT(양보 안 함).
// EdgeNodeLock 으로 갈아끼우면 양보가 생기고, NoLock 이면 사라짐을 대조.
class NoLock : public TrafficBase
{
public:
  MoveDecision request_move(const std::string &, int) override
  {
    return MoveDecision::GRANT;
  }
  void release(const std::string &, int) override {}
};

}  // namespace libi_fleet

PLUGINLIB_EXPORT_CLASS(libi_fleet::NoLock, libi_fleet::TrafficBase)
