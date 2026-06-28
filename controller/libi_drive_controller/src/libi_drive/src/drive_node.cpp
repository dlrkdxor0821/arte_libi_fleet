#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <libi_fleet_msgs/action/navigate.hpp>
#include <libi_fleet_msgs/action/perform_action.hpp>

using Navigate = libi_fleet_msgs::action::Navigate;
using PerformAction = libi_fleet_msgs::action::PerformAction;
using GoalNav = rclcpp_action::ServerGoalHandle<Navigate>;
using GoalAct = rclcpp_action::ServerGoalHandle<PerformAction>;

// Plan 1 스켈레톤: Navigate / PerformAction 을 즉시 succeed 하는 스텁.
// (Plan 2~3 에서 slotcar PathRequest 주행 / 실제 팔 트리거로 교체)
class DriveNode : public rclcpp::Node
{
public:
  DriveNode()
  : rclcpp::Node("libi_drive")
  {
    nav_ = rclcpp_action::create_server<Navigate>(
      this, "/robot1/navigate",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalNav>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalNav> gh) {
        auto res = std::make_shared<Navigate::Result>();
        res->arrived = true;
        res->error = "";
        gh->succeed(res);
        RCLCPP_INFO(get_logger(), "navigate stub -> arrived");
      });

    act_ = rclcpp_action::create_server<PerformAction>(
      this, "/robot1/perform_action",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const PerformAction::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalAct>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalAct> gh) {
        auto res = std::make_shared<PerformAction::Result>();
        res->success = true;
        res->error = "";
        gh->succeed(res);
        RCLCPP_INFO(get_logger(), "perform_action stub -> success");
      });

    RCLCPP_INFO(get_logger(), "libi_drive skeleton up (stub navigate/perform_action)");
  }

private:
  rclcpp_action::Server<Navigate>::SharedPtr nav_;
  rclcpp_action::Server<PerformAction>::SharedPtr act_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveNode>());
  rclcpp::shutdown();
  return 0;
}
