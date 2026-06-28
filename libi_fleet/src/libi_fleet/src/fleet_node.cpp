#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <libi_fleet_msgs/srv/submit_task.hpp>

using SubmitTask = libi_fleet_msgs::srv::SubmitTask;

// Plan 1 스켈레톤: SubmitTask 를 무조건 accept 하는 스텁.
// (Plan 3 에서 TaskManager + Dispatcher pluginlib 로 교체)
class FleetNode : public rclcpp::Node
{
public:
  FleetNode()
  : rclcpp::Node("libi_fleet")
  {
    srv_ = create_service<SubmitTask>(
      "/fms/submit_task",
      [this](const std::shared_ptr<SubmitTask::Request> req,
             std::shared_ptr<SubmitTask::Response> res) {
        res->accepted = true;
        res->task_id = "T-" + std::to_string(++counter_);
        res->reason = "";
        RCLCPP_INFO(
          get_logger(), "SubmitTask stub: type=%s -> %s",
          req->task_type.c_str(), res->task_id.c_str());
      });
    RCLCPP_INFO(get_logger(), "libi_fleet skeleton up (stub /fms/submit_task)");
  }

private:
  rclcpp::Service<SubmitTask>::SharedPtr srv_;
  int counter_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FleetNode>());
  rclcpp::shutdown();
  return 0;
}
