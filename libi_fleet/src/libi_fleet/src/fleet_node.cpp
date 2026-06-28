#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <libi_fleet_msgs/srv/submit_task.hpp>
#include <libi_fleet_msgs/srv/set_plugins.hpp>
#include <libi_fleet_msgs/msg/task_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

#include "libi_fleet/navgraph.hpp"
#include "libi_fleet/fms_types.hpp"
#include "libi_fleet/dispatcher_base.hpp"
#include "libi_fleet/traffic_base.hpp"

using SubmitTask = libi_fleet_msgs::srv::SubmitTask;
using SetPlugins = libi_fleet_msgs::srv::SetPlugins;
using TaskState = libi_fleet_msgs::msg::TaskState;
using RmfRobotState = rmf_fleet_msgs::msg::RobotState;
using PathRequest = rmf_fleet_msgs::msg::PathRequest;
using RmfLocation = rmf_fleet_msgs::msg::Location;

namespace libi_fleet
{

constexpr double kArrive = 0.35;   // 도착 판정 거리(m)

struct ActiveTask
{
  std::string id;
  std::string robot;
  std::vector<int> path;   // 정점 인덱스 경로(시작 포함)
  size_t idx{1};           // 현재 향하는 path 인덱스
  bool moving{false};
  bool wait_logged{false};
};

class FleetNode : public rclcpp::Node
{
public:
  FleetNode()
  : rclcpp::Node("libi_fleet"),
    disp_loader_("libi_fleet", "libi_fleet::DispatcherBase"),
    traf_loader_("libi_fleet", "libi_fleet::TrafficBase")
  {
    navgraph_file_ = declare_parameter<std::string>("navgraph_file", "");
    const std::string disp_name = declare_parameter<std::string>("dispatcher_plugin", "libi_fleet::GreedyCost");
    const std::string traf_name = declare_parameter<std::string>("traffic_plugin", "libi_fleet::EdgeNodeLock");
    const std::string fleet = declare_parameter<std::string>("fleet_name", "libi");
    fleet_name_ = fleet;

    if (!graph_.load(navgraph_file_)) {
      RCLCPP_FATAL(get_logger(), "navgraph 로드 실패: %s", navgraph_file_.c_str());
      throw std::runtime_error("navgraph load failed");
    }
    active_disp_ = disp_name;
    active_traf_ = traf_name;
    dispatcher_ = disp_loader_.createSharedInstance(disp_name);
    traffic_ = traf_loader_.createSharedInstance(traf_name);
    RCLCPP_INFO(get_logger(), "plugins: dispatcher=%s traffic=%s | navgraph=%d verts",
                disp_name.c_str(), traf_name.c_str(), graph_.size());

    state_sub_ = create_subscription<RmfRobotState>(
      "/robot_state", 10,
      std::bind(&FleetNode::on_robot_state, this, std::placeholders::_1));
    path_pub_ = create_publisher<PathRequest>("/robot_path_requests", rclcpp::QoS(10).reliable());
    task_pub_ = create_publisher<TaskState>("/fms/task_states", 10);

    srv_ = create_service<SubmitTask>(
      "/fms/submit_task",
      std::bind(&FleetNode::on_submit, this, std::placeholders::_1, std::placeholders::_2));
    plugins_srv_ = create_service<SetPlugins>(
      "/fms/set_plugins",
      std::bind(&FleetNode::on_set_plugins, this, std::placeholders::_1, std::placeholders::_2));
    reload_srv_ = create_service<std_srvs::srv::Trigger>(
      "/fms/reload_navgraph",
      std::bind(&FleetNode::on_reload, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(std::chrono::milliseconds(250),
                               std::bind(&FleetNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "libi_fleet FMS up");
  }

private:
  void on_robot_state(const RmfRobotState::SharedPtr msg)
  {
    auto & r = robots_[msg->name];
    r.name = msg->name;
    r.x = msg->location.x;
    r.y = msg->location.y;
  }

  void publish_task_state(const std::string & id, const std::string & state, const std::string & robot)
  {
    TaskState ts;
    ts.task_id = id;
    ts.state = state;
    ts.robot_id = robot;
    task_pub_->publish(ts);
  }

  void send_path(const std::string & robot, double x0, double y0, const Vertex & target)
  {
    PathRequest req;
    req.fleet_name = fleet_name_;
    req.robot_name = robot;
    req.task_id = robot + "-" + std::to_string(++path_seq_);   // 고유 task_id (slotcar dedup 회피)
    RmfLocation p0; p0.x = x0; p0.y = y0; p0.level_name = "L1";
    RmfLocation p1; p1.x = target.x; p1.y = target.y; p1.level_name = "L1";
    req.path = {p0, p1};
    path_pub_->publish(req);
  }

  void on_submit(const std::shared_ptr<SubmitTask::Request> req,
                 std::shared_ptr<SubmitTask::Response> res)
  {
    int goal = -1;
    try { goal = std::stoi(req->dropoff); } catch (...) { goal = -1; }
    if (goal < 0 || goal >= graph_.size()) {
      res->accepted = false; res->reason = "bad_goal_vertex"; return;
    }
    std::string robot;
    if (!req->robot.empty()) {              // 특정 로봇 강제 배정
      auto it = robots_.find(req->robot);
      if (it == robots_.end()) { res->accepted = false; res->reason = "unknown_robot"; return; }
      if (it->second.busy) { res->accepted = false; res->reason = "robot_busy"; return; }
      robot = req->robot;
    } else {                                // dispatcher 가 선택
      std::vector<RobotInfo> snapshot;
      for (const auto & kv : robots_) { snapshot.push_back(kv.second); }
      robot = dispatcher_->assign(goal, snapshot, graph_);
    }
    if (robot.empty()) {
      res->accepted = false; res->reason = "no_robot_available"; return;
    }
    auto & r = robots_[robot];
    int start = graph_.nearest(r.x, r.y);
    auto path = graph_.dijkstra(start, goal);
    if (path.size() < 2) {
      res->accepted = false; res->reason = "no_path"; return;
    }
    r.busy = true;
    std::string tid = "T-" + std::to_string(++task_counter_);
    r.task_id = tid;
    ActiveTask t; t.id = tid; t.robot = robot; t.path = path; t.idx = 1; t.moving = false;
    traffic_->request_move(robot, path[0]);   // 시작 노드 점유
    tasks_.push_back(t);
    res->accepted = true; res->task_id = tid; res->reason = "";
    publish_task_state(tid, "ASSIGNED", robot);
    RCLCPP_INFO(get_logger(), "[%s] %s 배차 → goal v%d, path %zu nodes",
                tid.c_str(), robot.c_str(), goal, path.size());
  }

  void on_timer()
  {
    for (auto it = tasks_.begin(); it != tasks_.end();) {
      ActiveTask & t = *it;
      RobotInfo & r = robots_[t.robot];
      const Vertex & tv = graph_.vertex(t.path[t.idx]);
      double d = std::hypot(r.x - tv.x, r.y - tv.y);

      if (t.moving && d < kArrive) {
        traffic_->release(t.robot, t.path[t.idx - 1]);   // 직전 노드 해제
        RCLCPP_INFO(get_logger(), "[%s] %s 도착 v%d", t.id.c_str(), t.robot.c_str(), t.path[t.idx]);
        t.idx++;
        t.moving = false;
        if (t.idx >= t.path.size()) {
          traffic_->release(t.robot, t.path.back());     // 최종 노드 해제
          r.busy = false; r.task_id.clear();
          publish_task_state(t.id, "COMPLETED", t.robot);
          RCLCPP_INFO(get_logger(), "[%s] %s 작업 완료", t.id.c_str(), t.robot.c_str());
          it = tasks_.erase(it);
          continue;
        }
      }

      if (!t.moving) {
        int next = t.path[t.idx];
        if (traffic_->request_move(t.robot, next) == MoveDecision::GRANT) {
          send_path(t.robot, r.x, r.y, graph_.vertex(next));
          t.moving = true; t.wait_logged = false;
          RCLCPP_INFO(get_logger(), "[%s] %s → v%d (GRANT)", t.id.c_str(), t.robot.c_str(), next);
        } else if (!t.wait_logged) {
          publish_task_state(t.id, "EXECUTING", t.robot);
          RCLCPP_WARN(get_logger(), "[%s] %s ⏸ v%d 점유중 → 양보 대기", t.id.c_str(), t.robot.c_str(), next);
          t.wait_logged = true;
        }
      }
      ++it;
    }
  }

  void on_set_plugins(const std::shared_ptr<SetPlugins::Request> req,
                      std::shared_ptr<SetPlugins::Response> res)
  {
    try {
      if (!req->dispatcher.empty()) {
        dispatcher_ = disp_loader_.createSharedInstance(req->dispatcher);
        active_disp_ = req->dispatcher;
      }
      if (!req->traffic.empty()) {
        traffic_ = traf_loader_.createSharedInstance(req->traffic);  // 잠금상태 초기화(테스트는 idle 시 스왑)
        active_traf_ = req->traffic;
      }
      res->ok = true;
    } catch (const std::exception & e) {
      res->ok = false; res->reason = e.what();
    }
    res->active_dispatcher = active_disp_;
    res->active_traffic = active_traf_;
    RCLCPP_INFO(get_logger(), "set_plugins → dispatcher=%s traffic=%s (ok=%d)",
                active_disp_.c_str(), active_traf_.c_str(), res->ok ? 1 : 0);
  }

  void on_reload(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    Navgraph g;
    if (g.load(navgraph_file_)) {
      graph_ = g;
      res->success = true;
      res->message = "reloaded " + std::to_string(graph_.size()) + " vertices";
      RCLCPP_INFO(get_logger(), "navgraph 리로드: %d 정점", graph_.size());
    } else {
      res->success = false;
      res->message = "load failed";
    }
  }

  // plugins
  pluginlib::ClassLoader<DispatcherBase> disp_loader_;
  pluginlib::ClassLoader<TrafficBase> traf_loader_;
  std::shared_ptr<DispatcherBase> dispatcher_;
  std::shared_ptr<TrafficBase> traffic_;
  std::string active_disp_;
  std::string active_traf_;

  Navgraph graph_;
  std::string navgraph_file_;
  std::string fleet_name_;
  std::map<std::string, RobotInfo> robots_;
  std::vector<ActiveTask> tasks_;
  int task_counter_{0};
  int path_seq_{0};

  rclcpp::Subscription<RmfRobotState>::SharedPtr state_sub_;
  rclcpp::Publisher<PathRequest>::SharedPtr path_pub_;
  rclcpp::Publisher<TaskState>::SharedPtr task_pub_;
  rclcpp::Service<SubmitTask>::SharedPtr srv_;
  rclcpp::Service<SetPlugins>::SharedPtr plugins_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace libi_fleet

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<libi_fleet::FleetNode>());
  rclcpp::shutdown();
  return 0;
}
