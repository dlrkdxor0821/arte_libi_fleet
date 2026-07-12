#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <libi_fleet_msgs/srv/submit_task.hpp>
#include <libi_fleet_msgs/srv/set_plugins.hpp>
#include <libi_fleet_msgs/srv/set_robot_mode.hpp>
#include <libi_fleet_msgs/srv/set_battery.hpp>
#include <libi_fleet_msgs/msg/task_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

#include "libi_fleet/fleet_task.hpp"
#include "libi_fleet/navgraph.hpp"
#include "libi_fleet/patrol_cycle.hpp"
#include "libi_fleet/fms_types.hpp"
#include "libi_fleet/dispatcher_base.hpp"
#include "libi_fleet/traffic_base.hpp"

using SubmitTask = libi_fleet_msgs::srv::SubmitTask;
using SetPlugins = libi_fleet_msgs::srv::SetPlugins;
using SetRobotMode = libi_fleet_msgs::srv::SetRobotMode;
using SetBattery = libi_fleet_msgs::srv::SetBattery;
using TaskState = libi_fleet_msgs::msg::TaskState;
using RmfRobotState = rmf_fleet_msgs::msg::RobotState;
using PathRequest = rmf_fleet_msgs::msg::PathRequest;
using RmfLocation = rmf_fleet_msgs::msg::Location;

namespace libi_fleet
{

class FleetNode : public rclcpp::Node
{
public:
  FleetNode()
  : rclcpp::Node("libi_fleet"),
    disp_loader_("libi_fleet", "libi_fleet::DispatcherBase"),
    traf_loader_("libi_fleet", "libi_fleet::TrafficBase")
  {
    navgraph_file_ = declare_parameter<std::string>("navgraph_file", "");
    const std::string disp_name = declare_parameter<std::string>("dispatcher_plugin", "libi_fleet::Auction");
    const std::string traf_name = declare_parameter<std::string>("traffic_plugin", "libi_fleet::ReservationDeadlock");
    const std::string fleet = declare_parameter<std::string>("fleet_name", "libi");
    fleet_name_ = fleet;

    // 배터리 소비 모델(sim 가정값). 완주 가능성 관문: battery% ≥ 소비% + reserve.
    energy_.drain_per_m   = declare_parameter<double>("battery_drain_per_m", 1.0);   // 주행 1m당 %
    energy_.drain_per_act = declare_parameter<double>("battery_drain_per_act", 0.5); // 팔 1동작당 %
    energy_.reserve       = declare_parameter<double>("battery_reserve_pct", 15.0);  // 최소 잔여 %

    // 순회(patrol) 모드: 켜지면 idle 로봇이 patrol_route(외곽 루프)를 무한 순회.
    patrol_ = declare_parameter<bool>("patrol", true);
    // "auto"(기본) → 우/하 우선 규칙으로 순회 루프 생성(그래프 로드 후). 그 외는 수동 정점 목록.
    const std::string route_s = declare_parameter<std::string>("patrol_route", "auto");

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

    // 순회 루프 확정: "auto" 면 우/하 우선 규칙으로 생성, 아니면 수동 정점 목록 파싱.
    if (route_s == "auto") {
      patrol_route_ = right_hand_boundary_cycle(graph_);
      if (patrol_route_.size() < 2) {   // 생성 실패 → 안전 fallback
        RCLCPP_WARN(get_logger(), "patrol_route auto 생성 실패 → 기본 루프 사용");
        patrol_route_ = {0, 1, 2, 3, 7, 6, 5, 4};
      }
    } else {
      std::stringstream ss(route_s); int v; while (ss >> v) { patrol_route_.push_back(v); }
    }
    {
      std::string s; for (int v : patrol_route_) { s += std::to_string(v) + " "; }
      RCLCPP_INFO(get_logger(), "순회 루프(우/하 우선): %s", s.c_str());
    }

    state_sub_ = create_subscription<RmfRobotState>(
      "/robot_state", 10,
      std::bind(&FleetNode::on_robot_state, this, std::placeholders::_1));
    path_pub_ = create_publisher<PathRequest>("/robot_path_requests", rclcpp::QoS(10).reliable());
    task_pub_ = create_publisher<TaskState>("/fms/task_states", 10);
    occ_pub_ = create_publisher<std_msgs::msg::String>("/fms/occupancy", 10);
    route_pub_ = create_publisher<std_msgs::msg::String>("/fms/routes", 10);
    goal_pub_ = create_publisher<std_msgs::msg::String>("/fms/goals", 10);

    srv_ = create_service<SubmitTask>(
      "/fms/submit_task",
      std::bind(&FleetNode::on_submit, this, std::placeholders::_1, std::placeholders::_2));
    plugins_srv_ = create_service<SetPlugins>(
      "/fms/set_plugins",
      std::bind(&FleetNode::on_set_plugins, this, std::placeholders::_1, std::placeholders::_2));
    reload_srv_ = create_service<std_srvs::srv::Trigger>(
      "/fms/reload_navgraph",
      std::bind(&FleetNode::on_reload, this, std::placeholders::_1, std::placeholders::_2));
    mode_srv_ = create_service<SetRobotMode>(
      "/fms/set_robot_mode",
      std::bind(&FleetNode::on_set_mode, this, std::placeholders::_1, std::placeholders::_2));
    battery_srv_ = create_service<SetBattery>(
      "/fms/set_battery",
      std::bind(&FleetNode::on_set_battery, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(std::chrono::milliseconds(150),
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
    // 배터리는 sim(slotcar)의 battery_percent 를 신뢰하지 않고 내부 상태로 관리(기본 100%).
    // 콘솔 UI(/fms/set_battery)로 각 로봇 배터리를 설정 → 완주 관문·우선순위에 반영.
    if (robot_mode_.find(msg->name) == robot_mode_.end()) {
      robot_mode_[msg->name] = patrol_ ? "PATROL" : "IDLE";   // 최초 관측 시 기본 모드
    }
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
    const int arm_actions = req->arm_actions > 0 ? req->arm_actions : 0;   // 팔 동작 횟수
    std::string robot;
    if (!req->robot.empty()) {              // 특정 로봇 강제 배정
      auto it = robots_.find(req->robot);
      if (it == robots_.end()) { res->accepted = false; res->reason = "unknown_robot"; return; }
      if (mode_of(req->robot) == "STOP") { res->accepted = false; res->reason = "robot_stopped"; return; }
      // 순회/기존 task 중이어도 받는다 — 아래 path/battery 통과 후 기존 task 취소하고 강제 배정.
      robot = req->robot;
    } else {                                // dispatcher 가 선택 (STOP·CHARGE 로봇 제외)
      std::vector<RobotInfo> snapshot;
      for (const auto & kv : robots_) {
        const std::string m = mode_of(kv.first);
        if (m == "STOP" || m == "CHARGE") { continue; }
        RobotInfo ri = kv.second;
        if (is_on_patrol(kv.first)) { ri.busy = false; }   // 순회는 중단 가능 → 경매 후보에 포함
        snapshot.push_back(ri);
      }
      robot = dispatcher_->assign(goal, arm_actions, snapshot, graph_, energy_);
    }
    if (robot.empty()) {
      res->accepted = false; res->reason = "no_robot_available"; return;
    }
    auto & r = robots_[robot];
    int start = graph_.nearest(r.x, r.y);
    auto path = graph_.dijkstra(start, goal);
    if (start == goal) { path = {goal, goal}; }   // 최근접 정점이 곧 목표 → 그 노드로 이동 후 완료(auction.cpp 와 일치, no_path 오거절 방지)
    if (path.size() < 2) {
      res->accepted = false; res->reason = "no_path"; return;   // 진짜 도달 불가만 거절
    }
    // 완주 가능성 관문(강제 배정도 포함 — 방전 좌초 방지). 자동배차는 dispatcher 가 이미 필터.
    double need = graph_.path_cost(path) * energy_.drain_per_m
                + arm_actions * energy_.drain_per_act + energy_.reserve;
    if (r.battery < need) {
      res->accepted = false; res->reason = "insufficient_battery"; return;
    }
    if (r.busy) { cancel_task(robot); }   // 순회/기존 task 취소하고 이 배차로 대체 (특정 배차·경매 낙찰 공통)
    r.busy = true;
    // 콘솔이 지정한 커스텀 작업 이름(requester 필드에 실려옴). 비우면 자동 T-N.
    std::string tid = req->requester.empty()
                        ? ("T-" + std::to_string(++task_counter_))
                        : req->requester;
    r.task_id = tid;
    ActiveTask t; t.id = tid; t.robot = robot; t.path = path; t.idx = 1; t.moving = false;
    t.priority = req->priority;
    t.start_seq = ++task_seq_;
    t.arm_actions = arm_actions;
    traffic_->request_move(robot, path[0], path[0], compute_priority(robot, t));   // 시작 노드 점유
    tasks_.push_back(t);
    res->accepted = true; res->task_id = tid; res->reason = "";
    publish_task_state(tid, "ASSIGNED", robot);
    RCLCPP_INFO(get_logger(), "[%s] %s 배차 → goal v%d, path %zu nodes",
                tid.c_str(), robot.c_str(), goal, path.size());
  }

  void on_timer()
  {
    // 순회 모드(per-robot): PATROL 모드 로봇이 task 없으면 외곽 루프 순회 부여
    if (patrol_route_.size() >= 2) {
      for (auto & kv : robots_) {
        RobotInfo & r = kv.second;
        if (r.busy || mode_of(r.name) != "PATROL") { continue; }
        bool has = false;
        for (const auto & t : tasks_) { if (t.robot == r.name) { has = true; break; } }
        if (!has) { start_patrol(r); }
      }
    }

    for (auto it = tasks_.begin(); it != tasks_.end();) {
      ActiveTask & t = *it;
      RobotInfo & r = robots_[t.robot];

      const Vertex & tv = graph_.vertex(t.path[t.idx]);
      double d = std::hypot(r.x - tv.x, r.y - tv.y);

      // ── stuck 감지: 이동 지시(t.moving)됐는데 위치가 안 변하면(슬롯카 벽 끼임 등) →
      //    예약 노드 해제하고 task 취소. wedged 로봇이 노드를 붙잡아 다른 로봇을 막는 걸 방지. ──
      if (t.moving && std::hypot(r.x - t.last_x, r.y - t.last_y) < 0.02) { t.no_move++; }
      else { t.no_move = 0; }
      t.last_x = r.x; t.last_y = r.y;
      if (t.no_move > kStuckTicks) {
        RCLCPP_ERROR(get_logger(), "[%s] %s ⚠ 무진행(슬롯카 stuck 추정) → 예약 해제·task 취소",
                     t.id.c_str(), t.robot.c_str());
        if (t.idx < t.path.size()) { traffic_->release_node(t.robot, t.path[t.idx]); }
        if (t.idx >= 1) { traffic_->release_node(t.robot, t.path[t.idx - 1]); }
        r.busy = false; r.task_id.clear();
        publish_task_state(t.id, "FAILED", t.robot);
        it = tasks_.erase(it); continue;
      }

      if (t.moving && d < kArrive) {
        // 도착: 예약한 목표 노드는 그대로 소유(다음 출발 때 release). 엣지 예약은 없음.
        RCLCPP_INFO(get_logger(), "[%s] %s 도착 v%d", t.id.c_str(), t.robot.c_str(), t.path[t.idx]);
        t.idx++;
        t.moving = false;
        t.reroutes = 0;   // 노드 도달 = 진전 → 우회 카운터 리셋
        t.wait_ticks = 0;   // 노드 도달 = 진전 → 타임드 우회 카운터 리셋
        if (t.idx >= t.path.size()) {
          if (t.patrol) {
            t.path = make_patrol_path(r, -1);   // 현재 위치서 canonical 랩 재생성(방향 유지)
            t.idx = 1; t.moving = false;
            RCLCPP_INFO(get_logger(), "[%s] %s 순회 1바퀴 → 계속", t.id.c_str(), t.robot.c_str());
          } else {
            traffic_->release_node(t.robot, t.path.back());     // 최종 노드 해제
            r.busy = false; r.task_id.clear();
            publish_task_state(t.id, "COMPLETED", t.robot);
            RCLCPP_INFO(get_logger(), "[%s] %s 작업 완료", t.id.c_str(), t.robot.c_str());
            it = tasks_.erase(it);
            continue;
          }
        }
      }

      if (!t.moving) {
        int cur = t.path[t.idx - 1];
        int next = t.path[t.idx];
        MoveDecision dec = traffic_->request_move(t.robot, cur, next, compute_priority(t.robot, t));
        if (dec == MoveDecision::GRANT) {
          if (cur != next) { traffic_->release_node(t.robot, cur); }   // 출발 순간 이전 노드 해제 (cur==next=start==goal 케이스는 목표 유지)
          send_path(t.robot, r.x, r.y, graph_.vertex(next));
          t.moving = true; t.wait_logged = false; t.stuck = false; t.wait_ticks = 0;   // 풀림 → escalation 해제
          RCLCPP_INFO(get_logger(), "[%s] %s → v%d (GRANT)", t.id.c_str(), t.robot.c_str(), next);
        } else if (dec == MoveDecision::DEADLOCK) {
          // 우회는 kMaxReroutes 번까지만(livelock 방지). 초과하면 우회 포기 → escalate + 대기.
          int goal_node = t.patrol ? patrol_succ(next) : t.path.back();   // 순회는 방향 유지(막힌 노드 다음)
          auto reroute = (t.reroutes < kMaxReroutes && goal_node >= 0)
                       ? graph_.dijkstra(cur, goal_node, next) : std::vector<int>{};   // next 를 피해 우회
          if (reroute.size() >= 2) {
            t.reroutes++;
            RCLCPP_WARN(get_logger(), "[%s] %s ⚠ 교착 감지(v%d) → 우회 %zu nodes (재시도 %d/%d)",
                        t.id.c_str(), t.robot.c_str(), next, reroute.size(), t.reroutes, kMaxReroutes);
            t.path = reroute; t.idx = 1; t.moving = false; t.wait_logged = false; t.stuck = false;
          } else {
            if (!t.stuck) {   // 우회 불가 or 우회 반복초과(livelock) → 우선순위 최상위 escalate(주변이 비켜줌) 후 대기
              t.stuck = true;
              RCLCPP_WARN(get_logger(), "[%s] %s ⛔ 완전막힘/우회반복(v%d) → 우선순위 최상위 상향, 대기",
                          t.id.c_str(), t.robot.c_str(), next);
            }
            if (!t.wait_logged) { publish_task_state(t.id, "EXECUTING", t.robot); t.wait_logged = true; }
          }
        } else {   // WAIT
          if (blocked_by_stopped(next)) {   // 정지 로봇(영구 장애물)이 막음 → 우회
            auto reroute = graph_.dijkstra(cur, t.path.back(), next);
            if (reroute.size() >= 2) {
              RCLCPP_WARN(get_logger(), "[%s] %s ⤴ 정지 로봇(v%d) 우회 → %zu nodes",
                          t.id.c_str(), t.robot.c_str(), next, reroute.size());
              t.path = reroute; t.idx = 1; t.moving = false; t.wait_logged = false;
            } else if (!t.wait_logged) {
              RCLCPP_ERROR(get_logger(), "[%s] %s 정지 로봇(v%d) 막힘, 우회 불가",
                           t.id.c_str(), t.robot.c_str(), next);
              t.wait_logged = true;
            }
          } else {
            // 일반 WAIT(움직이는 로봇이 점유). 오래 안 풀리면 타임드 우회(작업·순회 모두).
            t.wait_ticks++;
            if (!t.wait_logged) {
              publish_task_state(t.id, "EXECUTING", t.robot);
              RCLCPP_WARN(get_logger(), "[%s] %s ⏸ v%d 점유중 → 양보 대기", t.id.c_str(), t.robot.c_str(), next);
              t.wait_logged = true;
            }
            if (t.wait_ticks >= kRerouteWaitTicks) {
              int goal_node = t.patrol ? patrol_succ(next) : t.path.back();   // 순회는 방향 유지
              auto reroute = (goal_node >= 0) ? graph_.dijkstra(cur, goal_node, next)
                                              : std::vector<int>{};
              if (reroute.size() >= 2) {
                RCLCPP_WARN(get_logger(), "[%s] %s ⤴ %ds 대기 → 우회 %zu nodes (v%d 회피)",
                            t.id.c_str(), t.robot.c_str(), (kRerouteWaitTicks * 150 + 500) / 1000,
                            reroute.size(), next);
                t.path = reroute; t.idx = 1; t.moving = false; t.wait_logged = false;
                t.wait_ticks = 0;
              } else {
                t.wait_ticks = 0;   // 우회 불가 → 카운터만 리셋, 계속 대기(다음 주기 재시도)
              }
            }
          }
        }
      }
      ++it;
    }
    publish_occupancy();
    publish_routes();
    publish_goals();
  }

  // 각 로봇의 남은 경로(현재 노드→목표)를 JSON 으로 발행(시각화용): {"robot":[[x,y],...]}.
  void publish_routes()
  {
    std::string j = "{";
    bool first = true;
    for (const auto & t : tasks_) {
      if (!first) { j += ","; }
      j += "\"" + t.robot + "\":[";
      size_t start = t.idx > 0 ? t.idx - 1 : 0;   // 현재 향해 출발한 노드부터
      for (size_t i = start; i < t.path.size(); ++i) {
        const Vertex & v = graph_.vertex(t.path[i]);
        if (i > start) { j += ","; }
        j += "[" + std::to_string(v.x) + "," + std::to_string(v.y) + "]";
      }
      j += "]";
      first = false;
    }
    j += "}";
    std_msgs::msg::String m; m.data = j;
    route_pub_->publish(m);
  }

  // 각 로봇의 최종 목적지 발행(배차 task만; 순회는 제외 → 콘솔에서 "—"). {"robot": goalVertex}.
  void publish_goals()
  {
    std::string j = "{";
    bool first = true;
    for (const auto & t : tasks_) {
      if (t.patrol) { continue; }              // 순회는 최종 목적지 없음
      if (!first) { j += ","; }
      j += "\"" + t.robot + "\":" + std::to_string(t.path.back());
      first = false;
    }
    j += "}";
    std_msgs::msg::String m; m.data = j;
    goal_pub_->publish(m);
  }

  // 교통 플러그인의 실제 예약(노드→로봇)을 JSON 으로 발행(시각화용).
  void publish_occupancy()
  {
    std::string j = "{";
    bool first = true;
    for (const auto & no : traffic_->occupancy()) {
      if (!first) { j += ","; }
      j += "\"" + std::to_string(no.first) + "\":\"" + no.second + "\"";
      first = false;
    }
    j += "}";
    std_msgs::msg::String m; m.data = j;
    occ_pub_->publish(m);
  }

  std::string mode_of(const std::string & robot) const
  {
    auto it = robot_mode_.find(robot);
    return it == robot_mode_.end() ? "IDLE" : it->second;
  }

  // node 가 정지(STOP) 로봇에 점유돼 있으면 true → 영구 장애물이므로 우회 대상.
  bool blocked_by_stopped(int node) const
  {
    for (const auto & no : traffic_->occupancy()) {
      if (no.first == node && mode_of(no.second) == "STOP") { return true; }
    }
    return false;
  }

  double battery_of(const std::string & robot) const
  {
    auto it = robots_.find(robot);
    return it == robots_.end() ? 100.0 : it->second.battery;
  }

  // 교통 우선순위(단일 int): tier(가장 큼) > task 나이(오래된=큼) > 배터리(낮은=큼).
  //   tier: 순회=0 · 작업=1 · 충전복귀(CHARGE)=2 · 완전막힘(STUCK,동적)=3
  int compute_priority(const std::string & robot, const ActiveTask & t) const
  {
    int tier;
    if (t.stuck) { tier = 3; }                              // 완전 막힘(escalation)
    else if (mode_of(robot) == "CHARGE") { tier = 2; }      // 충전 복귀
    else if (t.patrol) { tier = 0; }                        // 순회
    else { tier = 1; }                                      // 작업
    int seq = t.start_seq < kSeqMax ? t.start_seq : kSeqMax;
    int age = kSeqMax - seq;                                // 오래된(작은 seq)일수록 큼
    int bi = static_cast<int>(std::lround(battery_of(robot)));
    if (bi < 0) { bi = 0; } else if (bi > 100) { bi = 100; }
    int batt = 100 - bi;                                    // 낮은 배터리일수록 큼
    return tier * kTierStep + age * kAgeStep + batt;
  }

  // 이 로봇의 활성 task 가 순회(patrol)인가 — 순회는 배차로 중단 가능.
  bool is_on_patrol(const std::string & robot) const
  {
    for (const auto & t : tasks_) { if (t.robot == robot) { return t.patrol; } }
    return false;
  }

  // 로봇의 활성 task 취소: 점유(현재+예약 노드) 해제 후 task 제거, busy 해제.
  void cancel_task(const std::string & robot)
  {
    for (auto it = tasks_.begin(); it != tasks_.end();) {
      if (it->robot == robot) {
        if (it->idx >= 1 && it->idx - 1 < it->path.size()) {
          traffic_->release(robot, it->path[it->idx - 1]);
        }
        if (it->idx < it->path.size()) {
          traffic_->release(robot, it->path[it->idx]);
        }
        it = tasks_.erase(it);
      } else {
        ++it;
      }
    }
    auto r = robots_.find(robot);
    if (r != robots_.end()) { r->second.busy = false; r->second.task_id.clear(); }
  }

  // canonical 순회 루프에서 node 의 다음 노드. node 가 루프에 없으면 -1.
  int patrol_succ(int node) const
  {
    const int n = static_cast<int>(patrol_route_.size());
    for (int i = 0; i < n; ++i) {
      if (patrol_route_[i] == node) { return patrol_route_[(i + 1) % n]; }
    }
    return -1;
  }

  // 현재 위치에서 canonical 방향으로 한 바퀴 랩 경로 생성(가장 가까운 정점 진입 → 정방향).
  // avoid_first>=0 이면 진입점의 다음 홉이 그 노드일 때 한 칸 앞에서 시작(방향은 유지).
  std::vector<int> make_patrol_path(const RobotInfo & r, int avoid_first) const
  {
    const size_t n = patrol_route_.size();
    size_t k = 0; double bd = 1e18;   // 가장 가까운 순회 정점 = 진입점
    for (size_t i = 0; i < n; ++i) {
      const Vertex & v = graph_.vertex(patrol_route_[i]);
      double dd = std::hypot(r.x - v.x, r.y - v.y);
      if (dd < bd) { bd = dd; k = i; }
    }
    if (avoid_first >= 0 && patrol_route_[(k + 1) % n] == avoid_first) { k = (k + 1) % n; }
    std::vector<int> path;
    for (size_t i = 0; i < n; ++i) { path.push_back(patrol_route_[(k + i) % n]); }
    path.push_back(patrol_route_[k]);   // 루프 닫기(마지막==처음)
    return path;
  }

  // 로봇을 외곽 루프(patrol_route)에 태워 무한 순회 시작.
  void start_patrol(RobotInfo & r)
  {
    std::vector<int> path = make_patrol_path(r, -1);
    r.busy = true;
    std::string tid = "P-" + r.name;
    r.task_id = tid;
    ActiveTask t; t.id = tid; t.robot = r.name; t.path = path;
    t.idx = 1; t.moving = false; t.patrol = true;   // 순회는 최저 tier
    t.start_seq = ++task_seq_;
    traffic_->request_move(r.name, path[0], path[0], compute_priority(r.name, t));   // 진입점 점유
    tasks_.push_back(t);
    publish_task_state(tid, "PATROL", r.name);
    RCLCPP_INFO(get_logger(), "[%s] %s 순회 시작 (진입 v%d, %zu nodes)",
                tid.c_str(), r.name.c_str(), path[0], path.size());
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

  void on_set_mode(const std::shared_ptr<SetRobotMode::Request> req,
                   std::shared_ptr<SetRobotMode::Response> res)
  {
    const std::string & m = req->mode;
    if (m != "PATROL" && m != "IDLE" && m != "STOP" && m != "CHARGE") {
      res->ok = false; res->reason = "bad_mode"; return;
    }
    // CHARGE 는 sim 용 상태 태그(실제 복귀 X) — 진행 중 task 를 유지한 채 우선순위만 최상위로.
    // 그 외 모드 전환은 현재 task 취소(점유 해제).
    if (m != "CHARGE") {
      cancel_task(req->robot);
      for (const auto & no : traffic_->occupancy()) {   // 남은 점유(이전 정지 claim 등) 해제
        if (no.second == req->robot) { traffic_->release(req->robot, no.first); }
      }
    }
    robot_mode_[req->robot] = m;      // 미관측 로봇도 저장(관측되면 적용)
    if (m == "STOP") {                // 정지 로봇은 현재 노드를 장애물로 점유(다른 로봇이 우회)
      auto it = robots_.find(req->robot);
      if (it != robots_.end()) {
        int node = graph_.nearest(it->second.x, it->second.y);
        traffic_->request_move(req->robot, node, node, kStopPrio);
      }
    }
    res->ok = true; res->reason = "";
    RCLCPP_INFO(get_logger(), "로봇 모드: %s → %s", req->robot.c_str(), m.c_str());
  }

  // sim 테스트용 배터리 설정(각 로봇). 완주 관문·우선순위에 즉시 반영.
  void on_set_battery(const std::shared_ptr<SetBattery::Request> req,
                      std::shared_ptr<SetBattery::Response> res)
  {
    double v = req->value;
    if (v < 0.0) { v = 0.0; } else if (v > 100.0) { v = 100.0; }
    auto & r = robots_[req->robot];   // 미관측 로봇도 생성(관측되면 위치 갱신)
    r.name = req->robot;
    r.battery = v;
    res->ok = true; res->reason = "";
    RCLCPP_INFO(get_logger(), "배터리 설정: %s → %.0f%%", req->robot.c_str(), v);
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
  bool patrol_{false};
  std::vector<int> patrol_route_;
  std::map<std::string, std::string> robot_mode_;   // 로봇 → PATROL|IDLE|STOP|CHARGE
  std::map<std::string, RobotInfo> robots_;
  std::vector<ActiveTask> tasks_;
  EnergyParams energy_;             // 배터리 소비 모델(완주 가능성 관문)
  int task_counter_{0};
  int path_seq_{0};
  int task_seq_{0};                 // 전체 task 생성 순서(우선순위 나이 tiebreak)

  rclcpp::Subscription<RmfRobotState>::SharedPtr state_sub_;
  rclcpp::Publisher<PathRequest>::SharedPtr path_pub_;
  rclcpp::Publisher<TaskState>::SharedPtr task_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr occ_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_pub_;
  rclcpp::Service<SubmitTask>::SharedPtr srv_;
  rclcpp::Service<SetPlugins>::SharedPtr plugins_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp::Service<SetRobotMode>::SharedPtr mode_srv_;
  rclcpp::Service<SetBattery>::SharedPtr battery_srv_;
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
