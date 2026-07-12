"""libi 알고리즘 테스트 콘솔 (FastAPI).

aba_service 와 별도 포트(8001)로 띄우는 개발용 관제 콘솔. rmf-web 대시보드(Map/Robots/Tasks)
패턴을 따른 디버깅 콘솔 — ROS 브리지로 FMS/슬롯카와 통신:
- 라이브 맵(navgraph + 로봇) · 로봇 텔레메트리(mode/배터리/task) · 태스크 피드
- 정점 직접 이동(PathRequest, FMS 우회) / 특정 로봇 task / dispatcher task
- 배차·교통 알고리즘 런타임 스왑(/fms/set_plugins) → 교통 양보 on/off 대조

실행: scripts/run_console.sh  (uvicorn :8001)
"""
import io
import json
import os
import threading
import time

import yaml
from fastapi import FastAPI, Response
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rmf_fleet_msgs.msg import RobotState, PathRequest, Location
from libi_fleet_msgs.srv import SubmitTask, SetPlugins, SetRobotMode, SetBattery
from libi_fleet_msgs.msg import TaskState
from std_msgs.msg import String
from std_srvs.srv import Trigger

NAVGRAPH = os.environ.get(
    "LIBI_NAVGRAPH",
    "/home/asd/personal_repo/arte_libi_fleet/libi_fleet/maps/library/new_map.navgraph.yaml")

# 배차(Auction) 배터리 게이트 파라미터 파일. UI에서 저장 → fleet 재시작 시 --params-file 로 로드.
ALGO_PARAMS = os.environ.get(
    "LIBI_ALGO_PARAMS",
    os.path.normpath(os.path.join(os.path.dirname(__file__),
                                  "../../../libi_fleet/config/algo_params.yaml")))
# UI 노출 파라미터: key → (라벨, 기본값, 최소, 최대)
ALGO_SPEC = {
    "battery_drain_per_m":   ("주행 소비 %/m",    1.0, 0.0, 100.0),
    "battery_drain_per_act": ("팔동작 소비 %/회", 0.5, 0.0, 100.0),
    "battery_reserve_pct":   ("완주 예비 %",      15.0, 0.0, 100.0),
}

ROBOT_MODE = {0: "IDLE", 1: "CHARGING", 2: "MOVING", 3: "PAUSED", 4: "WAITING",
              5: "EMERGENCY", 6: "HOME", 7: "DOCK", 8: "ERROR", 9: "CLEAN"}


def _spin_future(node, fut, timeout=5.0):
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.05)
    return fut.result() if fut.done() else None


class Bridge(Node):
    def __init__(self):
        super().__init__("console_bridge")
        self.robots = {}
        self.paths = {}       # name -> [[x,y],...]  로봇이 지금 향하는 경로 구간
        self.occupancy = {}   # "node" -> robot  (교통 플러그인 실제 예약)
        self.routes = {}      # robot -> [[x,y],...]  목표까지 남은 경로(FMS 발행)
        self.goals = {}       # robot -> 최종 목적지 정점 (배차 task만; 순회는 없음)
        self.battery_override = {}   # name -> 표시용 배터리 오버라이드
        self.fleet_mode = {}         # name -> 콘솔이 설정한 fleet 모드(PATROL/IDLE/STOP/CHARGE)
        self.task_log = []
        self.ng_full = yaml.safe_load(open(NAVGRAPH))
        ng = self.ng_full["levels"]["L1"]
        self.vertices = [[float(v[0]), float(v[1])] for v in ng["vertices"]]
        self.lanes = [[int(l[0]), int(l[1])] for l in ng["lanes"]]
        self.active = {"dispatcher": "?", "traffic": "?"}
        self.map_png = b""
        self.map_meta = None
        self._load_map()
        self.create_subscription(RobotState, "/robot_state", self._on_state, 10)
        self.create_subscription(TaskState, "/fms/task_states", self._on_task, 10)
        self.create_subscription(PathRequest, "/robot_path_requests", self._on_path, 10)
        self.create_subscription(String, "/fms/occupancy", self._on_occ, 10)
        self.create_subscription(String, "/fms/routes", self._on_routes, 10)
        self.create_subscription(String, "/fms/goals", self._on_goals, 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.path_pub = self.create_publisher(PathRequest, "/robot_path_requests", qos)
        self.task_cli = self.create_client(SubmitTask, "/fms/submit_task")
        self.plugins_cli = self.create_client(SetPlugins, "/fms/set_plugins")
        self.reload_cli = self.create_client(Trigger, "/fms/reload_navgraph")
        self.mode_cli = self.create_client(SetRobotMode, "/fms/set_robot_mode")
        self.battery_cli = self.create_client(SetBattery, "/fms/set_battery")
        self.seq = 0

    def _load_map(self):
        """점유격자(pgm)를 png(반전: 벽=밝게)로 변환해 콘솔 배경에 깔 수 있게 준비."""
        try:
            from PIL import Image, ImageOps
            mdir = os.path.dirname(NAVGRAPH)
            meta = yaml.safe_load(open(os.path.join(mdir, "new_map.yaml")))
            img = Image.open(os.path.join(mdir, meta["image"])).convert("L")
            img = ImageOps.invert(img)   # 벽(검정)→흰색: 어두운 배경에서 보이게
            buf = io.BytesIO()
            img.save(buf, format="PNG")
            self.map_png = buf.getvalue()
            self.map_meta = {"origin": [float(meta["origin"][0]), float(meta["origin"][1])],
                             "resolution": float(meta["resolution"]),
                             "width": img.width, "height": img.height}
        except Exception as e:
            self.get_logger().warn(f"맵 로드 실패: {e}")

    def _on_state(self, m):
        self.robots[m.name] = {
            "x": round(m.location.x, 3), "y": round(m.location.y, 3),
            "yaw": round(m.location.yaw, 4),   # 실제 Gazebo heading(rad) → 콘솔 화살표 방향
            "mode": ROBOT_MODE.get(m.mode.mode, str(m.mode.mode)),
            "battery": round(m.battery_percent, 0),
            "task": m.task_id,
        }

    def _on_task(self, m):
        self.task_log.append({"task_id": m.task_id, "state": m.state, "robot": m.robot_id})
        self.task_log = self.task_log[-60:]

    def _on_path(self, m):
        self.paths[m.robot_name] = [[round(p.x, 3), round(p.y, 3)] for p in m.path]

    def _on_routes(self, m):
        try:
            self.routes = json.loads(m.data)
        except Exception:
            pass

    def _on_goals(self, m):
        try:
            self.goals = json.loads(m.data)
        except Exception:
            pass

    def _on_occ(self, m):
        try:
            self.occupancy = json.loads(m.data)
        except Exception:
            pass

    def submit(self, goal, robot="", priority=0, arm_actions=0, name=""):
        if not self.task_cli.wait_for_service(timeout_sec=2.0):
            return {"accepted": False, "reason": "fleet_unavailable"}
        req = SubmitTask.Request(task_type="delivery", dropoff=str(goal), robot=robot,
                                 requester=str(name).strip(),   # 커스텀 작업 이름(비우면 자동 T-N)
                                 priority=int(priority), arm_actions=int(arm_actions))
        r = _spin_future(self, self.task_cli.call_async(req))
        if r is None:
            return {"accepted": False, "reason": "timeout"}
        return {"accepted": r.accepted, "task_id": r.task_id, "reason": r.reason}

    def bid_scores(self, goal):
        """Auction 입찰 = 각 로봇 최근접 정점→goal Dijkstra 경로비용. 낮을수록 유리."""
        import heapq
        V = self.vertices
        adj = {}
        for a, b in self.lanes:
            adj.setdefault(a, []).append(b)
        def d(i, j):
            return ((V[i][0]-V[j][0])**2 + (V[i][1]-V[j][1])**2) ** 0.5
        def nearest(x, y):
            return min(range(len(V)), key=lambda i: (V[i][0]-x)**2 + (V[i][1]-y)**2)
        def cost(src):
            INF = float('inf'); dist = [INF]*len(V); dist[src] = 0; pq = [(0.0, src)]
            while pq:
                cd, u = heapq.heappop(pq)
                if cd > dist[u]: continue
                if u == goal: return cd
                for w in adj.get(u, []):
                    nd = cd + d(u, w)
                    if nd < dist[w]: dist[w] = nd; heapq.heappush(pq, (nd, w))
            return dist[goal]
        out = []
        for name, r in self.robots.items():
            src = nearest(r["x"], r["y"])
            c = cost(src)
            out.append({"robot": name, "nearest": src,
                        "bid": None if c == float("inf") else round(c, 2)})
        out.sort(key=lambda e: (e["bid"] is None, e["bid"] if e["bid"] is not None else 0))
        winner = out[0]["robot"] if out and out[0]["bid"] is not None else None
        return {"goal": goal, "bids": out, "winner": winner}

    def set_battery(self, robot, value):
        self.battery_override[robot] = float(value)   # 콘솔 표시 즉시 반영
        if not self.battery_cli.wait_for_service(timeout_sec=1.0):
            return {"ok": False, "reason": "fleet_unavailable"}
        req = SetBattery.Request(robot=robot, value=float(value))   # 실제 fleet(완주 관문)에 반영
        r = _spin_future(self, self.battery_cli.call_async(req))
        if r is None:
            return {"ok": False, "reason": "timeout"}
        return {"ok": r.ok, "reason": r.reason}

    def set_mode(self, robot, mode):
        if not self.mode_cli.wait_for_service(timeout_sec=1.0):
            return {"ok": False, "reason": "fleet_unavailable"}
        req = SetRobotMode.Request(robot=robot, mode=mode)
        r = _spin_future(self, self.mode_cli.call_async(req))
        if r is None:
            return {"ok": False, "reason": "timeout"}
        if r.ok:
            self.fleet_mode[robot] = mode   # 상태 패널 표시용
        return {"ok": r.ok, "reason": r.reason}

    def set_plugins(self, dispatcher, traffic):
        if not self.plugins_cli.wait_for_service(timeout_sec=2.0):
            return {"ok": False, "reason": "fleet_unavailable"}
        req = SetPlugins.Request(dispatcher=dispatcher, traffic=traffic)
        r = _spin_future(self, self.plugins_cli.call_async(req))
        if r is None:
            return {"ok": False, "reason": "timeout"}
        self.active = {"dispatcher": r.active_dispatcher, "traffic": r.active_traffic}
        return {"ok": r.ok, **self.active, "reason": r.reason}

    # ── 정점 편집 ──
    def add_vertex(self, x, y):
        self.vertices.append([float(x), float(y)])
        return len(self.vertices) - 1

    def move_vertex(self, i, x, y):
        if 0 <= i < len(self.vertices):
            self.vertices[i] = [float(x), float(y)]
            return True
        return False

    def add_lane(self, a, b):
        if a == b:
            return False
        for p, q in ((a, b), (b, a)):
            if [p, q] not in self.lanes:
                self.lanes.append([p, q])
        return True

    def del_lane(self, a, b):
        self.lanes = [l for l in self.lanes if set(l) != {a, b}]
        return True

    def del_vertex(self, i):
        if not (0 <= i < len(self.vertices)):
            return False
        self.vertices.pop(i)
        # i 와 닿은 차선 제거 + i 이후 인덱스 한 칸씩 당김
        relanes = []
        for a, b in self.lanes:
            if a == i or b == i:
                continue
            relanes.append([a - 1 if a > i else a, b - 1 if b > i else b])
        self.lanes = relanes
        return True

    def save_navgraph(self):
        lvl = self.ng_full["levels"]["L1"]
        lvl["vertices"] = [[v[0], v[1], {"name": ""}] for v in self.vertices]
        lvl["lanes"] = [[l[0], l[1], {}] for l in self.lanes]
        with open(NAVGRAPH, "w") as f:
            yaml.safe_dump(self.ng_full, f, allow_unicode=True, sort_keys=False)
        msg = "saved (fleet 미연결)"
        if self.reload_cli.wait_for_service(timeout_sec=2.0):
            r = _spin_future(self, self.reload_cli.call_async(Trigger.Request()))
            msg = r.message if r else "reload_timeout"
        return {"ok": True, "vertices": len(self.vertices), "lanes": len(self.lanes), "fleet": msg}


_bridge = None


def get_bridge():
    return _bridge


app = FastAPI(title="libi test console")


@app.on_event("startup")
def _startup():
    global _bridge
    rclpy.init()
    _bridge = Bridge()
    threading.Thread(target=lambda: rclpy.spin(_bridge), daemon=True).start()


@app.on_event("shutdown")
def _shutdown():
    try:
        rclpy.shutdown()   # --reload/종료 시 rclpy 정리 → 재시작 크래시 방지
    except Exception:
        pass


class TaskReq(BaseModel):
    goal: int
    priority: int = 0
    robot: str = ""
    arm_actions: int = 0
    name: str = ""


class PluginReq(BaseModel):
    dispatcher: str = ""
    traffic: str = ""


@app.get("/api/state")
def state():
    b = get_bridge()
    robots = {n: {**r, "battery": b.battery_override.get(n, 100),   # 기본 100, UI 설정 시 그 값
                  "fleet_mode": b.fleet_mode.get(n, "")}            # 콘솔이 설정한 fleet 모드
              for n, r in b.robots.items()}
    return {"robots": robots, "paths": b.paths, "routes": b.routes, "occupancy": b.occupancy, "goals": b.goals,
            "tasks": b.task_log[-14:][::-1],
            "vertices": b.vertices, "lanes": b.lanes, "active": b.active, "map": b.map_meta}


@app.get("/api/map")
def api_map():
    return Response(content=get_bridge().map_png, media_type="image/png")


@app.post("/api/task")
def api_task(t: TaskReq):
    return get_bridge().submit(t.goal, t.robot, t.priority, t.arm_actions, t.name)


class ModeReq(BaseModel):
    robot: str
    mode: str


@app.post("/api/mode")
def api_mode(m: ModeReq):
    return get_bridge().set_mode(m.robot, m.mode)


class BidReq(BaseModel):
    goal: int


@app.post("/api/bids")
def api_bids(b: BidReq):
    return get_bridge().bid_scores(b.goal)


class BatteryReq(BaseModel):
    robot: str
    value: float


@app.post("/api/battery")
def api_battery(b: BatteryReq):
    return get_bridge().set_battery(b.robot, b.value)


def _read_algo_params():
    """algo_params.yaml 에서 3개 값을 읽어 dict 반환(없거나 깨지면 기본값)."""
    vals = {k: spec[1] for k, spec in ALGO_SPEC.items()}
    try:
        doc = yaml.safe_load(open(ALGO_PARAMS)) or {}
        ros = (doc.get("/**") or {}).get("ros__parameters", {})
        for k in ALGO_SPEC:
            if k in ros:
                vals[k] = float(ros[k])
    except FileNotFoundError:
        pass
    return vals


@app.get("/api/params")
def api_params_get():
    return {"params": _read_algo_params(),
            "spec": {k: {"label": s[0], "min": s[2], "max": s[3]} for k, s in ALGO_SPEC.items()}}


@app.post("/api/params")
def api_params_set(p: dict):
    """3개 파라미터 검증 후 yaml 저장. fleet 재시작 시 적용."""
    out = {}
    for k, (label, default, lo, hi) in ALGO_SPEC.items():
        if k not in p:
            return {"ok": False, "reason": f"missing:{k}"}
        try:
            v = float(p[k])
        except (TypeError, ValueError):
            return {"ok": False, "reason": f"not_number:{k}"}
        if not (lo <= v <= hi):
            return {"ok": False, "reason": f"out_of_range:{k} ({lo}~{hi})"}
        out[k] = v
    os.makedirs(os.path.dirname(ALGO_PARAMS), exist_ok=True)
    with open(ALGO_PARAMS, "w") as f:
        yaml.safe_dump({"/**": {"ros__parameters": out}}, f, default_flow_style=False, sort_keys=False)
    return {"ok": True, "params": out, "note": "fleet 재시작 시 적용"}


@app.post("/api/plugins")
def api_plugins(p: PluginReq):
    return get_bridge().set_plugins(p.dispatcher, p.traffic)


class VertexReq(BaseModel):
    x: float
    y: float
    index: int = -1


class LaneReq(BaseModel):
    a: int
    b: int


@app.post("/api/vertex")
def api_vertex(v: VertexReq):
    b = get_bridge()
    if v.index >= 0:
        return {"ok": b.move_vertex(v.index, v.x, v.y), "index": v.index}
    return {"ok": True, "index": b.add_vertex(v.x, v.y)}


@app.post("/api/lane")
def api_lane(lane: LaneReq):
    return {"ok": get_bridge().add_lane(lane.a, lane.b)}


@app.post("/api/lane/del")
def api_lane_del(lane: LaneReq):
    return {"ok": get_bridge().del_lane(lane.a, lane.b)}


@app.post("/api/navgraph/save")
def api_save():
    return get_bridge().save_navgraph()


class IndexReq(BaseModel):
    index: int


@app.post("/api/vertex/del")
def api_vertex_del(r: IndexReq):
    b = get_bridge()
    ok = b.del_vertex(r.index)
    return {"ok": ok, "vertices": len(b.vertices), "lanes": len(b.lanes)}


@app.get("/", response_class=HTMLResponse)
def index():
    return HTML


# ── blueprint ops 콘솔: rmf-web 패턴(Map/Robots/Tasks) + 알고리즘 스왑 ──
HTML = open(os.path.join(os.path.dirname(__file__), "console_page.html"),
            encoding="utf-8").read()   # 프론트(HTML/CSS/JS)는 별 파일로 분리
