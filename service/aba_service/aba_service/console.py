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

    def _on_occ(self, m):
        try:
            self.occupancy = json.loads(m.data)
        except Exception:
            pass

    def submit(self, goal, robot="", priority=0, arm_actions=0):
        if not self.task_cli.wait_for_service(timeout_sec=2.0):
            return {"accepted": False, "reason": "fleet_unavailable"}
        req = SubmitTask.Request(task_type="delivery", dropoff=str(goal), robot=robot,
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


class TaskReq(BaseModel):
    goal: int
    priority: int = 0
    robot: str = ""
    arm_actions: int = 0


class PluginReq(BaseModel):
    dispatcher: str = ""
    traffic: str = ""


@app.get("/api/state")
def state():
    b = get_bridge()
    robots = {n: {**r, "battery": b.battery_override.get(n, 100),   # 기본 100, UI 설정 시 그 값
                  "fleet_mode": b.fleet_mode.get(n, "")}            # 콘솔이 설정한 fleet 모드
              for n, r in b.robots.items()}
    return {"robots": robots, "paths": b.paths, "routes": b.routes, "occupancy": b.occupancy,
            "tasks": b.task_log[-14:][::-1],
            "vertices": b.vertices, "lanes": b.lanes, "active": b.active, "map": b.map_meta}


@app.get("/api/map")
def api_map():
    return Response(content=get_bridge().map_png, media_type="image/png")


@app.post("/api/task")
def api_task(t: TaskReq):
    return get_bridge().submit(t.goal, t.robot, t.priority, t.arm_actions)


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
HTML = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LIBI · FLEET OPS</title>
<style>
:root{
  --ink:#07182a; --ink2:#0a2236; --panel:#0b2034; --edge:#163a55; --grid:#0f2c45;
  --lane:#2f7fb0; --node:#79d2ff; --chalk:#e6f4fb; --dim:#6f93ab; --amber:#ffb65c;
  --r1:#ff5d62; --r2:#36d98a; --r3:#4ea3ff; --ok:#37d98a; --bad:#ff7a7a;
  --mono:ui-monospace,"SFMono-Regular",Menlo,Consolas,monospace;
  --sans:ui-sans-serif,system-ui,"Segoe UI",Roboto,sans-serif;
}
*{box-sizing:border-box}html,body{margin:0;height:100%}
body{background:var(--ink);color:var(--chalk);font-family:var(--sans);display:flex;flex-direction:column;overflow:hidden}
.bar{display:flex;align-items:center;gap:14px;padding:9px 16px;background:linear-gradient(180deg,var(--ink2),var(--ink));border-bottom:1px solid var(--edge)}
.brand{font-weight:800;letter-spacing:.22em;font-size:14px;white-space:nowrap}
.brand small{color:var(--dim);font-weight:600;letter-spacing:.3em;margin-left:6px}
.kpis{display:flex;gap:8px;margin-left:auto;font-family:var(--mono);font-size:12px;flex-wrap:wrap}
.kpi{border:1px solid var(--edge);border-radius:2px;padding:4px 10px;color:var(--dim);display:flex;gap:6px;align-items:baseline}
.kpi b{color:var(--node);font-size:14px} .kpi.t b{color:var(--amber)} .kpi.d b{color:var(--chalk)}
.kpi #c_l.up{color:var(--ok)} .kpi #c_l.down{color:var(--bad)}
.main{flex:1;display:flex;min-height:0}
.map{flex:1;position:relative;background:radial-gradient(120% 120% at 30% 8%,#0c2740 0%,var(--ink) 70%)}
canvas{position:absolute;inset:0;width:100%;height:100%}
.maptag{position:absolute;left:14px;top:12px;font-family:var(--mono);font-size:11px;letter-spacing:.18em;color:var(--dim)}
.panel{width:340px;background:var(--panel);border-left:1px solid var(--edge);display:flex;flex-direction:column;overflow:auto}
.statuspanel{width:252px;background:var(--panel);border-right:1px solid var(--edge);display:flex;flex-direction:column;overflow:auto}
.statuspanel>.ey{padding:13px 14px 2px}
.rcard{margin:8px 12px;padding:10px 11px;background:var(--ink2);border:1px solid var(--grid);border-left:3px solid var(--dim);border-radius:3px}
.rcard .rc-h{display:flex;align-items:center;gap:6px;font-family:var(--mono);font-size:13px;font-weight:700;color:var(--chalk)}
.rcard .rc-m{margin-left:auto;font-size:10px;font-weight:700;padding:2px 7px;border-radius:2px;letter-spacing:.04em}
.rcard .rc-row{display:flex;justify-content:space-between;font-family:var(--mono);font-size:11px;color:var(--dim);margin-top:5px}
.rcard .rc-row b{color:var(--chalk);font-weight:600}
.batt{height:7px;border-radius:4px;background:#0d2438;overflow:hidden;margin:8px 0 2px;border:1px solid var(--grid)}
.batt>i{display:block;height:100%;border-radius:4px;transition:width .3s}
.grp{padding:13px 16px;border-bottom:1px solid var(--grid)}
.ey{font-family:var(--mono);font-size:10px;letter-spacing:.26em;color:var(--dim);text-transform:uppercase;margin-bottom:9px}
label{font-size:11px;color:var(--dim);display:block;margin:8px 0 3px}
select{width:100%;background:var(--ink2);color:var(--chalk);border:1px solid var(--edge);border-radius:2px;padding:7px 8px;font-family:var(--mono);font-size:13px}
.btns{display:flex;gap:6px;margin-top:10px;flex-wrap:wrap}
button{flex:1;min-width:0;cursor:pointer;border:1px solid var(--edge);border-radius:2px;padding:9px 8px;font-family:var(--sans);font-size:12px;font-weight:600;background:var(--ink2);color:var(--chalk);transition:.12s}
button:hover{border-color:var(--node);color:#fff}
button.go{background:rgba(54,217,138,.12);border-color:#1f6e4d}
button.warn{background:rgba(255,182,92,.12);border-color:#7a5a22}
button.cmd{background:rgba(78,163,255,.12);border-color:#2a5c8a}
button:focus-visible{outline:2px solid var(--amber);outline-offset:1px}
.hint{font-size:10px;color:var(--dim);margin-top:7px;line-height:1.5}
table{width:100%;border-collapse:collapse;font-family:var(--mono);font-size:11px}
th{text-align:left;color:var(--dim);font-weight:500;padding:3px 4px;border-bottom:1px solid var(--grid)}
td{padding:4px;border-bottom:1px solid #0d2438}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px;vertical-align:middle}
.m-MOVING{color:var(--amber)} .m-IDLE{color:var(--dim)} .m-PAUSED{color:var(--bad)}
#feed{font-family:var(--mono);font-size:11px;line-height:1.7;max-height:150px;overflow:auto}
#feed .s{display:inline-block;width:78px} .st-ASSIGNED{color:var(--node)} .st-EXECUTING{color:var(--amber)} .st-COMPLETED{color:var(--ok)} .st-REJECTED,.st-FAILED{color:var(--bad)}
#log{font-family:var(--mono);font-size:11px;line-height:1.6;background:#05111d;color:#8fe7c4;padding:8px 10px;max-height:120px;overflow:auto;white-space:pre-wrap;border:1px solid var(--grid);border-radius:2px;margin-top:8px}
#log .t{color:var(--dim)} #log .e{color:#ff8a8a}
@media (prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<div class="bar">
  <div class="brand">LIBI<small>FLEET OPS</small></div>
  <div class="kpis">
    <span class="kpi">로봇 <b id="c_n">0</b></span>
    <span class="kpi">주행 <b id="c_m">0</b></span>
    <span class="kpi">진행 task <b id="c_a">0</b></span>
    <span class="kpi d">배차 <b id="c_d">·</b></span>
    <span class="kpi t">교통 <b id="c_t">·</b></span>
    <span class="kpi">link <b id="c_l">···</b></span>
  </div>
</div>
<div class="main">
  <aside class="statuspanel">
    <div class="ey">로봇 상태</div>
    <div id="robotcards"></div>
  </aside>
  <div class="map"><span class="maptag">NAVGRAPH · L1 · world coords (m)</span><canvas id="cv"></canvas></div>
  <aside class="panel">
    <div class="grp">
      <div class="ey">① 특정 로봇 배차</div>
      <label>로봇</label><select id="t_robot"></select>
      <label>목적지 정점</label><select id="t_vertex"></select>
      <label>task 우선도 (참고용)</label>
      <input id="t_prio" type="number" value="0" min="0" max="9"
             style="width:100%;box-sizing:border-box;padding:6px;background:#0a2236;color:#cfe6f5;border:1px solid #24506e;border-radius:6px">
      <label>로봇팔 동작 횟수 (배터리 완주 판단에 반영)</label>
      <input id="t_arm" type="number" value="0" min="0" max="99"
             style="width:100%;box-sizing:border-box;padding:6px;background:#0a2236;color:#cfe6f5;border:1px solid #24506e;border-radius:6px">
      <div class="btns"><button class="go" style="width:100%" onclick="doTaskRobot()">이 로봇에 배차 ▸</button></div>
      <div class="hint">선택 로봇을 <b>대기</b> 상태로 둔 뒤 배차 · 소비=주행1m당1% + 팔1회당0.5%, 배터리<소비+15%면 거절</div>
    </div>
    <div class="grp">
      <div class="ey">② 자동 배차 · 경매</div>
      <label>목적지 정점</label><select id="a_vertex"></select>
      <label>task 우선도 (참고용)</label>
      <input id="a_prio" type="number" value="0" min="0" max="9"
             style="width:100%;box-sizing:border-box;padding:6px;background:#0a2236;color:#cfe6f5;border:1px solid #24506e;border-radius:6px">
      <label>로봇팔 동작 횟수 (배터리 완주 판단에 반영)</label>
      <input id="a_arm" type="number" value="0" min="0" max="99"
             style="width:100%;box-sizing:border-box;padding:6px;background:#0a2236;color:#cfe6f5;border:1px solid #24506e;border-radius:6px">
      <div class="btns"><button class="go" style="width:100%" onclick="doTaskDisp()">경매 배차 · 점수 보기 ▸</button></div>
      <div class="hint">dispatcher가 완주 가능 로봇 중 최저 입찰(경로비용) 선택 → 점수 팝업</div>
    </div>
    <div class="grp">
      <div class="ey">③ 로봇 상태 · 배터리</div>
      <label>로봇</label><select id="s_robot"></select>
      <div class="btns">
        <button onclick="doMode('PATROL')">순회</button>
        <button onclick="doMode('IDLE')">대기</button>
        <button class="warn" onclick="doMode('STOP')">정지</button>
        <button class="cmd" onclick="doMode('CHARGE')">충전복귀</button>
      </div>
      <div class="hint">순회=자동순찰 · 대기=배차가능 · 정지=멈춤(우회) · 충전복귀=최우선(교착 시 안 비킴, sim 태그)</div>
      <label style="margin-top:8px">배터리 (%)</label>
      <div style="display:flex;gap:6px">
        <input id="bat" type="number" value="100" min="0" max="100"
               style="flex:1;box-sizing:border-box;padding:6px;background:#0a2236;color:#cfe6f5;border:1px solid #24506e;border-radius:6px">
        <button onclick="doBattery()">적용</button>
      </div>
    </div>
    <div class="grp">
      <div class="ey">로봇 텔레메트리</div>
      <table><thead><tr><th>로봇</th><th>상태</th><th>배터리</th><th>위치</th></tr></thead><tbody id="rtab"></tbody></table>
    </div>
    <div class="grp">
      <div class="ey">태스크 피드</div>
      <div id="feed"></div>
    </div>
    <div class="grp">
      <div class="ey">요청 로그</div>
      <div id="log"></div>
    </div>
  </aside>
</div>
<div id="bidModal" style="display:none;position:fixed;inset:0;background:rgba(3,10,18,.72);z-index:50;align-items:center;justify-content:center">
  <div style="background:#0a1b2b;border:1px solid #24506e;border-radius:10px;padding:18px 20px;min-width:340px;box-shadow:0 12px 44px rgba(0,0,0,.6)">
    <div style="font:600 14px ui-monospace;color:#cfe6f5;margin-bottom:3px">🏷 경매 입찰 결과 · Auction/SSI</div>
    <div id="bidSub" style="font:11px ui-monospace;color:#6f93ab;margin-bottom:10px"></div>
    <table id="bidTab" style="width:100%;border-collapse:collapse;font:12px ui-monospace;color:#cfe6f5"></table>
    <div style="display:flex;gap:8px;margin-top:14px;justify-content:flex-end">
      <button onclick="closeBids()">닫기</button>
      <button class="go" onclick="confirmBids()">이 결과로 배차</button>
    </div>
  </div>
</div>
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
let S=null, disp={}, T=null, mapImg=null, edit=false, sel=-1, drag=-1, moved=false, downEmpty=null;
function log(m,err){const l=document.getElementById('log');
  l.insertAdjacentHTML('afterbegin',`<span class="t">${new Date().toLocaleTimeString()}</span>  <span class="${err?'e':''}">${m}</span>\n`);}
function bounds(v){const xs=v.map(p=>p[0]),ys=v.map(p=>p[1]);return[Math.min(...xs)-1.2,Math.max(...xs)+1.2,Math.min(...ys)-1.2,Math.max(...ys)+1.2];}
function draw(){
  if(!S||!S.vertices.length)return;
  const dpr=devicePixelRatio||1,W=cv.clientWidth,H=cv.clientHeight;
  cv.width=W*dpr;cv.height=H*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,W,H);
  const v=S.vertices,[xmin,xmax,ymin,ymax]=bounds(v);
  const s=Math.min(W/(xmax-xmin),H/(ymax-ymin))*0.92;
  const ox=(W-(xmax-xmin)*s)/2, oy=(H-(ymax-ymin)*s)/2;
  const X=x=>ox+(x-xmin)*s, Y=y=>H-(oy+(y-ymin)*s);
  T={xmin,ymin,s,ox,oy,H};
  if(S.map){ if(!mapImg){mapImg=new Image();mapImg.src='/api/map';}
    if(mapImg.complete&&mapImg.naturalWidth){const m=S.map,mw=m.width*m.resolution,mh=m.height*m.resolution;
      ctx.globalAlpha=0.5;ctx.imageSmoothingEnabled=false;ctx.drawImage(mapImg,X(m.origin[0]),Y(m.origin[1]+mh),mw*s,mh*s);ctx.globalAlpha=1;}}
  ctx.strokeStyle='rgba(40,90,130,.14)';ctx.lineWidth=1;
  for(let gx=Math.ceil(xmin);gx<xmax;gx++){ctx.beginPath();ctx.moveTo(X(gx),0);ctx.lineTo(X(gx),H);ctx.stroke();}
  for(let gy=Math.ceil(ymin);gy<ymax;gy++){ctx.beginPath();ctx.moveTo(0,Y(gy));ctx.lineTo(W,Y(gy));ctx.stroke();}
  ctx.strokeStyle='rgba(240,246,252,.92)';ctx.lineWidth=2.6;ctx.shadowColor='rgba(255,255,255,.5)';ctx.shadowBlur=6;
  S.lanes.forEach(([a,b])=>{ctx.beginPath();ctx.moveTo(X(v[a][0]),Y(v[a][1]));ctx.lineTo(X(v[b][0]),Y(v[b][1]));ctx.stroke();});
  ctx.shadowBlur=0;
  const now=performance.now();
  const rcols={pinky1:'#ff5d62',pinky2:'#36d98a',pinky3:'#4ea3ff'},occ=S.occupancy||{};
  v.forEach((p,i)=>{
    const owner=occ[i], oc=owner?(rcols[owner]||'#c66bff'):null;
    if(oc){   // 점유 노드: 로봇 색으로 크게 채움 + 이중 글로우 링 (실제 예약 상태)
      const pr=13+2*Math.sin(now/300);
      ctx.shadowColor=oc;ctx.shadowBlur=26;ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),14,0,7);ctx.fillStyle=oc;ctx.fill();ctx.shadowBlur=0;
      ctx.lineWidth=3;ctx.strokeStyle=oc;ctx.globalAlpha=.85;ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),20,0,7);ctx.stroke();
      ctx.globalAlpha=.4;ctx.lineWidth=2;ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),20+pr,0,7);ctx.stroke();ctx.globalAlpha=1;
      ctx.fillStyle='#06121e';ctx.font='bold 11px ui-monospace';ctx.fillText(owner.replace('pinky','P'),X(p[0])-7,Y(p[1])+4);
    }else{
      ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),5,0,7);ctx.fillStyle='#0a2236';ctx.fill();
      ctx.lineWidth=1.6;ctx.strokeStyle='#79d2ff';ctx.stroke();
    }
    ctx.fillStyle=oc?'#eaf6ff':'#6f93ab';ctx.font='11px ui-monospace';ctx.fillText('v'+i,X(p[0])+(oc?22:10),Y(p[1])-(oc?12:7));});
  if(edit&&sel>=0&&v[sel]){ctx.strokeStyle='#ffb65c';ctx.lineWidth=2.4;ctx.beginPath();ctx.arc(X(v[sel][0]),Y(v[sel][1]),12,0,7);ctx.stroke();}
  const cols={pinky1:'#ff5d62',pinky2:'#36d98a',pinky3:'#4ea3ff'};
  // ── 로봇이 갈 경로(목표 노드까지 남은 route): 은은한 바탕선 + 흐르는 점선 + 화살촉 + 목표 펄스 ──
  //    shadowBlur 미사용(캔버스 렉의 주범) → 60fps 에서 가벼움.
  ctx.lineCap='round';ctx.lineJoin='round';
  for(const[name,pw]of Object.entries(S.routes||{})){
    if(!pw||pw.length<2)continue;
    const c=cols[name]||'#c66bff', pts=pw.map(q=>[X(q[0]),Y(q[1])]);
    ctx.strokeStyle=c;ctx.globalAlpha=.20;ctx.lineWidth=8;
    ctx.beginPath();pts.forEach((q,i)=>i?ctx.lineTo(q[0],q[1]):ctx.moveTo(q[0],q[1]));ctx.stroke();
    ctx.globalAlpha=.95;ctx.lineWidth=3;ctx.setLineDash([12,9]);ctx.lineDashOffset=-(now/45)%21;
    ctx.beginPath();pts.forEach((q,i)=>i?ctx.lineTo(q[0],q[1]):ctx.moveTo(q[0],q[1]));ctx.stroke();
    ctx.setLineDash([]);ctx.globalAlpha=1;
    const a=pts[pts.length-2],b=pts[pts.length-1],ang=Math.atan2(b[1]-a[1],b[0]-a[0]);
    ctx.fillStyle=c;ctx.beginPath();ctx.moveTo(b[0],b[1]);
    ctx.lineTo(b[0]-14*Math.cos(ang-.42),b[1]-14*Math.sin(ang-.42));
    ctx.lineTo(b[0]-14*Math.cos(ang+.42),b[1]-14*Math.sin(ang+.42));ctx.closePath();ctx.fill();
    ctx.strokeStyle=c;ctx.globalAlpha=.8;ctx.lineWidth=2;ctx.beginPath();ctx.arc(b[0],b[1],7+3*Math.sin(now/240),0,7);ctx.stroke();ctx.globalAlpha=1;
  }
  for(const[name,r]of Object.entries(S.robots)){
    const c=cols[name]||'#c66bff';
    // 실시간 부드럽게: 표시 위치를 실제 위치로 매 프레임 보간(잔상 없음). 순간이동(>1.5m)은 스냅.
    const d=disp[name]=disp[name]||[r.x,r.y];
    if(Math.hypot(r.x-d[0],r.y-d[1])>1.5){d[0]=r.x;d[1]=r.y;}else{d[0]+=(r.x-d[0])*0.25;d[1]+=(r.y-d[1])*0.25;}
    const px=X(d[0]),py=Y(d[1]);
    ctx.shadowColor=c;ctx.shadowBlur=14;ctx.fillStyle=c;ctx.beginPath();ctx.arc(px,py,9,0,7);ctx.fill();ctx.shadowBlur=0;
    ctx.fillStyle='#06121e';ctx.font='bold 10px ui-monospace';ctx.fillText(name.replace('pinky','P'),px-6,py+3);
    ctx.fillStyle=c;ctx.font='11px ui-monospace';ctx.fillText(name,px+12,py-10);
  }
}
function short(s){return (s||'?').split('::').pop();}
function battColor(b){return b>50?'#37d98a':(b>=15?'#ffb65c':'#ff7a7a');}
function modeChip(fm,rm){const M={PATROL:['순회','#4ea3ff'],IDLE:['대기','#6f93ab'],STOP:['정지','#ff7a7a'],CHARGE:['충전복귀','#c66bff']};
  return (fm&&M[fm])?M[fm]:[rm||'—','#ffb65c'];}
function renderCards(){
  if(!S)return; const cols={pinky1:'#ff5d62',pinky2:'#36d98a',pinky3:'#4ea3ff'};
  const rn=Object.keys(S.robots).sort();
  document.getElementById('robotcards').innerHTML = rn.map(n=>{
    const r=S.robots[n],c=cols[n]||'#c66bff',b=Math.round(r.battery),bc=battColor(b),mc=modeChip(r.fleet_mode,r.mode);
    return `<div class="rcard" style="border-left-color:${c}">
      <div class="rc-h"><span class="dot" style="background:${c}"></span>${n}<span class="rc-m" style="background:${mc[1]}22;color:${mc[1]}">${mc[0]}</span></div>
      <div class="batt"><i style="width:${b}%;background:${bc}"></i></div>
      <div class="rc-row"><span>배터리</span><b style="color:${bc}">${b}%</b></div>
      <div class="rc-row"><span>task</span><b>${r.task||'—'}</b></div>
      <div class="rc-row"><span>위치</span><b>${r.x.toFixed(1)}, ${r.y.toFixed(1)}</b></div>
    </div>`;}).join('') || '<div style="color:var(--dim);font-size:11px;padding:8px 14px">로봇 대기 중… (sim 미연결)</div>';
}
function fillSel(id,items,fmt){const el=document.getElementById(id);const cur=el.value;
  if(el.options.length!==items.length){el.innerHTML='';items.forEach((it,i)=>el.add(new Option(fmt(it,i),fmt(it,i,true))));if(cur)el.value=cur;}}
async function poll(){
  try{S=await (await fetch('/api/state')).json();
    const rn=Object.keys(S.robots);
    document.getElementById('c_n').textContent=rn.length;
    document.getElementById('c_m').textContent=rn.filter(n=>S.robots[n].mode==='MOVING').length;
    document.getElementById('c_a').textContent=new Set(S.tasks.filter(t=>t.state==='ASSIGNED'||t.state==='EXECUTING').map(t=>t.task_id)).size;
    document.getElementById('c_d').textContent=short(S.active.dispatcher);
    document.getElementById('c_t').textContent=short(S.active.traffic);
    const cl=document.getElementById('c_l');cl.textContent='UP';cl.className='up';
    const rsort=rn.slice().sort();
    ['t_robot','s_robot'].forEach(id=>{const el=document.getElementById(id);if(el&&el.options.length!==rsort.length){const cur=el.value;el.innerHTML='';rsort.forEach(n=>el.add(new Option(n,n)));if(cur)el.value=cur;}});
    ['t_vertex','a_vertex'].forEach(id=>{const el=document.getElementById(id);if(el&&el.options.length!==S.vertices.length){el.innerHTML='';S.vertices.forEach((p,i)=>el.add(new Option('v'+i+'  ('+p[0].toFixed(1)+', '+p[1].toFixed(1)+')',i)));}});
    // robots table
    const cols={pinky1:'#ff5d62',pinky2:'#36d98a',pinky3:'#4ea3ff'};
    document.getElementById('rtab').innerHTML=rn.sort().map(n=>{const r=S.robots[n];
      return `<tr><td><span class="dot" style="background:${cols[n]||'#c66bff'}"></span>${n}</td><td class="m-${r.mode}">${r.mode}</td><td>${r.battery}%</td><td>${r.x.toFixed(1)}, ${r.y.toFixed(1)}</td></tr>`;}).join('');
    // task feed
    document.getElementById('feed').innerHTML=S.tasks.map(t=>`<div><span class="s st-${t.state}">${t.state}</span> ${t.task_id} · ${t.robot}</div>`).join('')||'<div style="color:var(--dim)">대기 중…</div>';
    renderCards();
    draw();
  }catch(e){const cl=document.getElementById('c_l');cl.textContent='DOWN';cl.className='down';}
}
async function post(u,b){try{const r=await (await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})).json();
  const bad=r.accepted===false||r.ok===false;log(u.replace('/api/','')+' '+JSON.stringify(b)+' → '+JSON.stringify(r),bad);return r;}
  catch(e){log('요청 실패: '+u,true);}}
function val(id){const e=document.getElementById(id);return e?e.value:'';}
function ival(id){return parseInt(val(id))||0;}
function doTaskRobot(){post('/api/task',{goal:ival('t_vertex'),robot:val('t_robot'),priority:ival('t_prio'),arm_actions:ival('t_arm')});}
function doMode(m){post('/api/mode',{robot:val('s_robot'),mode:m});}
function doBattery(){post('/api/battery',{robot:val('s_robot'),value:parseFloat(val('bat'))});}
let bidGoal=null,bidPrio=0,bidArm=0;
async function doTaskDisp(){   // 자동배차: 경매 점수 팝업 후 배차
  const v=ival('a_vertex'); bidGoal=v; bidPrio=ival('a_prio'); bidArm=ival('a_arm');
  const r=await (await fetch('/api/bids',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({goal:v})})).json();
  const cols={pinky1:'#ff5d62',pinky2:'#36d98a',pinky3:'#4ea3ff'};
  const rows=r.bids.map(b=>{const win=b.robot===r.winner;return `<tr style="border-top:1px solid #16324a">`+
    `<td style="padding:6px 8px"><span class="dot" style="background:${cols[b.robot]||'#c66bff'}"></span>${b.robot}${win?' 🏆':''}</td>`+
    `<td style="padding:6px 8px;color:#6f93ab">v${b.nearest} 진입</td>`+
    `<td style="padding:6px 8px;text-align:right;${win?'color:#36d98a;font-weight:700':''}">${b.bid===null?'∞ 도달불가':b.bid.toFixed(2)+' m'}</td></tr>`;}).join('');
  document.getElementById('bidTab').innerHTML=`<tr style="color:#6f93ab;font-size:11px"><td style="padding:4px 8px">로봇</td><td style="padding:4px 8px">진입점</td><td style="padding:4px 8px;text-align:right">입찰가(경로비용)</td></tr>`+rows;
  document.getElementById('bidSub').textContent='목적지 v'+v+' · 최저 입찰가 낙찰 → '+(r.winner||'없음')+' (우선순위 '+bidPrio+')';
  document.getElementById('bidModal').style.display='flex';
}
function closeBids(){document.getElementById('bidModal').style.display='none';}
function confirmBids(){closeBids();post('/api/task',{goal:bidGoal,priority:bidPrio,arm_actions:bidArm});}
// ── 정점 편집 ──
function epos(e){const r=cv.getBoundingClientRect();return[e.clientX-r.left,e.clientY-r.top];}
function c2w(cx,cy){return[T.xmin+(cx-T.ox)/T.s, T.ymin+(T.H-cy-T.oy)/T.s];}
function nearV(cx,cy){let best=-1,bd=14;if(!S||!T)return -1;S.vertices.forEach((p,i)=>{const x=T.ox+(p[0]-T.xmin)*T.s,y=T.H-(T.oy+(p[1]-T.ymin)*T.s);const d=Math.hypot(x-cx,y-cy);if(d<bd){bd=d;best=i;}});return best;}
function toggleEdit(){edit=!edit;sel=-1;const b=document.getElementById('editbtn');b.textContent='편집 '+(edit?'ON':'OFF');b.style.borderColor=edit?'#ffb65c':'';b.style.color=edit?'#ffb65c':'';}
cv.addEventListener('mousedown',e=>{if(!edit||!T)return;const[cx,cy]=epos(e);const vi=nearV(cx,cy);moved=false;if(vi>=0){drag=vi;downEmpty=null;}else{drag=-1;downEmpty=[cx,cy];}});
cv.addEventListener('mousemove',e=>{if(!edit||drag<0)return;const[cx,cy]=epos(e);S.vertices[drag]=c2w(cx,cy);moved=true;draw();});
cv.addEventListener('mouseup',e=>{if(!edit)return;const[cx,cy]=epos(e);
  if(drag>=0){if(moved){const w=c2w(cx,cy);post('/api/vertex',{x:w[0],y:w[1],index:drag});}
    else{if(sel<0){sel=drag;log('정점 v'+drag+' 선택 — 다른 정점 클릭=차선');}else if(sel===drag){sel=-1;}else{post('/api/lane',{a:sel,b:drag});sel=-1;}}
    drag=-1;return;}
  if(downEmpty){const w=c2w(cx,cy);post('/api/vertex',{x:w[0],y:w[1]});downEmpty=null;}});
function saveNg(){post('/api/navgraph/save',{});}
function delVertex(){if(sel<0){log('삭제할 정점을 먼저 클릭해 선택하세요',true);return;}post('/api/vertex/del',{index:sel}).then(()=>{sel=-1;});}
cv.addEventListener('contextmenu',e=>{if(!edit||!T)return;e.preventDefault();const[cx,cy]=epos(e);const vi=nearV(cx,cy);if(vi>=0){post('/api/vertex/del',{index:vi}).then(()=>{if(sel===vi)sel=-1;else if(sel>vi)sel--;});}});
addEventListener('resize',draw);
setInterval(poll,200);poll();
(function anim(){draw();requestAnimationFrame(anim);})();   // 경로 흐름 애니메이션
log('console ready — 대상/목표 선택 후 명령하세요.');
</script></body></html>"""
