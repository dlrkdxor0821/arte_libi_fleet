"""libi 알고리즘 테스트 콘솔 (FastAPI).

aba_service 와 별도 포트(8001)로 띄우는 개발용 관제 콘솔. rmf-web 대시보드(Map/Robots/Tasks)
패턴을 따른 디버깅 콘솔 — ROS 브리지로 FMS/슬롯카와 통신:
- 라이브 맵(navgraph + 로봇) · 로봇 텔레메트리(mode/배터리/task) · 태스크 피드
- 정점 직접 이동(PathRequest, FMS 우회) / 특정 로봇 task / dispatcher task
- 배차·교통 알고리즘 런타임 스왑(/fms/set_plugins) → 교통 양보 on/off 대조

실행: scripts/run_console.sh  (uvicorn :8001)
"""
import io
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
from libi_fleet_msgs.srv import SubmitTask, SetPlugins
from libi_fleet_msgs.msg import TaskState
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
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.path_pub = self.create_publisher(PathRequest, "/robot_path_requests", qos)
        self.task_cli = self.create_client(SubmitTask, "/fms/submit_task")
        self.plugins_cli = self.create_client(SetPlugins, "/fms/set_plugins")
        self.reload_cli = self.create_client(Trigger, "/fms/reload_navgraph")
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

    def submit(self, goal, robot=""):
        if not self.task_cli.wait_for_service(timeout_sec=2.0):
            return {"accepted": False, "reason": "fleet_unavailable"}
        req = SubmitTask.Request(task_type="delivery", dropoff=str(goal), robot=robot)
        r = _spin_future(self, self.task_cli.call_async(req))
        if r is None:
            return {"accepted": False, "reason": "timeout"}
        return {"accepted": r.accepted, "task_id": r.task_id, "reason": r.reason}

    def move(self, robot, vertex):
        if robot not in self.robots:
            return {"ok": False, "reason": "unknown_robot"}
        x0, y0 = self.robots[robot]["x"], self.robots[robot]["y"]
        vx, vy = self.vertices[vertex]
        self.seq += 1
        req = PathRequest()
        req.fleet_name = "libi"
        req.robot_name = robot
        req.task_id = f"console-{self.seq}"
        p0 = Location(); p0.x = float(x0); p0.y = float(y0); p0.level_name = "L1"
        p1 = Location(); p1.x = float(vx); p1.y = float(vy); p1.level_name = "L1"
        req.path = [p0, p1]
        t = time.time()
        while self.path_pub.get_subscription_count() < 1 and time.time() - t < 2.0:
            time.sleep(0.05)
        self.path_pub.publish(req)
        return {"ok": True, "target": [vx, vy]}

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
    robot: str = ""


class MoveReq(BaseModel):
    robot: str
    vertex: int


class PluginReq(BaseModel):
    dispatcher: str = ""
    traffic: str = ""


@app.get("/api/state")
def state():
    b = get_bridge()
    return {"robots": b.robots, "tasks": b.task_log[-14:][::-1],
            "vertices": b.vertices, "lanes": b.lanes, "active": b.active, "map": b.map_meta}


@app.get("/api/map")
def api_map():
    return Response(content=get_bridge().map_png, media_type="image/png")


@app.post("/api/task")
def api_task(t: TaskReq):
    return get_bridge().submit(t.goal, t.robot)


@app.post("/api/move")
def api_move(m: MoveReq):
    return get_bridge().move(m.robot, m.vertex)


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
  <div class="map"><span class="maptag">NAVGRAPH · L1 · world coords (m)</span><canvas id="cv"></canvas></div>
  <aside class="panel">
    <div class="grp">
      <div class="ey">대상 지정 · 명령</div>
      <label>로봇</label><select id="robot"></select>
      <label>목표 정점</label><select id="vertex"></select>
      <div class="btns">
        <button class="cmd" onclick="doMove()">직접 이동</button>
        <button class="go" onclick="doTaskRobot()">이 로봇에 배차</button>
      </div>
      <button class="go" style="margin-top:6px" onclick="doTaskDisp()">dispatcher 자동배차</button>
      <div class="hint">직접 이동 = FMS 우회(좌표 주행) · 배차 = FMS 경유(교통 적용)</div>
    </div>
    <div class="grp">
      <div class="ey">알고리즘 (런타임 스왑)</div>
      <label>배차 dispatcher</label>
      <select id="disp">
        <option value="libi_fleet::GreedyCost">GreedyCost · 최근접</option>
        <option value="libi_fleet::FarthestCost">FarthestCost · 최원거리</option>
        <option value="libi_fleet::Hungarian">Hungarian · 최적매칭 (batch)</option>
        <option value="libi_fleet::Auction">Auction/SSI · 경매 (batch)</option>
        <option value="libi_fleet::Cbba">CBBA · 묶음합의 (batch)</option>
        <option value="libi_fleet::Milp">MILP/VRP · 수리최적 (batch)</option>
        <option value="libi_fleet::GaAco">GA/ACO · 메타휴리스틱 (batch)</option>
      </select>
      <label>교통 traffic</label>
      <select id="traf">
        <option value="libi_fleet::EdgeNodeLock">EdgeNodeLock · 양보 ON</option>
        <option value="libi_fleet::NoLock">NoLock · 양보 OFF</option>
        <option value="libi_fleet::Priority">Priority · 우선순위 양보</option>
        <option value="libi_fleet::DijkstraReservation">Dijkstra+노드예약</option>
        <option value="libi_fleet::CbsAstar">CBS+A* · 경로예약 (추후)</option>
        <option value="libi_fleet::Orca">ORCA/VO · 속도양보 (추후)</option>
      </select>
      <div class="btns"><button class="warn" onclick="applyPlugins()">알고리즘 적용</button></div>
    </div>
    <div class="grp">
      <div class="ey">정점 편집 (navgraph)</div>
      <div class="btns">
        <button id="editbtn" onclick="toggleEdit()">편집 OFF</button>
        <button class="warn" onclick="saveNg()">저장 + 반영</button>
      </div>
      <div class="btns"><button onclick="delVertex()">선택 정점 삭제</button></div>
      <div class="hint">편집 ON: 빈 곳=정점 추가 · 정점→정점=차선 · 드래그=이동 · 우클릭(또는 선택 후 삭제)=정점 삭제 · 저장=기록+리로드</div>
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
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
let S=null, trails={}, T=null, mapImg=null, edit=false, sel=-1, drag=-1, moved=false, downEmpty=null;
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
  ctx.strokeStyle='rgba(47,127,176,.85)';ctx.lineWidth=2.4;ctx.shadowColor='#2f7fb0';ctx.shadowBlur=8;
  S.lanes.forEach(([a,b])=>{ctx.beginPath();ctx.moveTo(X(v[a][0]),Y(v[a][1]));ctx.lineTo(X(v[b][0]),Y(v[b][1]));ctx.stroke();});
  ctx.shadowBlur=0;
  v.forEach((p,i)=>{ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),5,0,7);ctx.fillStyle='#0a2236';ctx.fill();
    ctx.lineWidth=1.6;ctx.strokeStyle='#79d2ff';ctx.stroke();
    ctx.fillStyle='#6f93ab';ctx.font='11px ui-monospace';ctx.fillText('v'+i,X(p[0])+8,Y(p[1])-6);});
  if(edit&&sel>=0&&v[sel]){ctx.strokeStyle='#ffb65c';ctx.lineWidth=2.4;ctx.beginPath();ctx.arc(X(v[sel][0]),Y(v[sel][1]),12,0,7);ctx.stroke();}
  const cols={pinky1:'#ff5d62',pinky2:'#36d98a',pinky3:'#4ea3ff'};
  for(const[name,r]of Object.entries(S.robots)){
    const c=cols[name]||'#c66bff', p=[r.x,r.y], tr=trails[name]=trails[name]||[];
    if(!tr.length||Math.hypot(tr[tr.length-1][0]-p[0],tr[tr.length-1][1]-p[1])>0.03){tr.push(p);if(tr.length>60)tr.shift();}
    ctx.strokeStyle=c+'66';ctx.lineWidth=2;ctx.beginPath();tr.forEach((q,i)=>i?ctx.lineTo(X(q[0]),Y(q[1])):ctx.moveTo(X(q[0]),Y(q[1])));ctx.stroke();
    ctx.shadowColor=c;ctx.shadowBlur=14;ctx.fillStyle=c;ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),9,0,7);ctx.fill();ctx.shadowBlur=0;
    ctx.fillStyle='#06121e';ctx.font='bold 10px ui-monospace';ctx.fillText(name.replace('pinky','P'),X(p[0])-6,Y(p[1])+3);
    ctx.fillStyle=c;ctx.font='11px ui-monospace';ctx.fillText(name,X(p[0])+12,Y(p[1])-10);
  }
}
function short(s){return (s||'?').split('::').pop();}
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
    const rs=document.getElementById('robot');if(rs.options.length!==rn.length){const cur=rs.value;rs.innerHTML='';rn.sort().forEach(n=>rs.add(new Option(n,n)));if(cur)rs.value=cur;}
    const vs=document.getElementById('vertex');if(vs.options.length!==S.vertices.length){vs.innerHTML='';S.vertices.forEach((p,i)=>vs.add(new Option('v'+i+'  ('+p[0].toFixed(1)+', '+p[1].toFixed(1)+')',i)));}
    // robots table
    const cols={pinky1:'#ff5d62',pinky2:'#36d98a',pinky3:'#4ea3ff'};
    document.getElementById('rtab').innerHTML=rn.sort().map(n=>{const r=S.robots[n];
      return `<tr><td><span class="dot" style="background:${cols[n]||'#c66bff'}"></span>${n}</td><td class="m-${r.mode}">${r.mode}</td><td>${r.battery}%</td><td>${r.x.toFixed(1)}, ${r.y.toFixed(1)}</td></tr>`;}).join('');
    // task feed
    document.getElementById('feed').innerHTML=S.tasks.map(t=>`<div><span class="s st-${t.state}">${t.state}</span> ${t.task_id} · ${t.robot}</div>`).join('')||'<div style="color:var(--dim)">대기 중…</div>';
    draw();
  }catch(e){const cl=document.getElementById('c_l');cl.textContent='DOWN';cl.className='down';}
}
async function post(u,b){try{const r=await (await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})).json();
  const bad=r.accepted===false||r.ok===false;log(u.replace('/api/','')+' '+JSON.stringify(b)+' → '+JSON.stringify(r),bad);return r;}
  catch(e){log('요청 실패: '+u,true);}}
function rv(){return[document.getElementById('robot').value,parseInt(document.getElementById('vertex').value)];}
function doMove(){const[r,v]=rv();post('/api/move',{robot:r,vertex:v});}
function doTaskRobot(){const[r,v]=rv();post('/api/task',{goal:v,robot:r});}
function doTaskDisp(){const[,v]=rv();post('/api/task',{goal:v});}
function applyPlugins(){post('/api/plugins',{dispatcher:document.getElementById('disp').value,traffic:document.getElementById('traf').value});}
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
setInterval(poll,400);poll();
log('console ready — 대상/목표 선택 후 명령하세요.');
</script></body></html>"""
