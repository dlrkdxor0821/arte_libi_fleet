# demos/_lib.sh — 데모 시나리오 공통 함수 (콘솔 HTTP API :8001 대상, ROS 소싱 불필요)
#   사용: 각 데모 스크립트가 `source "$(dirname "$0")/_lib.sh"` 로 로드.
API="${API:-http://localhost:8001}"

post(){ curl -s -X POST "$API/api/$1" -H "Content-Type: application/json" -d "$2"; }
state(){ curl -s "$API/api/state"; }

# 콘솔 살아있는지 확인 (없으면 안내 후 종료)
require_console(){
  if ! curl -s -o /dev/null --max-time 3 "$API/api/state"; then
    echo "❌ 콘솔(:8001) 응답 없음 — 먼저 ./run_sim.sh 로 스택을 띄우세요."; exit 1
  fi
}

mode(){ post mode "{\"robot\":\"$1\",\"mode\":\"$2\"}" >/dev/null; }
battery(){ post battery "{\"robot\":\"$1\",\"value\":$2}" >/dev/null; }
dispatch(){ post task "{\"goal\":$2,\"robot\":\"$1\",\"priority\":${3:-0},\"arm_actions\":${4:-0}}"; }
auction(){ post task "{\"goal\":$1,\"robot\":\"\",\"priority\":${2:-0}}"; }

# 로봇 한 대 상태 한 줄: "mode (x,y) 다음노드→v? 최종→v?"
robot_line(){
  state | python3 -c "
import sys,json
d=json.load(sys.stdin); n='$1'; r=d.get('robots',{}).get(n)
if not r: print('  (no robot)'); sys.exit()
g=d.get('goals',{}).get(n)
print(f\"  {n}: {r['mode']:7} pos=({r['x']:.2f},{r['y']:.2f}) 최종목적지={'v'+str(g) if g is not None else '—(순회)'}\")"
}
all_lines(){ for r in pinky1 pinky2 pinky3; do robot_line "$r"; done; }

# 조건 만족까지 대기 (로봇이 IDLE=도착 될 때까지, 최대 N초)
wait_idle(){
  local rb="$1" max="${2:-30}"
  for i in $(seq 1 "$max"); do
    m=$(state | python3 -c "import sys,json;print(json.load(sys.stdin)['robots']['$rb']['mode'])" 2>/dev/null)
    [ "$m" = "IDLE" ] && return 0
    sleep 1
  done
  return 1
}
