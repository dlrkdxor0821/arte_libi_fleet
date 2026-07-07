#!/usr/bin/env python3
"""demo_yield.py — 콘솔(:8001) HTTP API로 '양보' 장면을 결정적으로 연출한다.

패트롤/랜덤 배차로는 head-on 교착이 우연에 의존 → 데모 재현이 어렵다.
이 스크립트는 우선순위를 고정해 한쪽이 확정적으로 양보하게 만든다:
  ① 전원 IDLE (패트롤 정지 — 내가 명령한 것만 움직임)
  ② 두 로봇을 복도 양 끝(v0, v3)에 배치하고 도착까지 대기
  ③ 서로를 향해 교차 배차 + 보스를 CHARGE(최상위 tier)로 고정
     → 보스는 안 비키고, 상대가 확정적으로 우회(양보).

사용:
  python3 scripts/demo_yield.py demo        # 전체 연출: reset → stage → cross (head-on 우회)
  python3 scripts/demo_yield.py queue       # 단순 양보: 뒤 로봇이 앞 로봇 뒤에서 ⏸ 대기
  python3 scripts/demo_yield.py reset       # 전원 IDLE (초기화만)

옵션:
  --host 127.0.0.1:8001   콘솔 주소 (기본 localhost:8001)
  --boss pinky1           안 비키는 로봇(기본: 이름순 첫 로봇)
  --a 0 --b 3             복도 양 끝 정점(기본: 상단 통로 좌/우 자동감지)
  --loop                  demo 를 무한 반복 (녹화 중 여러 테이크)
"""
import argparse
import json
import math
import sys
import time
import urllib.request
import urllib.error

ARRIVE = 0.30   # 도착 판정(m) — FMS kArrive(0.35)보다 약간 타이트
POLL = 0.4
PARK = 7        # 데모에 안 쓰는 남는 로봇을 치워둘 코너 정점(우측 하단)


def api(host, path, body=None):
    url = f"http://{host}{path}"
    data = json.dumps(body).encode() if body is not None else None
    headers = {"Content-Type": "application/json"} if data is not None else {}
    method = "POST" if data is not None else "GET"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=6) as r:
        return json.load(r)


def state(host):
    return api(host, "/api/state")


def set_mode(host, robot, mode):
    return api(host, "/api/mode", {"robot": robot, "mode": mode})


def submit(host, robot, goal):
    return api(host, "/api/task",
               {"goal": int(goal), "robot": robot, "priority": 0, "arm_actions": 0})


def submit_retry(host, robot, goal, tries=12):
    """robot_busy(직전 task 완료 대기 중)면 잠깐 뒤 재시도."""
    for _ in range(tries):
        r = submit(host, robot, goal)
        if r.get("accepted"):
            return r
        if r.get("reason") not in ("robot_busy", "fleet_unavailable", "timeout"):
            return r   # bad_goal / insufficient_battery 등은 재시도 무의미
        time.sleep(0.5)
    return r


def robots(host):
    return sorted(state(host)["robots"].keys())


def corridor_ends(host):
    """상단 통로(y 최대 클러스터)의 좌/우 끝 정점 인덱스를 자동감지. 실패 시 (0, 3)."""
    vs = state(host)["vertices"]
    if len(vs) < 2:
        return 0, min(3, len(vs) - 1)
    ymax = max(p[1] for p in vs)
    top = [i for i, p in enumerate(vs) if abs(p[1] - ymax) < 1.0]
    if len(top) < 2:
        return 0, 3
    left = min(top, key=lambda i: vs[i][0])
    right = max(top, key=lambda i: vs[i][0])
    return left, right


def wait_arrival(host, robot, goal, timeout=45):
    vs = state(host)["vertices"]
    gx, gy = vs[goal][0], vs[goal][1]
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = state(host)["robots"].get(robot)
        if r and math.hypot(r["x"] - gx, r["y"] - gy) < ARRIVE:
            time.sleep(0.8)   # FMS 가 COMPLETED 처리(busy 해제)할 여유
            return True
        time.sleep(POLL)
    print(f"  ⚠ {robot} 가 v{goal} 도착 확인 실패(타임아웃) — 계속 진행")
    return False


# ── 장면 ──
def reset(host):
    for r in robots(host):
        set_mode(host, r, "IDLE")
    print("· 전원 IDLE (패트롤 정지)")


def park_extras(host, keep, node=PARK):
    """keep 목록 밖의 로봇을 코너로 치워 경로 간섭을 없앤다."""
    extras = [r for r in robots(host) if r not in keep]
    for r in extras:
        submit_retry(host, r, node)
    for r in extras:
        wait_arrival(host, r, node)
    if extras:
        print(f"· 무관 로봇 {extras} → v{node} 대기")


def scene_demo(host, boss, other, a, b):
    """head-on: 상단 복도에서 두 로봇이 마주침 → 낮은 우선순위(other)가 아래로 우회.
    other 의 우회 경로가 v5→v4 를 지나므로, '점유+양보+우회'가 한 장면에 다 나온다."""
    print(f"[DEMO] head-on · 보스={boss}(CHARGE·안 비킴) · 상대={other}(양보·우회) · 복도 v{a}↔v{b}")
    reset(host)
    time.sleep(0.6)
    park_extras(host, keep=[boss, other])
    print(f"· 배치: {boss}→v{a}, {other}→v{b} (복도 양 끝)")
    submit_retry(host, boss, a)
    submit_retry(host, other, b)
    wait_arrival(host, boss, a)
    wait_arrival(host, other, b)
    print("· 양 끝 배치 완료 — 잠시 후 교차")
    time.sleep(1.2)
    print(f"· 교차: {boss}: v{a}→v{b} [CHARGE 최우선], {other}: v{b}→v{a}")
    submit_retry(host, boss, b)
    set_mode(host, boss, "CHARGE")          # 보스: 최상위 tier → 교착 시 안 비킴
    time.sleep(0.4)                         # 보스 우선순위 확정 후 상대 투입
    submit_retry(host, other, a)
    print(f"→ 화면 확인: 상단서 마주침 → {other} 가 아래(v5→v4)로 우회, {boss} 는 직진 통과")


def scene_obstacle(host, boss, other, a, b, obstacle=6):
    """정지 장애물 우회: boss 를 통과 노드 v{obstacle}에 STOP 으로 세워둠(붙박이 점유)
    → 그 노드를 지나야 하는 other 가 blocked_by_stopped 로 우회.
    '누가 노드 붙잡고 있어서 내가 돌아간다'는 그림 그대로. 기본 v6(통과 노드)."""
    # v6 을 지나야만 하는 최단경로: v5 → v7 (v5-v6-v7 이 유일 최단). 막히면 위로 크게 우회.
    start, goal = 5, 7
    print(f"[OBSTACLE] {boss} 가 v{obstacle} STOP 점유 → {other}: v{start}→v{goal} 우회")
    reset(host)
    time.sleep(0.6)
    park_extras(host, keep=[boss, other], node=a)   # 남는 로봇은 좌상단으로
    submit_retry(host, boss, obstacle)              # 장애물 로봇을 v6 으로
    wait_arrival(host, boss, obstacle)
    set_mode(host, boss, "STOP")                    # v6 을 영구 장애물로 점유
    print(f"· {boss} v{obstacle} STOP (붙박이 장애물)")
    submit_retry(host, other, start)                # 통과 로봇을 v5 로 배치
    wait_arrival(host, other, start)
    time.sleep(0.8)
    submit_retry(host, other, goal)                 # v5→v7 시도 → v6 막힘 → 우회
    print(f"→ 화면 확인: {other} 가 v{obstacle}(STOP {boss})을 피해 위로 우회")


def scene_queue(host, boss, other, a, b):
    """단순 양보: 같은 목적지로 앞뒤로 보내 뒤 로봇이 ⏸ 양보 대기하는 정지 장면."""
    print(f"[QUEUE] {boss} 선행 · {other} 뒤에서 ⏸ 양보 대기 · 목적지 v{b}")
    reset(host)
    time.sleep(0.6)
    submit_retry(host, boss, a)             # 보스를 먼저 복도 안쪽으로
    wait_arrival(host, boss, a)
    submit_retry(host, boss, b)             # 보스가 통로를 점유하며 이동
    time.sleep(1.5)
    submit_retry(host, other, b)            # 상대가 뒤따르다 점유 노드에서 대기
    print(f"→ 화면 확인: {other} 가 {boss} 뒤 노드에서 멈춰 '양보 대기'(펄스)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", nargs="?", default="demo",
                    choices=["demo", "obstacle", "queue", "reset"])
    ap.add_argument("--host", default="localhost:8001")
    ap.add_argument("--boss", default=None)
    ap.add_argument("--a", type=int, default=None)
    ap.add_argument("--b", type=int, default=None)
    ap.add_argument("--loop", action="store_true")
    args = ap.parse_args()

    try:
        names = robots(args.host)
    except (urllib.error.URLError, OSError) as e:
        print(f"콘솔({args.host}) 연결 실패: {e}\n→ run_sim.sh 로 스택을 먼저 띄우세요.")
        return 1
    if args.scene != "reset" and len(names) < 2:
        print(f"로봇이 2대 이상 필요합니다. 현재: {names}")
        return 1

    boss = args.boss or (names[0] if names else "pinky1")
    other = next((n for n in names if n != boss), None)
    a, b = corridor_ends(args.host)
    if args.a is not None:
        a = args.a
    if args.b is not None:
        b = args.b

    if args.scene == "reset":
        reset(args.host)
        return 0

    scene = {"demo": scene_demo, "obstacle": scene_obstacle,
             "queue": scene_queue}[args.scene]
    while True:
        scene(args.host, boss, other, a, b)
        if not args.loop:
            break
        print("\n— 다음 테이크까지 8초 —\n")
        time.sleep(8)
    return 0


if __name__ == "__main__":
    sys.exit(main())
