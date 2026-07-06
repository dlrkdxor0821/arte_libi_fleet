#!/usr/bin/env python3
"""robot_markers.py — /robot_state → /robot_markers (rviz MarkerArray, frame=map).

slotcar 로봇은 TF/RobotModel 이 없어 rviz 에 안 보인다. /robot_state 의 위치를
색깔 실린더 + 이름 텍스트 마커로 변환해 rviz 에서 3대가 보이게 한다.
추가로 /robot_path_requests 를 구독해 로봇이 지금 향하는 구간(route)과
지나온 자취(trail)를 선으로 그려 이동 경로가 잘 보이게 한다.

사용: python3 scripts/sim/robot_markers.py  (rmf_fleet_msgs 소스 필요)
"""
from collections import deque

import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import RobotState, PathRequest
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

COLORS = {
    "pinky1": (1.0, 0.36, 0.38),
    "pinky2": (0.21, 0.85, 0.54),
    "pinky3": (0.31, 0.64, 1.0),
}

TRAIL_MAX = 80        # 자취로 남길 최근 위치 개수
TRAIL_MIN_STEP = 0.08  # 이 거리(m) 이상 움직여야 자취 점 추가


class RobotMarkers(Node):
    def __init__(self):
        super().__init__("robot_markers")
        self.robots = {}   # name -> (x, y)
        self.routes = {}   # name -> [(x, y), ...]  현재 향하는 경로(PathRequest)
        self.trails = {}   # name -> deque[(x, y)]   지나온 자취
        self.order = {}    # name -> 안정적 인덱스(마커 id 용)
        self.create_subscription(RobotState, "/robot_state", self._on_state, 10)
        self.create_subscription(PathRequest, "/robot_path_requests", self._on_path, 10)
        self.pub = self.create_publisher(MarkerArray, "/robot_markers", 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info("robot_markers: /robot_state(+path) → /robot_markers (rviz)")

    def _idx(self, name):
        if name not in self.order:
            self.order[name] = len(self.order)
        return self.order[name]

    def _on_state(self, m):
        x, y = m.location.x, m.location.y
        self.robots[m.name] = (x, y)
        tr = self.trails.setdefault(m.name, deque(maxlen=TRAIL_MAX))
        if not tr or (abs(tr[-1][0] - x) + abs(tr[-1][1] - y)) > TRAIL_MIN_STEP:
            tr.append((x, y))

    def _on_path(self, m):
        self.routes[m.name] = [(p.x, p.y) for p in m.path]

    def _tick(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)   # 매 틱 정리 후 재발행 → 잔상 방지

        for name, (x, y) in self.robots.items():
            r, g, b = COLORS.get(name, (0.8, 0.4, 1.0))
            k = self._idx(name)

            # 지나온 자취(trail) — 얇고 옅은 선
            tr = self.trails.get(name)
            if tr and len(tr) >= 2:
                arr.markers.append(self._line(
                    "trail", k, list(tr), r, g, b, width=0.05, alpha=0.45, z=0.04))

            # 현재 향하는 경로(route) — 굵고 밝은 선
            rt = self.routes.get(name)
            if rt and len(rt) >= 2:
                arr.markers.append(self._line(
                    "route", k, rt, r, g, b, width=0.09, alpha=0.9, z=0.06))

            # 몸통 실린더
            body = Marker()
            body.header.frame_id = "map"
            body.ns = "robot"; body.id = k
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = float(x)
            body.pose.position.y = float(y)
            body.pose.position.z = 0.15
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = 0.26
            body.scale.z = 0.3
            body.color.r, body.color.g, body.color.b, body.color.a = r, g, b, 0.95
            arr.markers.append(body)

            # 이름 텍스트
            txt = Marker()
            txt.header.frame_id = "map"
            txt.ns = "label"; txt.id = k
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(x)
            txt.pose.position.y = float(y)
            txt.pose.position.z = 0.55
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.28
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = r, g, b, 1.0
            txt.text = name
            arr.markers.append(txt)
        self.pub.publish(arr)

    @staticmethod
    def _line(ns, mid, pts, r, g, b, width, alpha, z):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns; m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, alpha
        m.points = [Point(x=float(px), y=float(py), z=float(z)) for px, py in pts]
        return m


def main():
    rclpy.init()
    rclpy.spin(RobotMarkers())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
