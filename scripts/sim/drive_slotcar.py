#!/usr/bin/env python3
"""drive_slotcar.py — slotcar 에 PathRequest 1발 발행 + 이동 검증 (고유 task_id).

open-rmf-practice slotcar_drive.py 의 arte_libi_fleet 판. 차이: task_id 를 매 호출
고유하게 생성(시간 기반) → slotcar dedup 에 안 걸려 반복 주행 가능.

사용: python3 scripts/sim/drive_slotcar.py <x> <y> [--robot pinky] [--fleet libi]
      --ros-args -p use_sim_time:=true
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rmf_fleet_msgs.msg import PathRequest, Location, RobotState


class DriveSlotcar(Node):
    def __init__(self, a):
        super().__init__("drive_slotcar")
        self.a = a
        self.state = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PathRequest, "/robot_path_requests", qos)
        self.create_subscription(RobotState, "/robot_state", self._on_state, 10)

    def _on_state(self, msg):
        if msg.name == self.a.robot:
            self.state = msg

    def wait_state(self, timeout=10.0):
        t0 = time.time()
        while self.state is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.state

    def go(self):
        s = self.wait_state()
        if s is None:
            print("FAIL: /robot_state 수신 못함")
            return 1
        x0, y0 = s.location.x, s.location.y
        req = PathRequest()
        req.fleet_name = self.a.fleet
        req.robot_name = self.a.robot
        req.task_id = f"drive-{int(time.time()*1000)}"  # 고유 task_id
        p0 = Location(); p0.x = x0; p0.y = y0; p0.level_name = self.a.level
        p1 = Location(); p1.x = self.a.x; p1.y = self.a.y; p1.level_name = self.a.level
        req.path = [p0, p1]
        # 첫 메시지 유실(pub/sub 디스커버리 레이스) 방지: slotcar 구독 매칭까지 대기
        t_match = time.time()
        while self.pub.get_subscription_count() < 1 and time.time() - t_match < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.pub.publish(req)
        dist = math.hypot(self.a.x - x0, self.a.y - y0)
        print(f"PathRequest({req.task_id}) ({x0:.2f},{y0:.2f})->({self.a.x:.2f},{self.a.y:.2f}) 직선거리 {dist:.2f}m")
        moved = 0.0
        last = (x0, y0)
        t0 = time.time()
        while time.time() - t0 < 12.0:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.state:
                cx, cy = self.state.location.x, self.state.location.y
                moved += math.hypot(cx - last[0], cy - last[1])
                last = (cx, cy)
        print(f"결과: 총 이동거리 {moved:.2f}m, 최종 pos=({last[0]:.2f},{last[1]:.2f})")
        return 0 if moved > 0.3 else 2


def main():
    p = argparse.ArgumentParser()
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("--robot", default="pinky")
    p.add_argument("--fleet", default="libi")
    p.add_argument("--level", default="L1")
    args, _ = p.parse_known_args()
    rclpy.init()
    rc = DriveSlotcar(args).go()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
