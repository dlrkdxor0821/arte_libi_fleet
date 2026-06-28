#!/usr/bin/env python3
"""Task 3 검증용 임시 클라이언트 — ros2 action CLI 부재 환경 대체.

/robot1/navigate 에 goal 1발 보내고 result.arrived 를 확인한다.
"""
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from libi_fleet_msgs.action import Navigate


def main():
    rclpy.init()
    node = Node("verify_drive")
    cli = ActionClient(node, Navigate, "/robot1/navigate")
    if not cli.wait_for_server(timeout_sec=10.0):
        print("FAIL: navigate action server 없음")
        sys.exit(1)
    goal = Navigate.Goal()
    goal.target.position.x = 1.0
    send_fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_fut, timeout_sec=5.0)
    gh = send_fut.result()
    if gh is None or not gh.accepted:
        print("FAIL: goal 거부됨")
        sys.exit(1)
    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=5.0)
    result = res_fut.result().result
    print(f"navigate result: arrived={result.arrived} error='{result.error}'")
    rclpy.shutdown()
    sys.exit(0 if result.arrived else 1)


if __name__ == "__main__":
    main()
