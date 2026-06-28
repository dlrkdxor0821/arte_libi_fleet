#!/usr/bin/env python3
"""robot_markers.py — /robot_state → /robot_markers (rviz MarkerArray, frame=map).

slotcar 로봇은 TF/RobotModel 이 없어 rviz 에 안 보인다. /robot_state 의 위치를
색깔 실린더 + 이름 텍스트 마커로 변환해 rviz 에서 3대가 보이게 한다.

사용: python3 scripts/sim/robot_markers.py  (rmf_fleet_msgs 소스 필요)
"""
import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import RobotState
from visualization_msgs.msg import Marker, MarkerArray

COLORS = {
    "pinky1": (1.0, 0.36, 0.38),
    "pinky2": (0.21, 0.85, 0.54),
    "pinky3": (0.31, 0.64, 1.0),
}


class RobotMarkers(Node):
    def __init__(self):
        super().__init__("robot_markers")
        self.robots = {}
        self.create_subscription(RobotState, "/robot_state", self._on_state, 10)
        self.pub = self.create_publisher(MarkerArray, "/robot_markers", 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info("robot_markers: /robot_state → /robot_markers (rviz)")

    def _on_state(self, m):
        self.robots[m.name] = (m.location.x, m.location.y)

    def _tick(self):
        arr = MarkerArray()
        i = 0
        for name, (x, y) in self.robots.items():
            r, g, b = COLORS.get(name, (0.8, 0.4, 1.0))
            body = Marker()
            body.header.frame_id = "map"
            body.ns = "robot"
            body.id = i; i += 1
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

            txt = Marker()
            txt.header.frame_id = "map"
            txt.ns = "label"
            txt.id = i; i += 1
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


def main():
    rclpy.init()
    rclpy.spin(RobotMarkers())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
