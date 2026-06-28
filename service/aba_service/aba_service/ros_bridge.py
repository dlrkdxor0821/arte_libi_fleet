"""aba_service ↔ libi_fleet ROS2 브리지 (rclpy).

FastAPI 에서 들어온 task 요청을 /fms/submit_task 서비스로 넘기고
{accepted, task_id, reason} 를 반환한다. (Plan 1: 동기 호출)
"""
import rclpy
from rclpy.node import Node
from libi_fleet_msgs.srv import SubmitTask


class RosBridge(Node):
    def __init__(self):
        super().__init__("aba_service_bridge")
        self.cli = self.create_client(SubmitTask, "/fms/submit_task")

    def submit_task(self, task_type, pickup, dropoff, requester):
        if not self.cli.wait_for_service(timeout_sec=2.0):
            return {"accepted": False, "task_id": "", "reason": "fleet_unavailable"}
        req = SubmitTask.Request(
            task_type=task_type, pickup=pickup, dropoff=dropoff, requester=requester)
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if r is None:
            return {"accepted": False, "task_id": "", "reason": "fleet_timeout"}
        return {"accepted": r.accepted, "task_id": r.task_id, "reason": r.reason}
