"""aba_service — 중앙관제 입구 (FastAPI).

UI 의 HTTP 명령을 받아 ROS 브리지를 통해 libi_fleet 에 제출한다.
가용 로봇 없음(거절) → HTTP 503. (무큐·거절 정책)
"""
from fastapi import FastAPI, Depends, HTTPException
from pydantic import BaseModel

app = FastAPI(title="aba_service")

_bridge = None


def get_bridge():
    """ROS 브리지 lazy 초기화 (테스트에서 override)."""
    global _bridge
    if _bridge is None:
        import rclpy
        from .ros_bridge import RosBridge
        rclpy.init()
        _bridge = RosBridge()
    return _bridge


class TaskReq(BaseModel):
    task_type: str
    pickup: str = ""
    dropoff: str = ""
    requester: str = ""


@app.post("/tasks")
def post_task(t: TaskReq, bridge=Depends(get_bridge)):
    res = bridge.submit_task(t.task_type, t.pickup, t.dropoff, t.requester)
    if not res["accepted"]:
        raise HTTPException(status_code=503, detail=res["reason"] or "no_robot_available")
    return {"accepted": True, "task_id": res["task_id"]}
