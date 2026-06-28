from fastapi.testclient import TestClient
from aba_service.main import app, get_bridge


class FakeBridge:
    def submit_task(self, task_type, pickup, dropoff, requester):
        return {"accepted": True, "task_id": "T-1", "reason": ""}


def test_post_task_accepted():
    app.dependency_overrides[get_bridge] = lambda: FakeBridge()
    client = TestClient(app)
    r = client.post("/tasks", json={
        "task_type": "delivery", "pickup": "s", "dropoff": "d", "requester": "m"})
    assert r.status_code == 200
    assert r.json()["task_id"] == "T-1"
    app.dependency_overrides.clear()


def test_post_task_rejected_is_503():
    class Rej:
        def submit_task(self, *a, **k):
            return {"accepted": False, "task_id": "", "reason": "no_robot_available"}
    app.dependency_overrides[get_bridge] = lambda: Rej()
    client = TestClient(app)
    r = client.post("/tasks", json={
        "task_type": "delivery", "pickup": "s", "dropoff": "d", "requester": "m"})
    assert r.status_code == 503
    assert r.json()["detail"] == "no_robot_available"
    app.dependency_overrides.clear()
