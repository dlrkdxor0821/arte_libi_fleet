#!/usr/bin/env bash
# libi 테스트 콘솔(FastAPI) 실행 — 포트 8001
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source ~/open-rmf-test/rmf_ws/install/setup.bash       # rmf_fleet_msgs
source "$REPO/install/setup.bash"                       # libi_fleet_msgs
cd "$REPO/service/aba_service"
exec python3 -m uvicorn aba_service.console:app --host 0.0.0.0 --port 8001
