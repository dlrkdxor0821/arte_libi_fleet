#!/usr/bin/env bash
# aba_service (FastAPI) 실행 — UI HTTP 입구
set -e
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/../install/setup.bash"
cd "$(dirname "$0")/../service/aba_service"
exec python3 -m uvicorn aba_service.main:app --host 0.0.0.0 --port 8000
