#!/usr/bin/env bash
# 테스트용 task 제출. 사용: scripts/submit_task.sh [type] [pickup] [dropoff] [requester]
curl -s -X POST http://localhost:8000/tasks \
  -H 'Content-Type: application/json' \
  -d "{\"task_type\":\"${1:-delivery}\",\"pickup\":\"${2:-}\",\"dropoff\":\"${3:-}\",\"requester\":\"${4:-}\"}"
echo
