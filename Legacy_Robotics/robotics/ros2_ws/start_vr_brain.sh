#!/bin/bash

trap "kill 0" EXIT

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "=== [0/3] Устранение конфликтов RViz ==="
killall -9 joint_state_publisher 2>/dev/null || true

echo "=== [1/3] Активация датчиков суставов ==="
ros2 run controller_manager spawner joint_state_broadcaster &
sleep 2

echo "=== [2/3] Активация моторов рук ==="
ros2 run controller_manager spawner arms_controller &
sleep 2

echo "=== [3/3] Включение VR-моста ==="
python3 "$workspace_dir/vr_bridge.py" &

wait
