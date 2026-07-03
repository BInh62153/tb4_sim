#!/usr/bin/env bash
# Chờ motion_control (Create3 base) lên rồi bật backup_only.
#
# Bug liên quan: Nav2 BT + RecoveryManager đều gọi BackUp; Create3 mặc định
# safety_override=none nên chặn lùi quá xa → "Reached backup limit!" → robot dính.
# backup_only chỉ tắt giới hạn lùi, vẫn giữ các safety khác (cliff, bump, ...).
#
# Node có thể là /motion_control hoặc /<namespace>/motion_control tùy launch.

set -euo pipefail
source /opt/ros/humble/setup.bash 2>/dev/null || true

SAFETY_VALUE="${MOTION_SAFETY_OVERRIDE:-backup_only}"
MAX_WAIT="${MOTION_SAFETY_WAIT_SEC:-120}"

find_motion_node() {
  ros2 node list 2>/dev/null | grep -E 'motion_control$' | head -1 | sed 's|^/||'
}

echo "[set_motion_safety] waiting for motion_control (max ${MAX_WAIT}s)..."
elapsed=0
NODE=""
while [ "$elapsed" -lt "$MAX_WAIT" ]; do
  NODE="$(find_motion_node || true)"
  if [ -n "$NODE" ] && ros2 param list "/${NODE}" 2>/dev/null | grep -q safety_override; then
    break
  fi
  sleep 2
  elapsed=$((elapsed + 2))
done

if [ -z "$NODE" ]; then
  echo "[set_motion_safety] WARN: motion_control not found — skip (sim may use different base stack)"
  exit 0
fi

if ros2 param set "/${NODE}" safety_override "$SAFETY_VALUE"; then
  echo "[set_motion_safety] OK: /${NODE} safety_override=${SAFETY_VALUE}"
else
  echo "[set_motion_safety] WARN: failed to set safety_override on /${NODE}"
  exit 0
fi
