#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RADIO_START="${MATCH_RADIO_START:-$ROOT_DIR/radio/apps/match_rx/start.sh}"
VISION_DIR="$ROOT_DIR/vision"
VISION_START="${MATCH_VISION_START:-$VISION_DIR/scripts/start_launcher.sh}"
RESTART_DELAY_SEC="${MATCH_RESTART_DELAY_SEC:-2}"
RADIO_ARGS=("$@")

# One stable identifier groups radio, radar-integration and vision artifacts.
# It is intentionally retained when only one supervised side is restarted, so
# a missing/restarted component cannot shift an independent gameN counter.
export SHARK_MATCH_RUN_ID="${SHARK_MATCH_RUN_ID:-match_$(date +%Y%m%d_%H%M%S)_$$}"
if [[ ! "$SHARK_MATCH_RUN_ID" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] \
    || ((${#SHARK_MATCH_RUN_ID} > 88)); then
  echo "SHARK_MATCH_RUN_ID 必须为不超过 88 字符的安全标识，当前值: $SHARK_MATCH_RUN_ID" >&2
  exit 2
fi

usage() {
  cat <<'EOF'
用法: ./start_match.sh [无线电启动参数]

同时启动：
  1. 比赛无线电 RX、裁判串口、融合节点和 Dashboard
  2. 视觉雷达比赛启动台（可配置、标定并启动比赛主程序）

常用参数会原样传给无线电启动器，例如：
  --panel-only      仅启动 Dashboard（不启动 RX/裁判串口；仍会打开视觉启动台）
  --no-browser       不自动打开 RX Dashboard
  --headless         不打开 GNU Radio 原生窗口
  --native-gui       打开 GNU Radio 原生诊断窗口

无线电或视觉任一侧掉线/退出时，只会自动重启故障侧，另一侧继续运行。
按 Ctrl+C 会同时关闭无线电和视觉雷达。
EOF
}

if (($#)) && [[ "$1" == "--help" || "$1" == "-h" ]]; then
  usage
  exit 0
fi

[[ -f "$RADIO_START" ]] || { echo "缺少无线电启动器: $RADIO_START" >&2; exit 1; }
[[ -f "$VISION_START" ]] || { echo "缺少视觉雷达启动台入口: $VISION_START" >&2; exit 1; }
[[ "$RESTART_DELAY_SEC" =~ ^[0-9]+$ ]] || {
  echo "MATCH_RESTART_DELAY_SEC 必须是非负整数，当前值: $RESTART_DELAY_SEC" >&2
  exit 2
}

radio_pid=""
vision_pid=""
radio_restart_at=0
vision_restart_at=0

stop_child() {
  local pid="$1"
  local group="$2"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0
  if [[ "$group" == "true" ]]; then
    kill -TERM -- "-$pid" >/dev/null 2>&1 || {
      kill -0 "$pid" >/dev/null 2>&1 && kill -TERM "$pid" >/dev/null 2>&1
    } || true
  else
    kill -0 "$pid" >/dev/null 2>&1 || return 0
    kill -TERM "$pid" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM HUP
  set +e
  stop_child "$vision_pid" true
  stop_child "$radio_pid" false

  local deadline=$((SECONDS + 10))
  while ((SECONDS < deadline)); do
    local running=false
    [[ "$vision_pid" =~ ^[0-9]+$ ]] && kill -0 -- "-$vision_pid" >/dev/null 2>&1 && running=true
    [[ "$radio_pid" =~ ^[0-9]+$ ]] && kill -0 "$radio_pid" >/dev/null 2>&1 && running=true
    [[ "$running" == "false" ]] && break
    sleep 0.1
  done

  if [[ "$vision_pid" =~ ^[0-9]+$ ]] && kill -0 -- "-$vision_pid" >/dev/null 2>&1; then
    kill -KILL -- "-$vision_pid" >/dev/null 2>&1 || true
  fi
  if [[ "$radio_pid" =~ ^[0-9]+$ ]] && kill -0 "$radio_pid" >/dev/null 2>&1; then
    kill -KILL "$radio_pid" >/dev/null 2>&1 || true
  fi
  [[ "$vision_pid" =~ ^[0-9]+$ ]] && wait "$vision_pid" 2>/dev/null || true
  [[ "$radio_pid" =~ ^[0-9]+$ ]] && wait "$radio_pid" 2>/dev/null || true
  exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

start_radio() {
  echo "[一键比赛] 启动无线电比赛链路，match_run_id=$SHARK_MATCH_RUN_ID"
  setsid bash "$RADIO_START" "${RADIO_ARGS[@]}" &
  radio_pid=$!
  echo "[一键比赛] 无线电 PID=$radio_pid"
}

start_vision() {
  echo "[一键比赛] 启动视觉雷达比赛启动台: $VISION_START，match_run_id=$SHARK_MATCH_RUN_ID"
  setsid bash "$VISION_START" &
  vision_pid=$!
  echo "[一键比赛] 视觉 PID=$vision_pid"
}

start_radio
start_vision
echo "[一键比赛] 两侧独立监督；单侧退出只重启故障侧，按 Ctrl+C 可全部停止。"

while true; do
  if [[ -n "$radio_pid" ]] && ! kill -0 "$radio_pid" >/dev/null 2>&1; then
    radio_status=0
    wait "$radio_pid" || radio_status=$?
    radio_pid=""
    radio_restart_at=$((SECONDS + RESTART_DELAY_SEC))
    echo "[一键比赛][WARN] 无线电已退出，状态=$radio_status；视觉继续运行，${RESTART_DELAY_SEC} 秒后重启无线电。" >&2
  fi
  if [[ -n "$vision_pid" ]] && ! kill -0 "$vision_pid" >/dev/null 2>&1; then
    vision_status=0
    wait "$vision_pid" || vision_status=$?
    vision_pid=""
    vision_restart_at=$((SECONDS + RESTART_DELAY_SEC))
    echo "[一键比赛][WARN] 视觉雷达已退出，状态=$vision_status；无线电继续运行，${RESTART_DELAY_SEC} 秒后重启视觉。" >&2
  fi
  if [[ -z "$radio_pid" ]] && ((SECONDS >= radio_restart_at)); then
    start_radio
  fi
  if [[ -z "$vision_pid" ]] && ((SECONDS >= vision_restart_at)); then
    start_vision
  fi
  sleep 1
done
