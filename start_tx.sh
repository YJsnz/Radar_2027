#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TX_START="$ROOT_DIR/radio/apps/lab_tx/start.sh"

if (($#)) && [[ "$1" == "--help" || "$1" == "-h" ]]; then
  cat <<'EOF'
用法: ./start_tx.sh [--host HOST] [--port PORT] [--no-browser]

启动受保护的实验室 TX Panel。该脚本不会自动发射 RF；真实发射仍需在
Panel 中操作，并遵守 RM_RADIO_LAB_TX_ANTENNA_CONFIRM 安全确认。
EOF
  exit 0
fi

[[ -f "$TX_START" ]] || { echo "缺少 TX 启动器: $TX_START" >&2; exit 1; }

echo "[一键 TX] 启动实验室 TX Panel（不会自动发射 RF）"
exec bash "$TX_START" "$@"
