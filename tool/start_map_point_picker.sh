#!/usr/bin/env bash
set -e
TOOL_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec python3 "$TOOL_ROOT/map_point_picker.py" "$@"
