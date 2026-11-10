#!/usr/bin/env bash
set -e
TOOL_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MINDVISION_ENV_FILE="${MINDVISION_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/shark-radar/mindvision.env}"
if [ -r "$MINDVISION_ENV_FILE" ]; then
    source "$MINDVISION_ENV_FILE"
fi
if [ -n "${MINDVISION_SDK_ROOT:-}" ] && [ -d "$MINDVISION_SDK_ROOT" ]; then
    sdk_python=$(find "$MINDVISION_SDK_ROOT" -type f -name mvsdk.py -print -quit)
    sdk_library=$(find "$MINDVISION_SDK_ROOT" -type f -iname '*mvsdk*.so*' -print -quit)
    if [ -n "$sdk_python" ]; then
        export PYTHONPATH="$(dirname "$sdk_python")${PYTHONPATH:+:$PYTHONPATH}"
    fi
    if [ -n "$sdk_library" ]; then
        export LD_LIBRARY_PATH="$(dirname "$sdk_library")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
fi
export PYTHONPATH="$TOOL_ROOT/..${PYTHONPATH:+:$PYTHONPATH}"
CALIBRATION_PYTHON="$TOOL_ROOT/../vision/.venv/bin/python"
if [ ! -x "$CALIBRATION_PYTHON" ]; then
    CALIBRATION_PYTHON=$(command -v python3)
fi
exec "$CALIBRATION_PYTHON" -m tool.intrinsic_calibration "$@"
