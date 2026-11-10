#!/usr/bin/env bash
set -e
TOOL_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ -r /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
export PYTHONPATH="$TOOL_ROOT/../radio/src/rm_radio_ros:$TOOL_ROOT/..${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m tool.armor_simulator "$@"
