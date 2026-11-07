#!/usr/bin/env bash
set -eu

PROJECT_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python"

# Desktop autostart does not inherit shell profile exports; allow a small
# per-user environment file to define MINDVISION_SDK_ROOT for that entrypoint.
MINDVISION_ENV_FILE="${MINDVISION_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/shark-radar/mindvision.env}"
if [ -r "$MINDVISION_ENV_FILE" ]; then
    set +u
    . "$MINDVISION_ENV_FILE"
    set -u
fi

# radio_ros 是比赛默认裁判通信方式。桌面启动不会读取交互式 shell，
# 因此在启动台及其 QProcess 子进程继承环境前显式加载 ROS2 Humble。
if [ -r /opt/ros/humble/setup.sh ]; then
    # shellcheck disable=SC1091
    # ROS Humble's generated setup scripts read AMENT_TRACE_SETUP_FILES and
    # several other optional variables without nounset-safe expansion.
    # Temporarily relax `set -u` only while importing that external script.
    set +u
    . /opt/ros/humble/setup.sh
    set -u
fi

# Desktop autostart does not source an interactive shell. Discover the common
# Hikrobot MVS install location before Python imports the vendor binding.
if [ -z "${MVCAM_COMMON_RUNENV:-}" ]; then
    for mvs_root in /opt/MVS/lib /opt/MVS /usr/local/MVS/lib; do
        if [ -f "$mvs_root/64/libMvCameraControl.so" ]; then
            export MVCAM_COMMON_RUNENV="$mvs_root"
            break
        fi
    done
fi
if [ -n "${MVCAM_COMMON_RUNENV:-}" ] && [ -d "$MVCAM_COMMON_RUNENV/64" ]; then
    export LD_LIBRARY_PATH="$MVCAM_COMMON_RUNENV/64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

# MindVision SDK 安装目录由操作者设置，避免把厂商版本/安装位置写死。
if [ -n "${MINDVISION_SDK_ROOT:-}" ] && [ -d "$MINDVISION_SDK_ROOT" ]; then
    mindvision_python_file=$(find "$MINDVISION_SDK_ROOT" -type f -name mvsdk.py -print -quit)
    if [ -n "$mindvision_python_file" ]; then
        mindvision_python_dir=$(dirname "$mindvision_python_file")
        export PYTHONPATH="$mindvision_python_dir${PYTHONPATH:+:$PYTHONPATH}"
    fi

    mindvision_library=$(find "$MINDVISION_SDK_ROOT" -type f \
        \( -iname '*mvsdk*.so*' -o -iname '*mvcam*.so*' \) -print -quit)
    if [ -n "$mindvision_library" ]; then
        mindvision_library_dir=$(dirname "$mindvision_library")
        export LD_LIBRARY_PATH="$mindvision_library_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
fi

if [ ! -x "$PYTHON_BIN" ]; then
    PYTHON_BIN=$(command -v python3 || true)
fi

if [ -z "$PYTHON_BIN" ]; then
    echo "未找到 Python 3，请先安装 Python 或创建项目 .venv。" >&2
    exit 1
fi

cd "$PROJECT_ROOT"
exec "$PYTHON_BIN" -u launcher.py "$@"
