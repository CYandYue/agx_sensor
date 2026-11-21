#!/bin/bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
capture_args=("$@")
set --
source /opt/ros/noetic/setup.bash
exec /usr/bin/python3 "$project_dir/recorder/capture.py" "${capture_args[@]}"
