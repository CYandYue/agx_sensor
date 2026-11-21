#!/bin/bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${AGILEX_LIO_WS:-/home/agilex/agilex_ws/lio_ws}/devel/setup.bash"
cd "$project_dir"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release "$@"
