#!/bin/bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
launch_args=("$@")
set --
source "$project_dir/devel/setup.bash"
exec roslaunch lidar_rgbd_capture sensors.launch "${launch_args[@]}"
