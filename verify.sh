#!/bin/bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test_args=("$@")
set --
source "$project_dir/devel/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 -m unittest discover -s "$project_dir/tests" -p test_recorder.py -v
exec /usr/bin/python3 "$project_dir/tests/smoke_pipeline.py" "${test_args[@]}"
