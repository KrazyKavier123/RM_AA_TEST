#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
if [[ ! -f install/setup.bash ]]; then echo "Run bash tools/build.sh first" >&2; exit 1; fi
set +u
source install/setup.bash
set -u
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
