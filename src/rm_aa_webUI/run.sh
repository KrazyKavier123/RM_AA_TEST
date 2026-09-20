#!/usr/bin/env bash
set -euo pipefail
ui_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$ui_dir/../.." && pwd)"
python_bin="$workspace_dir/.venv/bin/python"
if [[ ! -x "$python_bin" ]]; then python_bin=python3; fi
exec "$python_bin" "$ui_dir/backend/server.py" --workspace "$workspace_dir" "$@"
