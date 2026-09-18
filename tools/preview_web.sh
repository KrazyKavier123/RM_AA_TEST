#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$workspace_dir/src/rm_aa_bringup/scripts/debug_web.py" --demo "$@"
