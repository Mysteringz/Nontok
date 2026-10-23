#!/bin/zsh
# Run from any directory; keep the virtual environment in this repository.
set -eu
cd "$(dirname "$0")/.."
if [[ ! -x .venv-dashboard/bin/python ]]; then
  python3 -m venv .venv-dashboard
fi
.venv-dashboard/bin/python -m pip install -r tools/requirements.txt
exec .venv-dashboard/bin/python dashboard/server.py "$@"
