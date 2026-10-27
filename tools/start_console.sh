#!/bin/zsh
# Launch the live hardware-probe console and open it in the browser.
# Extra arguments go to probe_dashboard.py, e.g. --bluetooth or --port /dev/cu.X
set -eu
cd "$(dirname "$0")/.."
if [[ ! -x .venv-dashboard/bin/python ]]; then
  python3 -m venv .venv-dashboard
  .venv-dashboard/bin/python -m pip install -r tools/requirements.txt
fi
(sleep 1.5 && open "http://127.0.0.1:8766") &
exec .venv-dashboard/bin/python tools/probe_dashboard.py "$@"
