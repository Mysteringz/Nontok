#!/bin/zsh
# Launch the Mac car controller (for the professor's sketch) and open it in the browser.
# Extra arguments go to car_controller.py, e.g. --bt /dev/cu.DIVA --usb /dev/cu.usbserial-X
set -eu
cd "$(dirname "$0")/.."
if [[ ! -x .venv-dashboard/bin/python ]]; then
  python3 -m venv .venv-dashboard
  .venv-dashboard/bin/python -m pip install -r tools/requirements.txt
fi
(sleep 1.5 && open "http://127.0.0.1:8767") &
exec .venv-dashboard/bin/python tools/car_controller.py "$@"
