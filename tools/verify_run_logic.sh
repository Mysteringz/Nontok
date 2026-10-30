#!/bin/zsh
# Host check of the on-Mega tournament controller against a simulated arena.
# Usage: tools/verify_run_logic.sh [v|q] [arc_deg] [aim_deg] [run]   (v: print that run's events)
set -eu
cd "$(dirname "$0")/.."
out="${TMPDIR:-/tmp}/nontok-run-logic-test"
c++ -std=c++11 -O2 -Wall -Wextra -o "$out" tests/run_logic_test.cpp firmware/hardware_probe_mega/src/run_logic.cpp
"$out" "$@"
