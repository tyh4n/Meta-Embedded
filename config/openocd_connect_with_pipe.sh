#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "$0")/.." && pwd)"
openocd_bin="${OPENOCD_BIN:-$(command -v openocd || true)}"
[[ -n "$openocd_bin" ]] || { echo "OpenOCD not found; install it or set OPENOCD_BIN." >&2; exit 1; }

exec "$openocd_bin" \
    -l "$project_root/openocd.log" \
    -c "gdb_port pipe" \
    -f "$project_root/config/openocd_stm32f4.cfg"
