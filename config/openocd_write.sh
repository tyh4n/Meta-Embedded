#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 <firmware.elf>" >&2
    exit 2
}

[[ $# -eq 1 ]] || usage
elf="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
[[ -f "$elf" ]] || { echo "Firmware not found: $elf" >&2; exit 1; }

project_root="$(cd "$(dirname "$0")/.." && pwd)"
openocd_bin="${OPENOCD_BIN:-$(command -v openocd || true)}"
[[ -n "$openocd_bin" ]] || { echo "OpenOCD not found; install it or set OPENOCD_BIN." >&2; exit 1; }

exec "$openocd_bin" \
    -f "$project_root/config/openocd_stm32f4.cfg" \
    -c "program \"$elf\" verify reset exit"
