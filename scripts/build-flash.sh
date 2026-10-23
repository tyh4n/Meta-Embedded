#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  scripts/build-flash.sh <target> [board] [debug|release]
  scripts/build-flash.sh --no-flash <target> [board] [debug|release]

Boards:
  2018a, rm_board_2018_a   RM Board 2018 A
  c, rm_board_c            RM Board C

Examples:
  scripts/build-flash.sh ut_blink 2018a
  scripts/build-flash.sh --no-flash STABILITY_SYSTEM c release
EOF
}

flash_enabled=1
if [[ ${1:-} == "--no-flash" ]]; then
    flash_enabled=0
    shift
fi

[[ $# -ge 1 ]] || { usage >&2; exit 2; }

target="$1"
board_arg="${2:-c}"
build_type="${3:-debug}"
project_root="$(cd "$(dirname "$0")/.." && pwd)"
board_normalized="$(printf '%s' "$board_arg" | tr '[:upper:]' '[:lower:]')"
build_type_normalized="$(printf '%s' "$build_type" | tr '[:upper:]' '[:lower:]')"

case "$board_normalized" in
    2018a|rm_board_2018_a)
        board_id="2018a"
        board_name="rm_board_2018_a"
        ;;
    c|rm_board_c)
        board_id="c"
        board_name="rm_board_c"
        ;;
    *)
        echo "Unknown board: $board_arg" >&2
        usage >&2
        exit 2
        ;;
esac

case "$build_type_normalized" in
    debug) build_type="debug" ;;
    release) build_type="release" ;;
    *)
        echo "Unknown build type: $build_type" >&2
        exit 2
        ;;
esac

preset="mac-arm64-${board_id}-${build_type}"
build_dir="$project_root/build/$preset"

echo "Target: ${target}"
echo "Board:  ${board_name}"
echo "Preset: ${preset}"

cmake --preset "$preset" -S "$project_root"
cmake --build "$build_dir" --target "$target" --parallel 8

elf="$build_dir/bin/${target}.elf"
[[ -f "$elf" ]] || { echo "ELF not found: $elf" >&2; exit 1; }
echo "ELF:    $elf"

if [[ $flash_enabled -eq 0 ]]; then
    exit 0
fi

openocd_bin="${OPENOCD_BIN:-$(command -v openocd || true)}"
[[ -n "$openocd_bin" ]] || {
    echo "OpenOCD not found. Run scripts/bootstrap-macos.sh or set OPENOCD_BIN." >&2
    exit 1
}

"$openocd_bin" \
    -f "$project_root/config/openocd_stm32f4.cfg" \
    -c "program \"$elf\" verify reset exit"
