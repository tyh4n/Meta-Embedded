#!/usr/bin/env bash
set -u

project_root="$(cd "$(dirname "$0")/.." && pwd)"
toolchain_dir="${ARM_NONE_EABI_TOOLCHAIN_DIR:-$HOME/.local/opt/arm-none-eabi/bin}"
failed=0

check_command() {
    local name="$1"
    local command_name="${2:-$1}"
    if command -v "$command_name" >/dev/null 2>&1; then
        printf '%-24s %s\n' "$name" "${command_name}: $(command -v "$command_name")"
    else
        printf '%-24s MISSING\n' "$name"
        failed=1
    fi
}

echo "Host"
uname -srm
echo
echo "Tools"
PATH="$toolchain_dir:$PATH"
check_command "CMake" cmake
check_command "Ninja" ninja
check_command "OpenOCD" openocd
check_command "ARM C compiler" arm-none-eabi-gcc
check_command "ARM C++ compiler" arm-none-eabi-g++
check_command "ARM GDB" arm-none-eabi-gdb

echo
echo "Versions"
cmake --version 2>/dev/null | head -n 1 || true
ninja --version 2>/dev/null || true
openocd --version 2>&1 | head -n 1 || true
arm-none-eabi-gcc --version 2>/dev/null | head -n 1 || true
arm-none-eabi-gdb --version 2>/dev/null | head -n 1 || true

echo
echo "OpenOCD configuration"
doctor_log="$(mktemp "${TMPDIR:-/tmp}/meta-openocd-doctor.XXXXXX")"
trap 'rm -f "$doctor_log"' EXIT
if openocd -f "$project_root/config/openocd_stm32f4.cfg" -c shutdown >"$doctor_log" 2>&1; then
    echo "config/openocd_stm32f4.cfg parsed successfully"
else
    cat "$doctor_log"
    failed=1
fi

if [[ -d /Applications/CLion.app ]]; then
    echo "CLion                   /Applications/CLion.app"
else
    echo "CLion                   not installed in /Applications"
fi

if [[ $failed -ne 0 ]]; then
    echo
    echo "Doctor found missing or invalid prerequisites."
    exit 1
fi

echo
echo "Environment looks ready."
