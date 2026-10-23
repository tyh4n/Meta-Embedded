#!/usr/bin/env bash
set -euo pipefail

toolchain_version="15.3.rel2"
toolchain_archive="arm-gnu-toolchain-${toolchain_version}-darwin-arm64-arm-none-eabi.pkg"
toolchain_url="https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/${toolchain_version}/${toolchain_archive}"
toolchain_sha256="779972fa545f738bfbba2ff0ae44d5ad7510b00cfb414702974844ba625412f3"
install_root="${HOME}/.local/opt"
versioned_root="${install_root}/arm-gnu-toolchain-${toolchain_version}"
stable_root="${install_root}/arm-none-eabi"

if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "This bootstrap script is for macOS." >&2
    exit 1
fi

if [[ "$(uname -m)" != "arm64" ]]; then
    echo "This project script expects an Apple Silicon Mac (arm64)." >&2
    exit 1
fi

if ! command -v brew >/dev/null 2>&1; then
    echo "Homebrew is required: https://brew.sh" >&2
    exit 1
fi

for formula in cmake ninja open-ocd; do
    if ! brew list --formula "$formula" >/dev/null 2>&1; then
        brew install "$formula"
    fi
done

mkdir -p "$install_root"

if [[ ! -x "$stable_root/bin/arm-none-eabi-gcc" ]]; then
    existing=""
    if [[ -x "$versioned_root/bin/arm-none-eabi-gcc" ]]; then
        existing="$versioned_root"
    else
        while IFS= read -r candidate; do
            existing="$(cd "$(dirname "$candidate")/.." && pwd)"
            break
        done < <(find /Applications/ArmGNUToolchain -type f -name arm-none-eabi-gcc 2>/dev/null | head -n 1)
    fi

    if [[ -n "$existing" ]]; then
        if [[ -e "$stable_root" || -L "$stable_root" ]]; then
            echo "Refusing to replace existing path: $stable_root" >&2
            exit 1
        fi
        ln -s "$existing" "$stable_root"
    else
        tmpdir="$(mktemp -d "${TMPDIR:-/tmp}/meta-arm-toolchain.XXXXXX")"
        trap 'rm -rf "$tmpdir"' EXIT

        echo "Downloading Arm GNU Toolchain ${toolchain_version}"
        curl -L --fail --show-error \
            --output "$tmpdir/$toolchain_archive" \
            "$toolchain_url"
        actual_sha256="$(shasum -a 256 "$tmpdir/$toolchain_archive" | awk '{print $1}')"
        if [[ "$actual_sha256" != "$toolchain_sha256" ]]; then
            echo "Toolchain checksum mismatch." >&2
            exit 1
        fi

        pkgutil --expand-full "$tmpdir/$toolchain_archive" "$tmpdir/pkg"
        mv "$tmpdir/pkg/Payload" "$versioned_root"
        ln -s "$versioned_root" "$stable_root"
    fi
fi

export ARM_NONE_EABI_TOOLCHAIN_DIR="$stable_root/bin"
echo "ARM GNU Toolchain: $stable_root"

"$(cd "$(dirname "$0")" && pwd)/doctor.sh"
