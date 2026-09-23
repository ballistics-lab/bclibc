#!/usr/bin/env bash
# Builds tiny_bclibc + wasm/tiny_bclibc_wasm.c into two import-free WebAssembly modules:
#
#   build/wasm/tiny_bclibc_dp.wasm   real_t = double
#   build/wasm/tiny_bclibc_sp.wasm   real_t = float (TINY_BCLIBC_SINGLE_PRECISION)
#
# Usage:
#   tiny_bclibc/build_wasm.sh              # OUT_DIR=/some/dir to write elsewhere
#
# Unlike ../build_wasm.sh (full bclibc + Emscripten JS glue, for Dart web), these modules need
# no JS runtime at all: `new WebAssembly.Instance(module, {})` is enough, so any bare
# WebAssembly host can run them (JavaScriptCore's JSContext on iOS/Pythonista, a browser,
# Node). The exported API is documented at the top of wasm/tiny_bclibc_wasm.c.
#
# Toolchain: needs a C compiler that targets wasm32 AND a wasm32 libm (tiny_bclibc calls
# sin/cos/atan2/pow/exp). Used in this order:
#   1. $CC if set (e.g. clang with a wasi-sdk sysroot: CC="clang --sysroot=/opt/wasi-sdk/share/wasi-sysroot")
#   2. `zig cc` if zig is on PATH
#   3. `python3 -m ziglang cc` (zig from PyPI: `pip install ziglang`) -- no system install needed
# The target is wasm32-wasi only to get a static libc/libm; nothing from WASI is called, and
# the script fails if a module ends up importing anything.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${OUT_DIR:-${SCRIPT_DIR}/build/wasm}"

if [[ -n "${CC:-}" ]]; then
    read -r -a CC_CMD <<< "${CC}"
    TARGET_FLAGS=(--target=wasm32-wasi)
elif command -v zig >/dev/null 2>&1; then
    CC_CMD=(zig cc)
    TARGET_FLAGS=(-target wasm32-wasi)
elif python3 -c "import ziglang" >/dev/null 2>&1; then
    CC_CMD=(python3 -m ziglang cc)
    TARGET_FLAGS=(-target wasm32-wasi)
else
    echo "No wasm32 C toolchain found: set CC (clang + wasi-sdk sysroot), install zig, or 'pip install ziglang'." >&2
    exit 1
fi

# Same version source as CMakeLists.txt: git describe, else a -dev fallback.
VERSION="$(git -C "${SCRIPT_DIR}" describe --tags --always 2>/dev/null || echo 1.0.0-dev)"
VERSION="${VERSION#v}"

mkdir -p "${OUT_DIR}"

build() {
    local out="$1"
    shift
    "${CC_CMD[@]}" "${TARGET_FLAGS[@]}" \
        -O2 -std=c99 -Wall -Wextra \
        -mexec-model=reactor -Wl,--no-entry -s \
        -I"${SCRIPT_DIR}/include" \
        -DTBW_VERSION="\"${VERSION}\"" \
        "$@" \
        "${SCRIPT_DIR}/wasm/tiny_bclibc_wasm.c" \
        -o "${out}"
    echo "Built ${out} ($(wc -c < "${out}") bytes)"
}

build "${OUT_DIR}/tiny_bclibc_dp.wasm"
build "${OUT_DIR}/tiny_bclibc_sp.wasm" -DTINY_BCLIBC_SINGLE_PRECISION

# The whole point of these modules is that a bare JS engine can instantiate them with `{}`.
if command -v node >/dev/null 2>&1; then
    node -e '
        const fs = require("fs");
        for (const f of process.argv.slice(1)) {
            const imports = WebAssembly.Module.imports(new WebAssembly.Module(fs.readFileSync(f)));
            if (imports.length) {
                console.error(f + " imports " + JSON.stringify(imports) + " - it must import nothing");
                process.exit(1);
            }
        }' "${OUT_DIR}/tiny_bclibc_dp.wasm" "${OUT_DIR}/tiny_bclibc_sp.wasm"
    echo "Import check passed: both modules import nothing."
else
    echo "node not found: skipped the no-imports check." >&2
fi
