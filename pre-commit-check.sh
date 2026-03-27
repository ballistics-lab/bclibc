#!/bin/bash
set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}>>> Starting pre-commit validation...${NC}"

BUILD_DIR="build_check"
OS=$(uname -s)

# 1. Cleaning and assembly
echo -e "\n${GREEN}1. Cleaning and building from scratch...${NC}"
rm -rf "$BUILD_DIR"
mkdir "$BUILD_DIR"
cd "$BUILD_DIR"

# Determine library names based on OS
if [[ "$OS" == "Darwin" ]]; then
    LIB_CORE="libbclibc_core.a"
    LIB_FFI="libbclibc_ffi.dylib"
    NM_CMD="nm -gU"
elif [[ "$OS" == "Linux" ]]; then
    LIB_CORE="libbclibc_core.a"
    LIB_FFI="libbclibc_ffi.so"
    NM_CMD="nm -D"
else
    echo -e "${RED}Unsupported OS for pre-commit check: $OS${NC}"
    exit 1
fi

# Compile with redirection of errors and warnings to a file
cmake -DCMAKE_BUILD_TYPE=Release ..
if [[ "$OS" == "Darwin" ]]; then
    make -j$(sysctl -n hw.ncpu) 2> build_warnings.log || (cat build_warnings.log && exit 1)
else
    make -j$(nproc) 2> build_warnings.log || (cat build_warnings.log && exit 1)
fi

# 2. Check for artifacts
echo -e "\n${GREEN}2. Checking build artifacts...${NC}"
if [ -f "$LIB_CORE" ] && [ -f "$LIB_FFI" ]; then
    echo -e "   ${GREEN}[OK]${NC} Libraries found."
else
    echo -e "   ${RED}[FAIL]${NC} Libraries missing!"
    exit 1
fi

# 3. Check for warnings (Zero Warning Policy)
echo -e "\n${GREEN}3. Validating warnings...${NC}"
if [ -s build_warnings.log ]; then
    echo -e "   ${YELLOW}[WARNING]${NC} Warnings detected during build:"
    cat build_warnings.log
    # Uncomment to block commit on warnings
    # exit 1
else
    echo -e "   ${GREEN}[OK]${NC} No warnings detected."
fi

# 4. Checking the version (extracting from the binary)
echo -e "\n${GREEN}4. Checking version metadata...${NC}"

# Get the expected version from the generated header
if [ -f "generated/bclibc/version.h" ]; then
    EXPECTED_VERSION=$(grep "BCLIBC_VERSION " generated/bclibc/version.h | cut -d'"' -f2)
else
    echo -e "   ${RED}[FAIL]${NC} version.h not found!"
    exit 1
fi

# Search the binary for version string
if [[ "$OS" == "Darwin" ]]; then
    LIB_VERSION=$(strings "$LIB_FFI" | grep -x "$EXPECTED_VERSION" | head -n 1 || true)
else
    LIB_VERSION=$(strings "$LIB_FFI" | grep -x "$EXPECTED_VERSION" | head -n 1 || true)
fi

if [ "$LIB_VERSION" == "$EXPECTED_VERSION" ]; then
    echo -e "   ${GREEN}[OK]${NC} Version matches: $LIB_VERSION"
else
    echo -e "   ${RED}[FAIL]${NC} Version mismatch!"
    echo -e "          Binary found: '$LIB_VERSION'"
    echo -e "          Header expected: '$EXPECTED_VERSION'"
    exit 1
fi

# 5. Check symbols export (whitelist BCLIBCFFI_)
echo -e "\n${GREEN}5. Validating exported symbols...${NC}"

if [[ "$OS" == "Darwin" ]]; then
    # macOS uses different symbol checking
    BAD_SYMBOLS=$($NM_CMD "$LIB_FFI" | grep -v "BCLIBCFFI_" | grep -E "^[0-9a-f]+ T " | cut -d' ' -f3 | grep -v "^_BCLIBCFFI_" || true)
else
    BAD_SYMBOLS=$($NM_CMD "$LIB_FFI" | grep " T " | grep -v "BCLIBCFFI_" | cut -d' ' -f3 || true)
fi

if [ -z "$BAD_SYMBOLS" ]; then
    echo -e "   ${GREEN}[OK]${NC} Symbol visibility is correctly restricted."
else
    echo -e "   ${RED}[FAIL]${NC} Leaked internal symbols found:"
    echo "$BAD_SYMBOLS"
    exit 1
fi

# 6. Check symlinks (Linux only)
if [[ "$OS" == "Linux" ]]; then
    echo -e "\n${GREEN}6. Checking SOVERSION symlinks...${NC}"
    if ls libbclibc_ffi.so.[0-9]* 1> /dev/null 2>&1; then
        echo -e "   ${GREEN}[OK]${NC} Symlinks are generated."
    else
        echo -e "   ${YELLOW}[WARNING]${NC} No SOVERSION symlinks found."
    fi
fi

cd ..
echo -e "\n${GREEN}>>> ALL CHECKS PASSED! Ready for commit.${NC}"
rm -rf "$BUILD_DIR"