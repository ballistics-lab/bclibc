#!/bin/bash
set -e

# Colors for output
GREEN='\033[032m'
RED='\033[031m'
NC='\033[0m' # No Color

echo -e "${GREEN}>>> Starting pre-commit validation...${NC}"

BUILD_DIR="build_check"

#1. Cleaning and assembly
echo -e "\n${GREEN}1. Cleaning and building from scratch...${NC}"
rm -rf "$BUILD_DIR"
mkdir "$BUILD_DIR"
cd "$BUILD_DIR"

# Compile with redirection of errors and warnings to a file
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc) 2> build_warnings.log || (cat build_warnings.log && exit 1)

# 2. Check for artifacts
echo -e "\n${GREEN}2. Checking build artifacts...${NC}"
if [ -f "libbclibc_core.a" ] && [ -f "libbclibc_ffi.so" ]; then
    echo -e "   [OK] Libraries found."
else
    echo -e "${RED}   [FAIL] Libraries missing!${NC}"
    exit 1
fi

# 3. Check for warnings (Zero Warning Policy)
echo -e "\n${GREEN}3. Validating warnings...${NC}"
if [ -s build_warnings.log ]; then
    echo -e "${RED}   [WARNING] Warnings detected during build:${NC}"
    cat build_warnings.log
    # You can uncomment the following line to block the commit if there are warnings
    # exit 1 
else
    echo -e "   [OK] No warnings detected."
fi

# 4. Checking the version (extracting from the binary)
echo -e "\n${GREEN}4. Checking version metadata...${NC}"

# Get the expected version from the generated header
EXPECTED_VERSION=$(grep "BCLIBC_VERSION " generated/bclibc/version.h | cut -d'"' -f2)

# Search the binary for a string that EXACTLY matches our expected version
# Use -x (exact match) in grep to avoid catching GLIBCXX_...
LIB_VERSION=$(strings libbclibc_ffi.so | grep -x "$EXPECTED_VERSION" | head -n 1 || true)

if [ "$LIB_VERSION" == "$EXPECTED_VERSION" ]; then
    echo -e "   [OK] Version matches: $LIB_VERSION"
else
    echo -e "${RED}   [FAIL] Version mismatch!${NC}"
    echo -e "          Binary found: '$LIB_VERSION'"
    echo -e "          Header expected: '$EXPECTED_VERSION'"
    exit 1
fi

# 5. Check symbols export (whitelist BCLIBCFFI_)
echo -e "\n${GREEN}5. Validating exported symbols...${NC}"
BAD_SYMBOLS=$(nm -D libbclibc_ffi.so | grep " T " | grep -v "BCLIBCFFI_" || true)

if [ -z "$BAD_SYMBOLS" ]; then
    echo -e "   [OK] Symbol visibility is correctly restricted."
else
    echo -e "${RED}   [FAIL] Leaked internal symbols found:${NC}"
    echo "$BAD_SYMBOLS"
    exit 1
fi

# 6. Check symlinks
echo -e "\n${GREEN}6. Checking SOVERSION symlinks...${NC}"
# Check for any numeric symlink (0 or 1)
if ls libbclibc_ffi.so.[0-9]* 1> /dev/null 2>&1; then
    echo -e "   [OK] Symlinks are generated."
else
    echo -e "${RED}   [WARNING] No SOVERSION symlinks found. Check set_target_properties in CMake.${NC}"
fi

cd ..
echo -e "\n${GREEN}>>> ALL CHECKS PASSED! Ready for commit.${NC}"
rm -rf "$BUILD_DIR"