#!/bin/bash
set -e

BUILD_DIR="build"
OS=$(uname -s)

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}>>> Building bclibc for $OS...${NC}"

# 1. Create build directory
if [ ! -d "$BUILD_DIR" ]; then
  echo -e "${GREEN}--- Creating build directory ---${NC}"
  mkdir "$BUILD_DIR"
fi

cd "$BUILD_DIR"

# 2. Configuration CMake
echo -e "${GREEN}--- Configuring project ---${NC}"
if [[ "$OS" == "Darwin" ]]; then
    cmake -DCMAKE_BUILD_TYPE=Release ..
else
    cmake -DCMAKE_BUILD_TYPE=Release ..
fi

# 3. Build targets (Core and FFI)
echo -e "${GREEN}--- Building targets (Core & FFI) ---${NC}"
if [[ "$OS" == "Darwin" ]]; then
    make -j$(sysctl -n hw.ncpu)
else
    make -j$(nproc)
fi

# 4. Validating
echo -e "${GREEN}------------------------------------------${NC}"
if [[ "$OS" == "Darwin" ]]; then
    LIB_CORE="libbclibc_core.a"
    LIB_FFI="libbclibc_ffi.dylib"
elif [[ "$OS" == "Linux" ]]; then
    LIB_CORE="libbclibc_core.a"
    LIB_FFI="libbclibc_ffi.so"
else
    echo -e "${RED}Unsupported OS: $OS${NC}"
    exit 1
fi

if [ -f "$LIB_CORE" ] && [ -f "$LIB_FFI" ]; then
  echo -e "${GREEN}SUCCESS: Libraries generated successfully!${NC}"
  echo -e "Static core:  $BUILD_DIR/$LIB_CORE"
  echo -e "Shared FFI:   $BUILD_DIR/$LIB_FFI"
  
  # Optional: check symbols (Linux only)
  if [[ "$OS" == "Linux" ]]; then
    echo -e "${GREEN}--- Exported FFI symbols: ---${NC}"
    nm -D "$LIB_FFI" | grep " T " | cut -d' ' -f3 | grep "BCLIBCFFI_" || true
  fi
else
  echo -e "${RED}ERROR: Build failed, libraries not found.${NC}"
  exit 1
fi