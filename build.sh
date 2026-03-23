#!/bin/bash
set -e

BUILD_DIR="build"

# 1. Create bbuild dir
if [ ! -d "$BUILD_DIR" ]; then
  echo "--- Creating build directory ---"
  mkdir "$BUILD_DIR"
fi

cd "$BUILD_DIR"

# 2. Configuration CMake
echo "--- Configuring project ---"
cmake -DCMAKE_BUILD_TYPE=Release ..

# 3. Build targets (Core and FFI)
echo "--- Building targets (Core & FFI) ---"
make -j$(nproc)

# 4. Validating
echo "------------------------------------------"
if [ -f "libbclibc_ffi.so" ] && [ -f "libbclibc_core.a" ]; then
  echo "SUCCESS: Libraries generated successfully!"
  echo "Static core:  $BUILD_DIR/libbclibc_core.a"
  echo "Shared FFI:   $BUILD_DIR/libbclibc_ffi.so"
  
  # Optional: check symbols
  echo "--- Exported FFI symbols: ---"
  nm -D libbclibc_ffi.so | grep " T " | cut -d' ' -f3
else
  echo "ERROR: Build failed, libraries not found."
  exit 1
fi