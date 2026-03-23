# BCLIBC: Pure C++ Ballistic Solver Engine

A high-performance ballistic trajectory solver featuring RK4 integration and Ridder's method for zero-finding. This project is structured as a modular C++ core with a dedicated C-compatible FFI (Foreign Function Interface) layer for seamless integration with Dart/Flutter, Python, or Rust.

---

## 🛠 Dependencies

To build the project, you need:

- CMake (3.13 or higher)
- GCC (C++17 support required) or Clang
- Make (Build automation utility)

---

## 🏗 Library Architecture

The build process generates two distinct artifacts in the `build/` directory:

- **libbclibc_core.a** — Static Core Library  
  Contains pure C++ logic, classes, and physics. Use this for C++ projects and unit testing.

- **libbclibc_ffi.so** — Shared FFI Library  
  A dynamic library with a stable C-API (prefixed with `BCLIBCFFI_`). Optimized for external language bindings.

---

## 🚀 Building the Project

### 1. Using Makefile (Recommended for Development)

The root `Makefile` provides shorthand commands for common tasks:

```bash
make        # Build everything (Core + FFI)
make core   # Build only the static core library
make ffi    # Build only the shared FFI library
make clean  # Remove the build directory
```

---

### 2. Using the Build Script (CI/CD)

For automated environments or clean builds:

```bash
chmod +x build.sh
./build.sh
```

---

## 🧪 Verifying FFI Exports

To ensure the symbol visibility script (`.version`) is working correctly and only the intended API is exposed, run:

```bash
nm -D build/libbclibc_ffi.so | grep " T "
```

You should only see symbols prefixed with `BCLIBCFFI_`.

---

## 📚 Integration with Dart/Flutter

* Copy `libbclibc_ffi.so` to your Flutter project's native assets folder
* Use `dart:ffi` to load the library
* *(Recommended)* Use the `ffigen` package with the header `include/bclibc/ffi/bclibc_ffi.h` to automatically generate Dart bindings

---

## 📁 Project Structure

```plaintext
.
├── build/                     # Build artifacts (CMake output)
│   ├── libbclibc_core.a      # Static core library
│   ├── libbclibc_ffi.so      # Shared FFI library
│   ├── CMakeCache.txt
│   ├── Makefile
│   └── CMakeFiles/           # Internal CMake build files
│
├── include/                  # Public headers
│   ├── bclibc.hpp
│   └── bclibc/
│       ├── base_types.hpp
│       ├── engine.hpp
│       ├── euler.hpp
│       ├── exceptions.hpp
│       ├── interp.hpp
│       ├── log.hpp
│       ├── rk4.hpp
│       ├── scope_guard.hpp
│       ├── traj_data.hpp
│       ├── traj_filter.hpp
│       ├── v3d.hpp
│       └── ffi/
│           └── bclibc_ffi.h  # C-compatible FFI API
│
├── src/                      # Implementation files
│   ├── base_types.cpp
│   ├── engine.cpp
│   ├── euler.cpp
│   ├── interp.cpp
│   ├── rk4.cpp
│   ├── traj_data.cpp
│   ├── traj_filter.cpp
│   └── ffi/
│       ├── bclibc_ffi.cpp
│       └── bclibc_ffi.version  # Symbol visibility control
│
├── CMakeLists.txt           # CMake configuration
├── Makefile                 # Dev shortcuts
├── build.sh                 # CI/CD build script
├── version.h.in
├── README.md
└── LICENSE
```

