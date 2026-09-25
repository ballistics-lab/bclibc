# The bare WebAssembly build (BCLIBC_WASM_BARE=ON, with cmake/zig-wasm32-wasi.cmake): the core and its flat C ABI
# (bclibc_ffi.h) in one module that imports nothing. Included by CMakeLists.txt, which stops after it.

set(BCLIBC_WASM_STACK_SIZE 1048576 CACHE STRING "Size of the shadow stack of the module, in bytes")
# The same ceiling as Emscripten's MAXIMUM_MEMORY (2 GiB), so the module grows as far as the Emscripten build did and
# no further; the memory is the module's own (exported), it needs nothing from the host.
set(BCLIBC_WASM_MAX_MEMORY 2147483648 CACHE STRING "Largest size the memory of the module may grow to, in bytes")

add_executable(bclibc_wasm
    ${BCLIBC_SOURCES}
    "${BCLIBC_SRC_DIR}/ffi/bclibc_ffi.cpp"
    "${BCLIBC_SRC_DIR}/wasm/bare_runtime.cpp"
)
target_include_directories(bclibc_wasm PRIVATE
    "${BCLIBC_INCLUDE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
)
target_compile_options(bclibc_wasm PRIVATE
    -Oz -flto -ffunction-sections -fdata-sections
    -ffp-contract=off  # strict IEEE arithmetic, as in the native build
)
target_link_options(bclibc_wasm PRIVATE
    -Oz -flto
    -mexec-model=reactor  # no main: the host calls the exports, after `_initialize`
    -Wl,--no-entry
    -Wl,--gc-sections
    -Wl,--export-dynamic  # every BCLIBCFFI_* function
    -Wl,--export=malloc   # the host puts arguments into the module's memory, and frees what it gets back
    -Wl,--export=free
    -Wl,-z,stack-size=${BCLIBC_WASM_STACK_SIZE}
    -Wl,--max-memory=${BCLIBC_WASM_MAX_MEMORY}
)
set_target_properties(bclibc_wasm PROPERTIES SUFFIX ".wasm" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_custom_command(TARGET bclibc_wasm POST_BUILD
    COMMAND ${CMAKE_COMMAND} -DWASM=$<TARGET_FILE:bclibc_wasm> -P "${CMAKE_CURRENT_LIST_DIR}/check_no_imports.cmake"
    COMMENT "Checking that bclibc_wasm imports nothing"
    VERBATIM
)
