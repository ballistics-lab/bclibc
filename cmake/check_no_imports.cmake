# Fails if the WebAssembly module imports from WASI (`cmake -DWASM=<file> -P check_no_imports.cmake`): a module that
# imports nothing can be instantiated with an empty import object, in any host.
file(READ "${WASM}" _hex HEX)
string(HEX "wasi_snapshot_preview1" _wasi)
string(FIND "${_hex}" "${_wasi}" _at)
if(NOT _at EQUAL -1)
    message(FATAL_ERROR "${WASM} imports from wasi_snapshot_preview1: it should import nothing (a library that "
        "reads the environment, writes to stdio or exits pulled WASI in)")
endif()
message(STATUS "${WASM}: no WASI imports")
