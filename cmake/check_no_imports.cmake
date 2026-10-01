# Fails if the WebAssembly module imports anything at all (`cmake -DWASM=<file> -P check_no_imports.cmake`): a module
# with no import section can be instantiated with an empty import object, in any host. Walks the binary's sections
# (id, LEB128 size, payload) instead of searching for a WASI string, so an import of any module is caught.
file(READ "${WASM}" _hex HEX)
string(LENGTH "${_hex}" _len)

# 4 bytes of magic + 4 of version come first; every offset below counts hex digits, two per byte.
set(_pos 16)
set(_imports FALSE)
while(_pos LESS _len)
    string(SUBSTRING "${_hex}" ${_pos} 2 _id_hex)
    math(EXPR _id "0x${_id_hex}")
    math(EXPR _pos "${_pos} + 2")

    set(_size 0)
    set(_shift 0)
    set(_more TRUE)
    while(_more)
        string(SUBSTRING "${_hex}" ${_pos} 2 _byte_hex)
        math(EXPR _byte "0x${_byte_hex}")
        math(EXPR _pos "${_pos} + 2")
        math(EXPR _size "${_size} | ((${_byte} & 0x7f) << ${_shift})")
        math(EXPR _shift "${_shift} + 7")
        if(_byte LESS 128)
            set(_more FALSE)
        endif()
    endwhile()

    if(_id EQUAL 2) # the import section; a lone zero count (one byte) still imports nothing
        if(_size GREATER 1)
            set(_imports TRUE)
        endif()
    endif()
    math(EXPR _pos "${_pos} + ${_size} * 2")
endwhile()

if(_imports)
    string(HEX "wasi_snapshot_preview1" _wasi)
    string(FIND "${_hex}" "${_wasi}" _wasi_at)
    set(_hint "")
    if(NOT _wasi_at EQUAL -1)
        set(_hint " (from wasi_snapshot_preview1: a library that reads the environment, writes to stdio or exits pulled WASI in)")
    endif()
    message(FATAL_ERROR "${WASM} has an import section: it should import nothing${_hint}")
endif()
message(STATUS "${WASM}: no imports")
