# ruff: noqa

"""Run the natmod test suite (test_bclibc.py) against the FFI backend.

Usage (from repo root):
    micropython examples/tiny_bclibc_mp_ffi/test_mp_ffi.py

Environment variables:
    TINY_BCLIBC_SO        path to libtiny_bclibc.so
    TINY_BCLIBC_PRECISION single | double  (default: double)

Injects tiny_bclibc_mp_ffi as 'tiny_bclibc' before the test suite imports it,
so the full test_bclibc.py runs against the FFI backend without modification.
"""

import sys

if sys.implementation.name != "micropython":
    try:
        import pytest

        pytest.skip("micropython-only", allow_module_level=True)
    except ImportError:
        raise SystemExit(0)

_HERE = __file__.rsplit("/", 1)[0] if "/" in __file__ else "."

sys.path.insert(0, _HERE)
import tiny_bclibc_mp_ffi as _ffi_mod

sys.modules["tiny_bclibc"] = _ffi_mod

_test_path = _HERE + "/../../micropython-natmod/test_bclibc.py"
with open(_test_path) as _f:
    _src = _f.read()

exec(
    compile(_src, _test_path, "exec"), {"__file__": _test_path, "__name__": "__main__"}
)
